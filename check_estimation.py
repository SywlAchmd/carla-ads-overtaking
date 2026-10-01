"""Validasi estimasi jarak & kecepatan VisionPerception terhadap ground truth
simulator (rencana kerja bagian 10.4, 11.4).

Geometri S1 dengan kecepatan dipaksa tetap: ego 13,4 m/s, target 7,0 m/s, mulai
55 m di depan. Tidak ada MPC/FSM/planner -- yang diukur perception saja, dan
jarak menyapu 55 -> 10 m dalam satu run. Pembandingnya transform aktor dari
simulator, bukan `GroundTruthPerception`; keluaran GT ikut diukur terhadap
pembanding yang sama supaya acuannya sendiri ikut terverifikasi.

    python check_estimation.py
"""
import argparse
import json
import math

import numpy as np

import config
import main
import perception
import sensors
import simulation
import yolopx

SECONDS, X0, V_TARGET = 7.0, 55.0, 7.0


def relative(ego, other):
    """Posisi & kecepatan `other` relatif ego, frame ego RH, dari titik asal aktor."""
    tf, v = ego.get_transform(), ego.get_velocity()
    yaw = math.radians(tf.rotation.yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    tl, vl = other.get_transform().location, other.get_velocity()
    dx, dy = tl.x - tf.location.x, tl.y - tf.location.y
    dvx, dvy = vl.x - v.x, vl.y - v.y
    return np.array([c * dx + s * dy, -(-s * dx + c * dy),
                     c * dvx + s * dvy, -(-s * dvx + c * dvy)])


def main_():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weight', default=None, help='alternate YOLOPX checkpoint (section 30)')
    args = ap.parse_args()
    params = json.load(open(config.VEHICLE_PARAMS_JSON))
    net = yolopx.YOLOPX(args.weight)
    dt = config.FIXED_DELTA_SECONDS
    print(f'YOLOPX epoch {net.epoch}, {net.device}, half={net.half}')

    with simulation.carla_world() as world:
        ref, ref5 = main.prepare_road(world)
        yaw_road = -math.degrees(np.mean(ref[2]))
        with simulation.ego_vehicle(world) as ego:
            print(f'ego bounding box center relative to actor origin: '
                  f'x={ego.bounding_box.location.x:+.3f} m  '
                  f'(rear_axle_offset_x={params["rear_axle_offset_x"]:+.3f})')
            with sensors.CameraRig(world, ego, params) as rig:
                for _ in range(int(config.WARMUP_SECONDS / dt)):
                    simulation.tick(world, [simulation.velocity(ego, config.V_REF, yaw_road)])
                    rig.grab()
                ego_x = main.localization.PathFrame(ref).ego(
                    main.localization.CarlaGTLocalization(
                        ego, params['rear_axle_offset_x']).update()).x
                target = main.spawn_vehicles(world, ref5, ego_x, X0, 0)
                vision = perception.VisionPerception(net, rig)
                gt = perception.GroundTruthPerception(world, ego)

                rows = []
                try:
                    for k in range(int(SECONDS / dt)):
                        simulation.tick(world, [
                            simulation.velocity(ego, config.V_REF, yaw_road),
                            simulation.velocity(target, V_TARGET, yaw_road)])
                        frame = rig.grab()
                        v_obs = vision.update(frame, dt, ego_v=config.V_REF)
                        g_obs = gt.update()
                        truth = relative(ego, target)
                        choose = lambda o: (o[np.argmin(np.abs(o[:, 0] - truth[0]))]
                                           if len(o) else np.full(4, np.nan))
                        rows.append(np.concatenate([[k * dt], truth,
                                                     choose(v_obs), choose(g_obs)]))
                finally:
                    target.destroy()

    a = np.array(rows)
    np.savez(f'{config.OUT_DIR}/check_estimation.npz', data=a)
    t, truth, vis, g = a[:, 0], a[:, 1:5], a[:, 5:9], a[:, 9:13]
    present = ~np.isnan(vis[:, 0])

    print(f'\n{"t":>5}{"true dist":>12}{"vision x":>10}{"err x":>9}'
          f'{"err y":>9}{"vx vision":>11}{"err vx":>10}{"GT x":>9}{"err":>8}')
    for i in range(0, len(t), 10):
        b, v_, g_ = truth[i], vis[i], g[i]
        if np.isnan(v_[0]):
            print(f'{t[i]:>5.1f}{b[0]:>12.2f}{"not detected":>28}'
                  f'{"":>21}{g_[0]:>9.2f}{g_[0] - b[0]:>+8.2f}')
        else:
            print(f'{t[i]:>5.1f}{b[0]:>12.2f}{v_[0]:>10.2f}{v_[0] - b[0]:>+9.2f}'
                  f'{v_[1] - b[1]:>+9.2f}{v_[2]:>11.2f}{v_[2] - b[2]:>+10.2f}'
                  f'{g_[0]:>9.2f}{g_[0] - b[0]:>+8.2f}')

    # Acuan kecepatan = PERGESERAN posisi ground truth, bukan get_velocity():
    # get_velocity() berderau 0,28 m/s per tick saat kecepatan aktor dipaksa tiap
    # tick, sedangkan pergeserannya hanya 0,017 m/s. Memakai get_velocity() sebagai
    # acuan berarti menghukum estimator dengan derau milik alat ukur.
    true_v = np.column_stack([np.gradient(truth[:, 0], dt), np.gradient(truth[:, 1], dt)])
    born = t[np.argmax(present)]
    settled = present & (t > born + 1.0)

    def summary(name, err):
        print(f'  {name:<30} bias {err.mean():+7.3f}   RMS {np.sqrt((err ** 2).mean()):6.3f}'
              f'   max |{np.abs(err).max():.3f}|')

    print(f'\ndetected {present.sum()}/{len(t)} ticks, distance '
          f'{truth[present, 0].min():.1f}-{truth[present, 0].max():.1f} m; '
          f'first detection {truth[np.argmax(present), 0]:.1f} m')
    print('position (all detected ticks):')
    summary('x longitudinal [m]', vis[present, 0] - truth[present, 0])
    summary('y lateral [m]', vis[present, 1] - truth[present, 1])
    print(f'velocity (settled, t > {born + 1.0:.2f} s), reference = position difference:')
    summary('vx [m/s]', vis[settled, 2] - true_v[settled, 0])
    summary('vy [m/s]', vis[settled, 3] - true_v[settled, 1])
    print('  for comparison, the noise of the reference itself:')
    print(f'    get_velocity() simulator   sd {truth[settled, 2].std():.3f} m/s')
    print(f'    position difference / dt   sd {true_v[settled, 0].std():.3f} m/s')
    print(f'    VisionPerception           sd {vis[settled, 2].std():.3f} m/s')

    conv = [tt - born for tt, e in zip(t[present], np.abs(vis[present, 2] - true_v[present, 0]))
            if e < 0.1]
    print(f'velocity convergence: error < 0.1 m/s after {conv[0]:.2f} s '
          f'({conv[0] / dt:.0f} frames) since the track was born')
    summary('GroundTruthPerception x [m]', g[present, 0] - truth[present, 0])


if __name__ == '__main__':
    main_()
