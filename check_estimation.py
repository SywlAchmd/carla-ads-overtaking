"""Validasi estimasi jarak & kecepatan VisionPerception terhadap ground truth
simulator (rencana kerja bagian 10.4, 11.4).

Geometri S1 dengan kecepatan dipaksa tetap: ego 13,4 m/s, target 7,0 m/s, mulai
55 m di depan. Tidak ada MPC/FSM/planner -- yang diukur perception saja, dan
jarak menyapu 55 -> 10 m dalam satu run. Pembandingnya transform aktor dari
simulator, bukan `GroundTruthPerception`; keluaran GT ikut diukur terhadap
pembanding yang sama supaya acuannya sendiri ikut terverifikasi.

    python check_estimation.py
"""
import json
import math

import numpy as np

import config
import main
import perception
import sensors
import simulation
import yolopx

DETIK, X0, V_TARGET = 7.0, 55.0, 7.0


def relatif(ego, lain):
    """Posisi & kecepatan `lain` relatif ego, frame ego RH, dari titik asal aktor."""
    tf, v = ego.get_transform(), ego.get_velocity()
    yaw = math.radians(tf.rotation.yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    tl, vl = lain.get_transform().location, lain.get_velocity()
    dx, dy = tl.x - tf.location.x, tl.y - tf.location.y
    dvx, dvy = vl.x - v.x, vl.y - v.y
    return np.array([c * dx + s * dy, -(-s * dx + c * dy),
                     c * dvx + s * dvy, -(-s * dvx + c * dvy)])


def main_():
    params = json.load(open(config.VEHICLE_PARAMS_JSON))
    net = yolopx.YOLOPX()
    dt = config.FIXED_DELTA_SECONDS
    print(f'YOLOPX epoch {net.epoch}, {net.device}, half={net.half}')

    with simulation.carla_world() as world:
        ref, ref5 = main.siapkan_jalan(world)
        yaw_jalan = -math.degrees(np.mean(ref[2]))
        with simulation.ego_vehicle(world) as ego:
            print(f'pusat bounding box ego relatif titik asal aktor: '
                  f'x={ego.bounding_box.location.x:+.3f} m  '
                  f'(rear_axle_offset_x={params["rear_axle_offset_x"]:+.3f})')
            with sensors.RigKamera(world, ego, params) as rig:
                for _ in range(int(config.WARMUP_DETIK / dt)):
                    simulation.tick(world, [simulation.kecepatan(ego, config.V_REF, yaw_jalan)])
                    rig.ambil()
                ego_x = main.localization.PathFrame(ref).ego(
                    main.localization.CarlaGTLocalization(
                        ego, params['rear_axle_offset_x']).update()).x
                target = main.spawn_kendaraan(world, ref5, ego_x, X0, 0)
                lihat = perception.VisionPerception(net, rig)
                gt = perception.GroundTruthPerception(world, ego)

                baris = []
                try:
                    for k in range(int(DETIK / dt)):
                        simulation.tick(world, [
                            simulation.kecepatan(ego, config.V_REF, yaw_jalan),
                            simulation.kecepatan(target, V_TARGET, yaw_jalan)])
                        frame = rig.ambil()
                        v_obs = lihat.update(frame, dt, ego_v=config.V_REF)
                        g_obs = gt.update()
                        benar = relatif(ego, target)
                        pilih = lambda o: (o[np.argmin(np.abs(o[:, 0] - benar[0]))]
                                           if len(o) else np.full(4, np.nan))
                        baris.append(np.concatenate([[k * dt], benar,
                                                     pilih(v_obs), pilih(g_obs)]))
                finally:
                    target.destroy()

    a = np.array(baris)
    np.savez(f'{config.OUT_DIR}/check_estimation.npz', data=a)
    t, benar, vis, g = a[:, 0], a[:, 1:5], a[:, 5:9], a[:, 9:13]
    ada = ~np.isnan(vis[:, 0])

    print(f'\n{"t":>5}{"jarak benar":>12}{"vision x":>10}{"galat x":>9}'
          f'{"galat y":>9}{"vx vision":>11}{"galat vx":>10}{"GT x":>9}{"galat":>8}')
    for i in range(0, len(t), 10):
        b, v_, g_ = benar[i], vis[i], g[i]
        if np.isnan(v_[0]):
            print(f'{t[i]:>5.1f}{b[0]:>12.2f}{"tidak terdeteksi":>28}'
                  f'{"":>21}{g_[0]:>9.2f}{g_[0] - b[0]:>+8.2f}')
        else:
            print(f'{t[i]:>5.1f}{b[0]:>12.2f}{v_[0]:>10.2f}{v_[0] - b[0]:>+9.2f}'
                  f'{v_[1] - b[1]:>+9.2f}{v_[2]:>11.2f}{v_[2] - b[2]:>+10.2f}'
                  f'{g_[0]:>9.2f}{g_[0] - b[0]:>+8.2f}')

    # Acuan kecepatan = PERGESERAN posisi ground truth, bukan get_velocity():
    # get_velocity() berderau 0,28 m/s per tick saat kecepatan aktor dipaksa tiap
    # tick, sedangkan pergeserannya hanya 0,017 m/s. Memakai get_velocity() sebagai
    # acuan berarti menghukum estimator dengan derau milik alat ukur.
    benar_v = np.column_stack([np.gradient(benar[:, 0], dt), np.gradient(benar[:, 1], dt)])
    lahir = t[np.argmax(ada)]
    mapan = ada & (t > lahir + 1.0)

    def ringkas(nama, galat):
        print(f'  {nama:<30} bias {galat.mean():+7.3f}   RMS {np.sqrt((galat ** 2).mean()):6.3f}'
              f'   maks |{np.abs(galat).max():.3f}|')

    print(f'\nterdeteksi {ada.sum()}/{len(t)} tick, jarak '
          f'{benar[ada, 0].min():.1f}-{benar[ada, 0].max():.1f} m; '
          f'deteksi pertama {benar[np.argmax(ada), 0]:.1f} m')
    print('posisi (seluruh tick terdeteksi):')
    ringkas('x memanjang [m]', vis[ada, 0] - benar[ada, 0])
    ringkas('y melintang [m]', vis[ada, 1] - benar[ada, 1])
    print(f'kecepatan (mapan, t > {lahir + 1.0:.2f} s), acuan pergeseran posisi:')
    ringkas('vx [m/s]', vis[mapan, 2] - benar_v[mapan, 0])
    ringkas('vy [m/s]', vis[mapan, 3] - benar_v[mapan, 1])
    print('  sebagai pembanding, derau acuan itu sendiri:')
    print(f'    get_velocity() simulator   sd {benar[mapan, 2].std():.3f} m/s')
    print(f'    pergeseran posisi / dt     sd {benar_v[mapan, 0].std():.3f} m/s')
    print(f'    VisionPerception           sd {vis[mapan, 2].std():.3f} m/s')

    konv = [tt - lahir for tt, e in zip(t[ada], np.abs(vis[ada, 2] - benar_v[ada, 0]))
            if e < 0.1]
    print(f'konvergensi kecepatan: galat < 0,1 m/s setelah {konv[0]:.2f} s '
          f'({konv[0] / dt:.0f} frame) sejak track lahir')
    ringkas('GroundTruthPerception x [m]', g[ada, 0] - benar[ada, 0])


if __name__ == '__main__':
    main_()
