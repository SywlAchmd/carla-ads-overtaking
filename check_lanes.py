"""Validasi geometri lajur YOLOPX terhadap peta HD simulator (bagian 28).

Yang diuji `lanes.from_mask`: lebar lajur, simpangan ego dari tengah lajur,
dan sudut hadap -- ketiganya diukur dari kepala segmentasi lajur, lalu
dibandingkan dengan `world.get_map()`. Peta HD di sini adalah ALAT UKUR, bukan
masukan kendali; pemakaian ground truth yang sah, sama seperti
`check_estimation.py`.

Pose ego DITETAPKAN tiap tick, bukan hasil fisika. Alasannya: yang diuji
perception, jadi simpangan dan sudut hadap harus diketahui persis dan disapu
sengaja. Kalau ego selalu di tengah lajur, `lane_dev` yang selalu mengembalikan
nol pun akan tampak benar.

    python check_lanes.py
    python check_lanes.py --seconds 16
"""
import argparse
import json
import math

import numpy as np

import config
import lanes
import localization
import main
import sensors
import simulation
import yolopx

# (simpangan melintang m, sudut hadap deg) yang disapu. Nol dulu supaya kasus
# termudah terlihat lebih dahulu, lalu menyimpang ke kedua arah.
SWEEP = ((0.0, 0.0), (0.6, 0.0), (-0.6, 0.0), (1.2, 0.0), (-1.2, 0.0),
          (0.0, 4.0), (0.0, -4.0), (0.8, 6.0))


def _summary(name, err, unit='m'):
    err = np.asarray(err, dtype=float)
    err = err[np.isfinite(err)]
    if not len(err):
        print(f'  {name:<32} no samples')
        return
    print(f'  {name:<32} bias {err.mean():+7.3f}   RMS {np.sqrt((err ** 2).mean()):6.3f}'
          f'   max |{np.abs(err).max():.3f}| {unit}   n={len(err)}')


def main_():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=16.0)
    ap.add_argument('--weight', default=None,
                    help='alternate YOLOPX checkpoint, e.g. to compare per-marking versus '
                         'continuous lane annotation (section 30)')
    args = ap.parse_args()

    params = json.load(open(config.VEHICLE_PARAMS_JSON))
    net = yolopx.YOLOPX(args.weight)
    dt = config.FIXED_DELTA_SECONDS
    print(f'YOLOPX epoch {net.epoch}, {net.device}, half={net.half}')
    print(f'lane width according to the map: {config.LANE_WIDTH:.2f} m')

    rows = []
    with simulation.carla_world() as world:
        ref, ref5 = main.prepare_road(world)
        s_ref, x_ref, y_ref, psi_ref, z_ref = ref5
        with simulation.ego_vehicle(world) as ego:
            with sensors.CameraRig(world, ego, params) as rig:
                for _ in range(int(config.WARMUP_SECONDS / dt)):
                    simulation.tick(world)
                    rig.grab()

                n = int(args.seconds / dt)
                s0 = 20.0
                for k in range(n):
                    d_set, yaw_set = SWEEP[min(k * len(SWEEP) // n, len(SWEEP) - 1)]
                    s = s0 + config.V_REF * k * dt
                    psi = float(np.interp(s, s_ref, psi_ref))
                    # sama seperti main.spawn_vehicles: geser tegak lurus jalan,
                    # lalu ubah ke frame CARLA yang kidal
                    simulation.tick(world, [simulation.pose(
                        ego,
                        float(np.interp(s, s_ref, x_ref)) - d_set * math.sin(psi),
                        -(float(np.interp(s, s_ref, y_ref)) + d_set * math.cos(psi)),
                        float(np.interp(s, s_ref, z_ref)) + 0.3,
                        -math.degrees(psi + math.radians(yaw_set)))])
                    image = rig.grab()

                    _, da, ll = net.infer(sensors.rgb_array(image['rgb']))
                    g = lanes.from_mask(ll, (config.CAMERA_HEIGHT, config.CAMERA_WIDTH),
                                        drivable=da)       # sama dengan jalur kendali
                    if g is None:
                        rows.append([k * dt, d_set, yaw_set, np.nan, np.nan, np.nan, 0, 0])
                        continue
                    width = g.lane_width
                    dev = g.lane_dev
                    rows.append([
                        k * dt, d_set, yaw_set,
                        np.nan if width is None else width,
                        np.nan if dev is None else dev,
                        math.degrees(g.yaw), len(g.offset), g.n_pixels])

    a = np.array(rows, dtype=float)
    np.savez(f'{config.OUT_DIR}/check_lanes.npz', data=a)
    t, d_set, yaw_set, width, dev, yaw_meas, n_lines, n_px = a.T
    present = np.isfinite(dev) & np.isfinite(width)

    print(f'\nlane geometry read: {int(present.sum())}/{len(t)} frame '
          f'({100 * present.mean():.0f}%)')
    if present.sum():
        print(f'lines detected: mode {int(np.bincount(n_lines.astype(int)).argmax())}, '
              f'range {int(n_lines.min())}-{int(n_lines.max())}; '
              f'mean pixels {n_px[present].mean():.0f}')

    print('\naccuracy against the HD map:')
    _summary('lane width [m]', width[present] - config.LANE_WIDTH)
    _summary('ego offset from center [m]', dev[present] - d_set[present])
    _summary('heading angle [deg]', yaw_meas[present] - yaw_set[present], 'deg')

    print('\nper sweep point:')
    print(f'  {"d set":>7}{"yaw set":>9}{"n":>5}{"width":>9}{"err":>8}'
          f'{"dev meas":>10}{"err":>8}{"yaw meas":>10}{"err":>8}')
    for key in SWEEP:
        m = present & (d_set == key[0]) & (yaw_set == key[1])
        if not m.sum():
            continue
        print(f'  {key[0]:>7.1f}{key[1]:>9.1f}{int(m.sum()):>5}'
              f'{width[m].mean():>9.3f}{width[m].mean() - config.LANE_WIDTH:>+8.3f}'
              f'{dev[m].mean():>10.3f}{(dev[m] - d_set[m]).mean():>+8.3f}'
              f'{yaw_meas[m].mean():>10.2f}{(yaw_meas[m] - yaw_set[m]).mean():>+8.2f}')
    print(f'\nlog: {config.OUT_DIR}/check_lanes.npz')


if __name__ == '__main__':
    main_()
