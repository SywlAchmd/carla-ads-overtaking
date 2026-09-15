"""Tahap 1 bagian 3.2 + 3.3 -- validasi bicycle model terhadap plant CARLA.

Throttle konstan untuk warm-up lurus, lalu steer konstan 5 detik. Model diintegrasi
dengan kecepatan dan sudut roda yang benar-benar terjadi di CARLA, sehingga yang
diuji murni geometri: L dan konversi titik referensi. Pemetaan throttle -> akselerasi
urusan Tahap 5. Sekaligus memverifikasi SIDE_SIGN (bagian 0.3).

    python validate_model.py --scan   # cari spawn point di ruas lurus
    python validate_model.py          # jalankan validasi
"""
import argparse
import json
import math
import os
import sys

import carla
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import config
import control
import localization
import simulation

WHEELS = [carla.VehicleWheelLocation.FL_Wheel, carla.VehicleWheelLocation.FR_Wheel]


def bicycle_rk4(state, v, delta, L, dt):
    def f(s):
        _, _, psi = s
        return np.array([v * math.cos(psi), v * math.sin(psi), (v / L) * math.tan(delta)])

    k1 = f(state)
    k2 = f(state + 0.5 * dt * k1)
    k3 = f(state + 0.5 * dt * k2)
    k4 = f(state + dt * k3)
    return state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


def count_same_direction_lanes(wp):
    """(jumlah lajur searah di kanan, di kiri) -- menyalip ke kanan butuh kanan >= 1."""
    out = []
    for getter in ('get_right_lane', 'get_left_lane'):
        n, nb = 0, getattr(wp, getter)()
        while (nb is not None and nb.lane_type == carla.LaneType.Driving
               and (nb.lane_id > 0) == (wp.lane_id > 0)):
            n += 1
            nb = getattr(nb, getter)()
        out.append(n)
    return out


def scan_straight_spawns(world):
    probe_m, step = 150.0, 5.0
    carla_map = world.get_map()
    results = []
    for idx, sp in enumerate(carla_map.get_spawn_points()):
        wp = carla_map.get_waypoint(sp.location, project_to_road=True,
                                    lane_type=carla.LaneType.Driving)
        if wp is None:
            continue
        pts = list(simulation.walk_lane(wp, probe_m, step))
        if (len(pts) - 1) * step < probe_m:          # buntu / percabangan
            continue
        yaw0 = pts[0].transform.rotation.yaw
        worst = max(abs((p.transform.rotation.yaw - yaw0 + 180) % 360 - 180) for p in pts)
        widths = [p.lane_width for p in pts]
        n_right, n_left = count_same_direction_lanes(wp)
        results.append((worst, idx, n_right, n_left, min(widths), max(widths)))

    results.sort(key=lambda r: (r[0], -r[2]))          # lurus dulu, lalu lajur kanan terbanyak
    print(f"\n{'spawn':>6}{'maks |dyaw|':>13}{'lajur kanan':>13}{'lajur kiri':>12}"
          f"{'lebar lajur':>13}   (probe {probe_m:.0f} m)")
    print('-' * 68)
    for dyaw, idx, n_right, n_left, w_min, w_max in results[:10]:
        varies = '~' if w_max - w_min > 0.05 else ' '
        print(f'{idx:>6}{dyaw:>12.2f}°{n_right:>13}{n_left:>12}{w_min:>12.2f}{varies}')
    print(f"\nOVERTAKE_SIDE='{config.OVERTAKE_SIDE}', jadi pilih baris dengan lajur "
          f"{config.OVERTAKE_SIDE} >= 1 (idealnya 2+ supaya ada ruang abort).")
    print('Catat lebar lajurnya -- itu nilai LANE_WIDTH untuk planner (Tahap 3), bukan')
    print('asumsi 3.5 m. Tanda ~ berarti lebarnya berubah di sepanjang probe; hindari,')
    print('referensi lateralnya jadi bergeser.')


def uji_steer(world, ego, params):
    """Verifikasi steer_command: perintahkan delta, baca sudut roda nyata.

    Kecepatan ditahan dengan set_target_velocity TIAP tick -- kalau hanya diset
    sekali lalu diandalkan throttle, mobil melambat dan kurva dievaluasi pada
    kecepatan yang salah.
    """
    print(f"{'v minta':>9}{'v nyata':>9}{'delta minta':>13}{'delta nyata':>13}"
          f"{'error':>9}{'  |  rumus 7.5':>15}{'jadinya':>10}")
    print('-' * 82)
    for v_target in (5.6, 13.9):                      # 20 dan 50 km/jam
        for delta in (0.02, 0.05, 0.10, 0.20):
            for i in range(30):                       # tahan kecepatan + tunggu roda mapan
                yaw = math.radians(ego.get_transform().rotation.yaw)
                v = ego.get_velocity()
                v_now = v.x * math.cos(yaw) + v.y * math.sin(yaw)
                simulation.tick(world, [
                    simulation.kecepatan(ego, v_target),
                    carla.command.ApplyVehicleControl(ego.id, carla.VehicleControl(
                        throttle=0.0, steer=control.steer_command(delta, v_now, params)))])
            # negasi ke right-handed, sama seperti wheel_delta di run():
            # membandingkan dengan sudut mentah CARLA menyembunyikan salah tanda
            nyata = -math.radians(sum(ego.get_wheel_steer_angle(w) for w in WHEELS)
                                  / len(WHEELS))
            salah = delta / params['delta_max']
            print(f'{v_target * 3.6:>6.0f} km/j{v_now * 3.6:>8.1f}{delta:>13.4f}'
                  f'{nyata:>13.4f}{nyata - delta:>+9.4f}{salah:>15.4f}'
                  f'{salah * params["delta_max_phys"]:>10.4f}')


def run(world, ego, rear_offset_x):
    dt = config.FIXED_DELTA_SECONDS
    carla_map = world.get_map()
    loc = localization.CarlaGTLocalization(ego, rear_offset_x)
    control = carla.VehicleControl(throttle=config.VALIDATION_THROTTLE)

    kirim = [simulation.kecepatan(ego, config.VALIDATION_SPEED)]
    for _ in range(int(config.VALIDATION_WARMUP / dt)):
        simulation.tick(world, kirim + [carla.command.ApplyVehicleControl(ego.id, control)])
        kirim = []

    control.steer = config.VALIDATION_STEER
    log, off_road = [], 0
    for k in range(int(config.VALIDATION_DURATION / dt)):
        simulation.tick(world, [carla.command.ApplyVehicleControl(ego.id, control)])
        s = loc.update()
        delta = -math.radians(sum(ego.get_wheel_steer_angle(w) for w in WHEELS) / len(WHEELS))
        log.append(dict(t=k * dt, x=s.x, y=s.y, yaw=s.yaw, v=s.v,
                        yaw_rate=s.yaw_rate, delta=delta))
        off_road += carla_map.get_waypoint(ego.get_location(),
                                           project_to_road=False) is None

    if off_road:
        print(f'PERINGATAN: {off_road * dt:.1f} s dari {config.VALIDATION_DURATION:.0f} s '
              'kendaraan keluar area drivable. Gesekan di luar aspal mengacaukan\n'
              '            perbandingan; turunkan VALIDATION_STEER atau VALIDATION_DURATION.')
    return log


def integrate(log, L, dt):
    """Integrasi dari state awal fase kemudi, input identik dengan plant."""
    s = np.array([log[0]['x'], log[0]['y'], log[0]['yaw']])
    out = [s.copy()]
    for r in log[:-1]:
        s = bicycle_rk4(s, r['v'], r['delta'], L, dt)
        out.append(s.copy())
    return np.array(out)


def analyse(log, model, L):
    t = np.array([r['t'] for r in log])
    plant = np.array([[r['x'], r['y'], r['yaw']] for r in log])
    err = np.hypot(plant[:, 0] - model[:, 0], plant[:, 1] - model[:, 1])
    yaw_err = np.degrees((plant[:, 2] - model[:, 2] + np.pi) % (2 * np.pi) - np.pi)

    print(f"\n{'t (s)':>7}{'|err| posisi (m)':>19}{'err yaw (deg)':>16}")
    print('-' * 42)
    for target in range(1, int(t[-1]) + 1):
        i = int(np.argmin(np.abs(t - target)))
        print(f'{t[i]:>7.2f}{err[i]:>19.3f}{yaw_err[i]:>16.2f}')
    print(f'\nError posisi maksimum : {err.max():.3f} m')
    print(f'Error posisi akhir    : {err[-1]:.3f} m')
    print(f'Kecepatan rata-rata   : {np.mean([r["v"] for r in log]):.2f} m/s')
    print(f'delta rata-rata       : {np.mean([r["delta"] for r in log]):.4f} rad '
          f'(steer_cmd = {config.VALIDATION_STEER:+.2f})')

    v = np.array([r['v'] for r in log])
    a_lat = (v ** 2 * np.tan(np.abs([r['delta'] for r in log])) / L).max()
    print(f'Rentang kecepatan     : {v.min() * 3.6:.0f} - {v.max() * 3.6:.0f} km/jam')
    print(f'a_lat maksimum        : {a_lat:.2f} m/s²')

    yr_log = np.array([r['yaw_rate'] for r in log])
    yr_fd = np.gradient(np.unwrap(plant[:, 2]), t)
    print(f'\nyaw_rate (Tahap 2)')
    print(f'  terlog dari CARLA   : {yr_log.mean():+.4f} rad/s (rata-rata)')
    print(f'  d(yaw)/dt numerik   : {yr_fd.mean():+.4f} rad/s')
    ok = np.mean(np.abs(yr_log - yr_fd)) < 0.5 * np.mean(np.abs(yr_fd))
    print('  -> ' + ('tanda & besaran COCOK' if ok else
                     'TIDAK COCOK -- periksa negasi yaw_rate di localization.py'))

    if err[-1] > 1.0 and a_lat > 3.0:
        print(f'\nCATATAN: a_lat {a_lat:.1f} m/s² sudah di luar amplop kinematic bicycle '
              '(slip ban).\n         Error sebesar ini wajar; bandingkan dengan run '
              'a_lat rendah sebelum menuduh bug.')
    elif err[-1] > 1.0:
        print('\nPERIKSA: error akhir > 1 m padahal a_lat rendah. Tersangka utama: '
              'rear_axle_offset_x\n         salah tanda, L salah, atau konversi yaw belum negatif.')

    check_side_sign(plant)
    return err


def check_side_sign(plant):
    """steer CARLA positif = belok kanan. Tentukan tanda y yang sepadan di frame RH."""
    psi0 = plant[0, 2]
    d = plant[-1, :2] - plant[0, :2]
    lateral = -math.sin(psi0) * d[0] + math.cos(psi0) * d[1]
    empirical = int(np.sign(lateral))

    print('\nSIDE_SIGN (bagian 0.3)')
    print(f'  steer_cmd {config.VALIDATION_STEER:+.2f} (kanan) -> deviasi lateral '
          f'{lateral:+.2f} m di frame right-handed')
    print(f'  maka kanan = y {"negatif" if empirical < 0 else "positif"}, '
          f"SIDE_SIGN untuk OVERTAKE_SIDE='right' = {empirical:+d}")

    actual = empirical if config.OVERTAKE_SIDE == 'right' else -empirical
    if actual == config.SIDE_SIGN:
        print(f'  config.SIDE_SIGN = {config.SIDE_SIGN:+d} -> COCOK')
    else:
        print(f'  config.SIDE_SIGN = {config.SIDE_SIGN:+d} -> SALAH, ganti jadi {actual:+d}')


def plot(log, model, err, path):
    t = np.array([r['t'] for r in log])
    plant = np.array([[r['x'], r['y']] for r in log])

    fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
    ax[0].plot(plant[:, 0], plant[:, 1], label='Plant (CARLA)', lw=2)
    ax[0].plot(model[:, 0], model[:, 1], '--', label='Kinematic bicycle', lw=2)
    ax[0].set_xlabel('X (m)'); ax[0].set_ylabel('Y (m), right-handed')
    ax[0].set_title('Lintasan'); ax[0].axis('equal'); ax[0].legend(); ax[0].grid(alpha=.3)

    ax[1].plot(t, err, color='crimson')
    ax[1].set_xlabel('t (s)'); ax[1].set_ylabel('|error| posisi (m)')
    ax[1].set_title('Model mismatch'); ax[1].grid(alpha=.3)

    ax[2].plot(t, [r['v'] for r in log], label='v (m/s)')
    ax[2].plot(t, [r['delta'] for r in log], label='delta (rad)')
    ax[2].set_xlabel('t (s)'); ax[2].set_title('Input')
    ax[2].legend(); ax[2].grid(alpha=.3)

    fig.suptitle(f'Validasi prediction model -- {config.EGO_BP}')
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f'\nGrafik  : {path}')


def save_csv(log, model, path):
    data = np.column_stack([[r['t'] for r in log], [r['delta'] for r in log],
                            [r['v'] for r in log], [r['yaw_rate'] for r in log],
                            [[r['x'], r['y'], r['yaw']] for r in log], model])
    np.savetxt(path, data, delimiter=',', fmt='%.6f', comments='',
               header='t,delta,v,yaw_rate,x,y,yaw,x_model,y_model,yaw_model')
    print(f'Log CSV : {path}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scan', action='store_true',
                    help='daftar spawn point pada ruas lurus, lalu keluar')
    ap.add_argument('--steer', action='store_true',
                    help='verifikasi steer_command terhadap sudut roda nyata')
    args = ap.parse_args()

    if not os.path.exists(config.VEHICLE_PARAMS_JSON):
        sys.exit('Jalankan extract_params.py dulu.')
    with open(config.VEHICLE_PARAMS_JSON) as f:
        params = json.load(f)

    with simulation.carla_world() as world:
        if args.scan:
            scan_straight_spawns(world)
            return
        with simulation.ego_vehicle(world) as ego:
            if args.steer:
                uji_steer(world, ego, params)
                return
            log = run(world, ego, params['rear_axle_offset_x'])

    model = integrate(log, params['L'], config.FIXED_DELTA_SECONDS)
    err = analyse(log, model, params['L'])
    plot(log, model, err, os.path.join(config.OUT_DIR, 'model_validation.png'))
    save_csv(log, model, os.path.join(config.OUT_DIR, 'model_validation.csv'))


if __name__ == '__main__':
    main()
