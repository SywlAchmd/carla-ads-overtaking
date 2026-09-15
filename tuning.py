"""Harness tuning MPC — bagian 7.6 langkah 1-3.

Step response lateral pada acuan GARIS LURUS, tanpa halangan dan tanpa FSM.
Tuning di skenario penuh tidak bisa dipakai: mengubah satu bobot ikut mengubah
kapan FSM terpicu dan kandidat mana yang lolos, sehingga efek bobot tidak bisa
dipisahkan dari efek skenario.

    python tuning.py                         # konfigurasi sekarang
    python tuning.py --sweep RD_DELTA 5,20,80,200
    python tuning.py --sweep Q_Y 5,20,60 --step -1.5

Urutan bagian 7.6: (1) Q_Y dan Q_PSI sampai tracking rapat, (2) RD_DELTA sampai
kemudi tidak bergetar, (3) Q_V dan R_A untuk tracking kecepatan.
"""
import argparse
import json

import carla
import numpy as np

import config
import control
import localization
import main
import simulation

PETA = {'Q_X': ('Q', 0), 'Q_Y': ('Q', 1), 'Q_PSI': ('Q', 2), 'Q_V': ('Q', 3),
        'R_A': ('R', 0), 'R_DELTA': ('R', 1),
        'RD_A': ('Rd', 0), 'RD_DELTA': ('Rd', 1),
        'QF': ('Qf_scale', None), 'RHO': ('rho', None),
        'PI_KP': ('kp', None), 'PI_KI': ('ki', None)}


def bobot_dengan(nama, nilai):
    """Salin bobot config lalu timpa satu entri."""
    kunci, idx = PETA[nama]
    b = dict(Q=config.MPC_Q, Qf_scale=config.MPC_QF_SCALE, R=config.MPC_R,
             Rd=config.MPC_RD, rho=config.MPC_RHO,
             kp=config.THROTTLE_KP, ki=config.THROTTLE_KI)
    if idx is None:
        b[kunci] = nilai
    else:
        t = list(b[kunci])
        t[idx] = nilai
        b[kunci] = tuple(t)
    return b


def step_response(world, params, ref, bobot, step, detik):
    """Ego mapan di y=0, lalu acuan dilompatkan ke y=step pada t=0.

    Ego di-spawn ULANG tiap konfigurasi. set_transform hanya memindahkan posisi;
    putaran roda dan kompresi suspensi terbawa dari konfigurasi sebelumnya,
    sehingga hasilnya bergantung urutan sapuan. Terbukti: ki=0,1 memberi
    v_err 0,042 saat dijalankan pertama dan 0,263 saat dijalankan ketiga.
    """
    frame = localization.PathFrame(ref)
    mpc = control.MPCController(params, bobot=bobot)

    with simulation.ego_vehicle(world) as ego_actor:
        loc = localization.CarlaGTLocalization(ego_actor,
                                               params['rear_axle_offset_x'])
        simulation.tick(world, [simulation.kecepatan(ego_actor, config.EGO_V0)])
        return _jalankan(world, ego_actor, frame, loc, mpc, step, detik)


def _jalankan(world, ego_actor, frame, loc, mpc, step, detik):
    dt = config.FIXED_DELTA_SECONDS
    a_filt, v_prev, log, kirim = 0.0, None, [], []
    for k in range(-int(config.WARMUP_DETIK / dt), int(detik / dt)):
        simulation.tick(world, kirim)
        ego = frame.ego(loc.update())
        a_mentah = 0.0 if v_prev is None else (ego.v - v_prev) / dt
        v_prev = ego.v
        a_filt += config.A_FILTER_ALPHA * (
            float(np.clip(a_mentah, config.A_MIN, config.A_MAX)) - a_filt)

        y_target = step if k >= 0 else 0.0            # lompatan acuan tepat di t=0
        cmd = mpc.compute(ego.as_vector(),
                          main.xref_tahan(ego, y_target, config.V_REF), a_filt)
        kirim = [carla.command.ApplyVehicleControl(ego_actor.id, main.to_carla(cmd))]
        if k >= 0:
            log.append([k * dt, ego.y, cmd.delta_cmd, cmd.solve_time_ms,
                        float(cmd.solver_ok), ego.v])
    return np.array(log)


def metrik(log, step):
    t, y, delta, ms, ok, v = log.T
    ev = v - config.V_REF
    e = y - step                                      # harus meluruh ke nol
    besar = abs(step)
    # overshoot = sejauh mana y melewati target, searah gerakan
    overshoot = max(0.0, float((np.sign(step) * (y - step)).max())) / besar * 100
    luar = np.nonzero(np.abs(e) > 0.05 * besar)[0]
    settling = float(t[luar[-1]] + (t[1] - t[0])) if len(luar) else 0.0
    akhir = t >= t[-1] - 1.0
    # Chatter hanya bermakna SETELAH mapan: ramp awal yang cepat itu koreksi
    # yang sah, bukan getaran. Merata-ratakan seluruh run mencampur keduanya.
    d_mapan = delta[akhir]
    return dict(overshoot=overshoot, settling=settling,
                sisa=float(np.abs(e[akhir]).mean()),
                jitter=float(np.abs(np.diff(d_mapan)).mean()),
                jitter_penuh=float(np.abs(np.diff(delta)).mean()),
                delta_maks=float(np.abs(delta).max()),
                v_err=float(np.abs(ev).mean()), v_min=float(v.min()),
                v_sisa=float(np.abs(ev[akhir]).mean()),
                solve=float(ms.mean()), gagal=int((ok == 0).sum()))


def baris(label, m):
    print(f'{label:>12}{m["overshoot"]:>11.1f}{m["settling"]:>11.2f}'
          f'{m["sisa"]:>11.3f}{m["jitter"]*1e3:>13.4f}'
          f'{m["jitter_penuh"]*1e3:>12.3f}{m["delta_maks"]:>11.4f}'
          f'{m["v_err"]:>10.3f}{m["v_sisa"]:>10.3f}{m["v_min"]*3.6:>9.1f}'
          f'{m["solve"]:>9.1f}{m["gagal"]:>7}')


def main_():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sweep', nargs=2, metavar=('BOBOT', 'NILAI'),
                    help=f'nama bobot ({", ".join(PETA)}) dan daftar nilai')
    ap.add_argument('--step', type=float, default=-1.0, help='lompatan lateral (m)')
    ap.add_argument('--detik', type=float, default=6.0)
    args = ap.parse_args()

    if args.sweep:
        nama = args.sweep[0].upper()
        if nama not in PETA:
            raise SystemExit(f'bobot tidak dikenal: {nama}. Pilihan: {", ".join(PETA)}')
        konfig = [(f'{nama}={float(v):g}', bobot_dengan(nama, float(v)))
                  for v in args.sweep[1].split(',')]
    else:
        konfig = [('config', None)]

    params = json.load(open(config.VEHICLE_PARAMS_JSON))
    with simulation.carla_world() as world:
        ref, _ = main.siapkan_jalan(world)
        print(f'Step response lateral {args.step:+.2f} m, acuan garis lurus, '
              f'tanpa halangan, {config.V_REF * 3.6:.0f} km/jam')
        print('ego di-spawn ulang tiap konfigurasi\n')
        print(f'{"konfigurasi":>12}{"overshoot%":>11}{"settling s":>11}'
              f'{"sisa m":>11}{"chatter mrad":>13}{"jitter tot":>12}'
              f'{"delta maks":>11}{"v err":>10}{"v sisa":>10}{"v min":>9}'
              f'{"solve":>9}{"gagal":>7}')
        print('-' * 124)
        for label, bobot in konfig:
            log = step_response(world, params, ref, bobot, args.step, args.detik)
            baris(label, metrik(log, args.step))
    print('\nbagian 7.6: kemudi bergerigi hampir selalu berarti RD_DELTA terlalu kecil')


if __name__ == '__main__':
    main_()
