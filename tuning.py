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

MAP = {'Q_X': ('Q', 0), 'Q_Y': ('Q', 1), 'Q_PSI': ('Q', 2), 'Q_V': ('Q', 3),
        'R_A': ('R', 0), 'R_DELTA': ('R', 1),
        'RD_A': ('Rd', 0), 'RD_DELTA': ('Rd', 1),
        'QF': ('Qf_scale', None), 'RHO': ('rho', None),
        'PI_KP': ('kp', None), 'PI_KI': ('ki', None)}


def weight_with(name, value):
    """Salin bobot config lalu timpa satu entri."""
    key, idx = MAP[name]
    b = dict(Q=config.MPC_Q, Qf_scale=config.MPC_QF_SCALE, R=config.MPC_R,
             Rd=config.MPC_RD, rho=config.MPC_RHO,
             kp=config.THROTTLE_KP, ki=config.THROTTLE_KI)
    if idx is None:
        b[key] = value
    else:
        t = list(b[key])
        t[idx] = value
        b[key] = tuple(t)
    return b


def step_response(world, params, ref, weight, step, seconds):
    """Ego mapan di y=0, lalu acuan dilompatkan ke y=step pada t=0.

    Ego di-spawn ULANG tiap konfigurasi. set_transform hanya memindahkan posisi;
    putaran roda dan kompresi suspensi terbawa dari konfigurasi sebelumnya,
    sehingga hasilnya bergantung urutan sapuan. Terbukti: ki=0,1 memberi
    v_err 0,042 saat dijalankan pertama dan 0,263 saat dijalankan ketiga.
    """
    frame = localization.PathFrame(ref)
    mpc = control.MPCController(params, weight=weight)

    with simulation.ego_vehicle(world) as ego_actor:
        loc = localization.CarlaGTLocalization(ego_actor,
                                               params['rear_axle_offset_x'])
        simulation.tick(world, [simulation.velocity(ego_actor, config.EGO_V0)])
        return _run(world, ego_actor, frame, loc, mpc, step, seconds)


def _run(world, ego_actor, frame, loc, mpc, step, seconds):
    dt = config.FIXED_DELTA_SECONDS
    a_filt, v_prev, log, send = 0.0, None, [], []
    for k in range(-int(config.WARMUP_SECONDS / dt), int(seconds / dt)):
        simulation.tick(world, send)
        ego = frame.ego(loc.update())
        a_raw = 0.0 if v_prev is None else (ego.v - v_prev) / dt
        v_prev = ego.v
        a_filt += config.A_FILTER_ALPHA * (
            float(np.clip(a_raw, config.A_MIN, config.A_MAX)) - a_filt)

        y_target = step if k >= 0 else 0.0            # lompatan acuan tepat di t=0
        cmd = mpc.compute(ego.as_vector(),
                          main.xref_hold(ego, config.V_REF, y_target), a_filt)
        send = [carla.command.ApplyVehicleControl(ego_actor.id, main.to_carla(cmd))]
        if k >= 0:
            log.append([k * dt, ego.y, cmd.delta_cmd, cmd.solve_time_ms,
                        float(cmd.solver_ok), ego.v])
    return np.array(log)


def metric(log, step):
    t, y, delta, ms, ok, v = log.T
    ev = v - config.V_REF
    e = y - step                                      # harus meluruh ke nol
    large = abs(step)
    # overshoot = sejauh mana y melewati target, searah gerakan
    overshoot = max(0.0, float((np.sign(step) * (y - step)).max())) / large * 100
    outside = np.nonzero(np.abs(e) > 0.05 * large)[0]
    settling = float(t[outside[-1]] + (t[1] - t[0])) if len(outside) else 0.0
    end = t >= t[-1] - 1.0
    # Chatter hanya bermakna SETELAH mapan: ramp awal yang cepat itu koreksi
    # yang sah, bukan getaran. Merata-ratakan seluruh run mencampur keduanya.
    d_settled = delta[end]
    return dict(overshoot=overshoot, settling=settling,
                residual=float(np.abs(e[end]).mean()),
                jitter=float(np.abs(np.diff(d_settled)).mean()),
                jitter_full=float(np.abs(np.diff(delta)).mean()),
                delta_lim=float(np.abs(delta).max()),
                v_err=float(np.abs(ev).mean()), v_min=float(v.min()),
                v_residual=float(np.abs(ev[end]).mean()),
                solve=float(ms.mean()), failed=int((ok == 0).sum()))


def rows(label, m):
    print(f'{label:>12}{m["overshoot"]:>11.1f}{m["settling"]:>11.2f}'
          f'{m["residual"]:>11.3f}{m["jitter"]*1e3:>13.4f}'
          f'{m["jitter_full"]*1e3:>12.3f}{m["delta_lim"]:>11.4f}'
          f'{m["v_err"]:>10.3f}{m["v_residual"]:>10.3f}{m["v_min"]*3.6:>9.1f}'
          f'{m["solve"]:>9.1f}{m["failed"]:>7}')


def main_():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sweep', nargs=2, metavar=('WEIGHT', 'VALUES'),
                    help=f'weight name ({", ".join(MAP)}) and a list of values')
    ap.add_argument('--step', type=float, default=-1.0, help='lateral step (m)')
    ap.add_argument('--seconds', type=float, default=6.0)
    args = ap.parse_args()

    if args.sweep:
        name = args.sweep[0].upper()
        if name not in MAP:
            raise SystemExit(f'unknown weight: {name}. Choices: {", ".join(MAP)}')
        cfg = [(f'{name}={float(v):g}', weight_with(name, float(v)))
                  for v in args.sweep[1].split(',')]
    else:
        cfg = [('config', None)]

    params = json.load(open(config.VEHICLE_PARAMS_JSON))
    with simulation.carla_world() as world:
        ref, _ = main.prepare_road(world)
        print(f'Lateral step response {args.step:+.2f} m, straight-line reference, '
              f'no obstacles, {config.V_REF * 3.6:.0f} km/h')
        print('ego is respawned for every configuration\n')
        print(f'{"config":>12}{"overshoot%":>11}{"settling s":>11}'
              f'{"residual m":>11}{"chatter mrad":>13}{"jitter tot":>12}'
              f'{"delta max":>11}{"v err":>10}{"v residual":>10}{"v min":>9}'
              f'{"solve":>9}{"failed":>7}')
        print('-' * 124)
        for label, weight in cfg:
            log = step_response(world, params, ref, weight, args.step, args.seconds)
            rows(label, metric(log, args.step))
    print('\nsection 7.6: jagged steering almost always means RD_DELTA is too small')


if __name__ == '__main__':
    main_()
