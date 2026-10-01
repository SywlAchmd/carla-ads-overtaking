"""Metrik bab 4 dipisah per fase manuver (bagian 11.6, menutup catatan 15.5).

Bagian 15.5: "deviasi dari tengah lajur saat LANE_KEEPING" hampir seluruhnya
berisi ekor setelah kembali ke lajur; sebelum manuver angkanya 0,001 m. Satu
angka gabungan karena itu mengukur hal lain daripada yang disangka. Modul ini
memisahkannya per fase FSM, dan memisahkan LANE_KEEPING sebelum manuver dari
LANE_KEEPING sesudahnya -- dua hal yang sangat berbeda meski namanya sama.

    python metrics.py                       # per fase, dari log run tunggal
    python metrics.py --experiment          # per fase, seluruh ulangan Tahap 9
    python metrics.py --layer --experiment  # dikelompokkan per layer arsitektur
    python metrics.py --experiment --suffix _before   # hasil sebelum perbaikan bagian 27
"""
import argparse

import numpy as np

import config

ORDER = ['LANE_KEEPING (before)', 'CHECK_OVERTAKE', 'LANE_CHANGE_OVERTAKE',
        'OVERTAKING', 'LANE_CHANGE_RETURN', 'LANE_KEEPING (after)']
WHEELBASE = 3.0438489732406473


def phase(states):
    """Nama fase tiap tick, memisahkan LANE_KEEPING sebelum dan sesudah manuver."""
    maneuver = np.flatnonzero(states != 'LANE_KEEPING')
    labelled = np.array(states, dtype=object).copy()
    if len(maneuver):
        labelled[:maneuver[0]] = 'LANE_KEEPING (before)'
        labelled[maneuver[-1] + 1:] = 'LANE_KEEPING (after)'
    return labelled


def metric(log, states, columns):
    """-> {fase: {metrik: nilai}}. Satu run."""
    k = {n: i for i, n in enumerate(columns)}
    f = phase(states)
    out = {}
    for name in ORDER:
        m = f == name
        if not m.any():
            continue
        xte = np.abs(log[m, k['lane_dev']])
        a_lat = log[m, k['v']] ** 2 * np.tan(log[m, k['delta_cmd']]) / WHEELBASE
        d = np.diff(log[m, k['steer']]) if m.sum() > 1 else np.array([0.0])
        # HATI-HATI. `track` BUKAN galat pelacakan pengendali, meskipun tampak
        # begitu. `y_ref` adalah rencana planner yang DI-ANCHOR ULANG di posisi
        # ego tiap replan (10 Hz), jadi pada tick replan nilainya 1,8e-08 m --
        # nol karena konstruksi, bukan karena pengendalinya bagus. Yang tersisa
        # hanyalah galat lookahead 50 ms antar replan.
        #
        # Ini persis jebakan nomor 1 di TUNING_MPC.md bagian 9, dan sempat
        # terlaporkan sebagai "galat pengendali 0,0017 m" pada draf bagian 23.
        # Dipertahankan sebagai diagnostik kehalusan, dengan nama yang jujur.
        #
        # Galat pelacakan yang sah menuntut acuan yang TIDAK menempel ke ego --
        # perlu mencatat rencana pada lookahead tetap, lalu membandingkannya
        # dengan posisi sebenarnya setelah selang itu. Belum ada di log.
        track = np.abs(log[m, k['y']] - log[m, k['y_ref']])
        # XTE ke tengah lajur TERDEKAT -- acuan geometris yang tidak menempel
        # ke ego, jadi sah di seluruh fase (bagian 22.5).
        center = np.array([0.0, config.SIDE_SIGN * config.LANE_WIDTH,
                          config.SIDE_SIGN * 2 * config.LANE_WIDTH])
        # y frame PETA bila ada: XTE harus diukur terhadap lajur yang SEBENARNYA,
        # bukan terhadap lajur yang diyakini kamera (bagian 28.3).
        y_value = log[m, k['y_map']] if 'y_map' in k else log[m, k['y']]
        xte_lane = np.abs(y_value[:, None] - center[None, :]).min(axis=1)
        out[name] = dict(
            n=int(m.sum()),
            track_mean=float(track.mean()), track_max=float(track.max()),
            xte_lane_mean=float(xte_lane.mean()), xte_lane_max=float(xte_lane.max()),
            xte_mean=float(xte.mean()), xte_max=float(xte.max()),
            yaw_max=float(np.degrees(np.abs(log[m, k['yaw']])).max()),
            v_err=float(np.abs(log[m, k['v']] - log[m, k['v_goal']]).mean()),
            a_lat_max=float(np.abs(a_lat).max()),
            jitter=float(np.abs(d).sum()))
    return out


def combine(per_run):
    """Rata-rata dan sd lintas ulangan, per fase per metrik."""
    result = {}
    for name in ORDER:
        present = [r[name] for r in per_run if name in r]
        if not present:
            continue
        result[name] = {kk: (float(np.mean([a[kk] for a in present])),
                            float(np.std([a[kk] for a in present]))) for kk in present[0]}
    return result


def show(title, h):
    print(f'\n{title}')
    print(f'{"phase":<24}{"tick":>6}{"track mean":>14}{"track max":>16}'
          f'{"XTE lane":>13}{"XTE max":>13}{"yaw max":>13}{"v err":>14}'
          f'{"a_lat max":>13}{"jitter":>15}')
    for name in ORDER:
        if name not in h:
            continue
        v = h[name]
        sd = lambda kk, fmt: (f'{fmt.format(v[kk][0])}' if v[kk][1] < 5e-6
                              else f'{fmt.format(v[kk][0])}±{fmt.format(v[kk][1])}')
        print(f'{name:<24}{v["n"][0]:>5.0f} {sd("track_mean", "{:.5f}"):>14}'
              f'{sd("track_max", "{:.5f}"):>16}{sd("xte_lane_mean", "{:.3f}"):>13}'
              f'{sd("xte_lane_max", "{:.3f}"):>13}'
              f'{sd("yaw_max", "{:.2f}"):>13}{sd("v_err", "{:.3f}"):>14}'
              f'{sd("a_lat_max", "{:.2f}"):>13}{sd("jitter", "{:.3f}"):>15}')


def per_layer(log, states, columns):
    """Metrik dikelompokkan menurut layer arsitektur, untuk slide evaluasi.

    Localization sengaja TIDAK ada di sini: ia memakai transform ground truth
    CARLA, jadi tidak punya galat terhadap dirinya sendiri. Yang divalidasi
    untuk layer itu adalah PREDICTION MODEL-nya (bagian 5, `validate_model.py`),
    dan itu eksperimen terpisah yang tidak ikut di run skenario.
    """
    import planning
    import config as _c
    k = {n: i for i, n in enumerate(columns)}
    present = lambda n: n in k
    y, yaw, v = log[:, k['y']], log[:, k['yaw']], log[:, k['v']]
    maneuver = states != 'LANE_KEEPING'
    replan = np.arange(len(log)) % 2 == 0          # planner bekerja 10 Hz

    # Zona aman yang benar-benar tercapai: g >= 1 berarti constraint dihormati.
    # Dihitung dari GROUND TRUTH, antar PUSAT bodi -- yang dinilai harus benar.
    xc = log[:, k['x']] + config.AXLE_TO_CENTER * np.cos(yaw)
    yc = y + config.AXLE_TO_CENTER * np.sin(yaw)
    g = planning.safety_zone(xc - log[:, k['x_tgt']], yc - log[:, k['y_tgt']])

    a_lat = v ** 2 * np.tan(log[:, k['delta_cmd']]) / WHEELBASE
    jerk_lat = np.diff(a_lat) / np.diff(log[:, k['t']])
    track = np.abs(y - log[:, k['y_ref']])
    nl = log[replan, k['n_feasible']]

    dt = float(log[1, k['t']] - log[0, k['t']])
    lk = states == 'LANE_KEEPING'
    t = log[:, k['t']]

    # XTE terhadap TENGAH LAJUR TERDEKAT. Geometri lajur tidak ikut bergerak
    # bersama ego, jadi tidak ada masalah anchoring seperti pada y_ref
    # (bagian 22.5). Terbaca sepanjang run: galat sungguhan saat menjaga lajur,
    # dan memuncak di setengah lebar lajur saat menyeberang -- memang begitu.
    center = np.array([0.0, config.SIDE_SIGN * config.LANE_WIDTH,
                      config.SIDE_SIGN * 2 * config.LANE_WIDTH])
    y_value = log[:, k['y_map']] if 'y_map' in k else log[:, k['y']]
    xte_lane = np.abs(y_value[:, None] - center[None, :]).min(axis=1)

    # ITAE memakai waktu sejak MANUVER dimulai, bukan sejak run mulai: galat
    # yang lambat hilang setelah manuver itu yang ingin dihukum.
    man = np.flatnonzero(states != 'LANE_KEEPING')
    t0 = t[man[0]] if len(man) else t[0]
    tw = np.clip(t - t0, 0.0, None)

    # Galat pelacakan yang SAH: rencana pada lookahead tetap versus posisi yang
    # benar-benar terjadi setelah selang itu.
    pred = {}
    for name, col, seconds in (('0.5 s', 'y_plan_05', 0.5), ('2.0 s', 'y_plan_20', 2.0)):
        if col not in k:
            continue
        shift = int(round(seconds / dt))
        e = np.abs(log[:-shift, k[col]] - log[shift:, k['y']])
        pred['prediction error @ %s, RMS [m]' % name] = float(np.sqrt((e ** 2).mean()))
        pred['prediction error @ %s, max [m]' % name] = float(e.max())

    out = {'Controller (MPC)_prediction': pred, 'Controller (MPC)_IAE': {
        'IAE speed |v - v_goal| [m]': float(np.abs(v - log[:, k['v_goal']]).sum() * dt),
        'IAE lateral during LANE_KEEPING [m.s]': float(np.abs(log[lk, k['lane_dev']]).sum() * dt),
        'IAE lateral to nearest lane [m.s]': float(xte_lane.sum() * dt),
        'ISE lateral to nearest lane [m2.s]': float((xte_lane ** 2).sum() * dt),
        'ITAE lateral to nearest lane [m.s2]': float((tw * xte_lane).sum() * dt),
        'XTE to nearest lane, RMS [m]': float(np.sqrt((xte_lane ** 2).mean())),
        'XTE to nearest lane, max [m]': float(xte_lane.max()),
    }, 'Planner': {
        'candidates passed per replan (of 9)': nl.mean(),
        'replans without candidates [%]': 100.0 * (nl == 0).mean(),
        'planned maneuver duration T [s]': (np.nanmean(log[:, k['t_plan']])
                                              if present('t_plan') else float('nan')),
        'mean chosen offset [m]': log[maneuver & (log[:, k['offset']] > 0),
                                             k['offset']].mean(),
        'jerk lateral RMS [m/s3]': float(np.sqrt((jerk_lat ** 2).mean())),
        'safety zone g minimum (>=1 safe)': float(g.min()),
    }, 'Controller (MPC)': {
        'lateral tracking error RMS [m]': float(np.sqrt((track ** 2).mean())),
        'lateral tracking error max [m]': float(track.max()),
        'speed error RMS [m/s]': float(np.sqrt(((v - log[:, k['v_goal']]) ** 2).mean())),
        'max heading angle [deg]': float(np.degrees(np.abs(yaw)).max()),
        'mean solve time [ms]': float(log[1:, k['solve_ms']].mean()),
        'max solve time [ms]': float(log[1:, k['solve_ms']].max()),
        'mean solver iterations': (float(log[1:, k['iterations']].mean())
                                     if present('iterations') else float('nan')),
        'solver success [%]': 100.0 * (log[:, k['solver_ok']] == 1).mean(),
        'safety zone slack max (0 = compliant)': (float(log[:, k['eps']].max())
                                             if present('eps') else float('nan')),
        'lateral limit slack max (0 = compliant)': (float(log[:, k['eps_lat']].max())
                                                 if present('eps_lat') else float('nan')),
        'max lateral acceleration [m/s2]': float(np.abs(a_lat).max()),
        'total steering jitter': float(np.abs(np.diff(log[:, k['steer']])).sum()),
        'mean control effort |a| [m/s2]': float(np.abs(log[:, k['a_cmd']]).mean()),
    }}
    return out


def main_():
    ap = argparse.ArgumentParser()
    ap.add_argument('--experiment', action='store_true',
                    help='use all Stage 9 repeats, not a single run')
    ap.add_argument('--layer', action='store_true',
                    help='group by architecture layer, not by phase')
    ap.add_argument('--suffix', default='',
                    help='file name suffix, e.g. _before -- to compare '
                         'results before and after the fix (section 27)')
    args = ap.parse_args()

    if args.layer:
        for mode in ('gt', 'vision'):
            pattern = (f'{config.OUT_DIR}/experiment_s1_{mode}{args.suffix}.npz'
                    if args.experiment
                    else f'{config.OUT_DIR}/run_s1_mpc_{mode}{args.suffix}.npz')
            try:
                d = np.load(pattern, allow_pickle=True)
            except FileNotFoundError:
                print(f'\n{pattern} missing -- skipped')
                continue
            columns = list(d['columns'])
            runs = ([(d['log'][i], d['fsm_state'][i]) for i in range(len(d['log']))]
                    if args.experiment and 'log' in d else [(d['log'], d['fsm_state'])])
            per = [per_layer(lg, st, columns) for lg, st in runs]
            print(f'\nMPC + {mode}, {len(per)} run')
            for layer in per[0]:
                print(f'  {layer}')
                for name in per[0][layer]:
                    v = np.array([r[layer][name] for r in per])
                    tail = f' ± {v.std():.4g}' if v.std() > 1e-9 else ''
                    print(f'    {name:<40}{v.mean():>12.4g}{tail}')
        return
    for mode in ('gt', 'vision'):
        pattern = (f'{config.OUT_DIR}/experiment_s1_{mode}{args.suffix}.npz'
                if args.experiment
                else f'{config.OUT_DIR}/run_s1_mpc_{mode}{args.suffix}.npz')
        try:
            d = np.load(pattern, allow_pickle=True)
        except FileNotFoundError:
            print(f'\n{pattern} missing -- skipped')
            continue
        columns = list(d['columns'])
        if args.experiment:
            if 'log' not in d:
                print(f'\n{pattern} holds no raw logs -- rerun experiment.py')
                continue
            per_run = [metric(d['log'][i], d['fsm_state'][i], columns)
                       for i in range(len(d['log']))]
            show(f'MPC + {mode}, {len(per_run)} repeats (mean±sd)', combine(per_run))
        else:
            h = {n: {kk: (vv, 0.0) for kk, vv in v.items()}
                 for n, v in metric(d['log'], d['fsm_state'], columns).items()}
            show(f'MPC + {mode}, single run', h)
    print('\ntrack = |y - planner reference|. NOT a tracking error: the reference is re-anchored')
    print('  at the ego position every replan, so only a 50 ms lookahead is measured.')
    print('XTE lane = |y - NEAREST lane center|, a geometric reference not attached to the ego.')
    print('  Its maximum is ~half a lane width while crossing -- as it should be.')
    print('yaw = heading relative to the road, degrees.')
    print('v err = mean |v - v_goal|, m/s. a_lat = commanded lateral acceleration, '
          f'comfort limit {config.MAX_LATERAL_ACCEL} m/s2.')
    print('steer jitter = total |steer change| between ticks, unitless.')


if __name__ == '__main__':
    main_()
