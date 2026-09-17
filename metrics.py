"""Metrik bab 4 dipisah per fase manuver (bagian 11.6, menutup catatan 15.5).

Bagian 15.5: "deviasi dari tengah lajur saat LANE_KEEPING" hampir seluruhnya
berisi ekor setelah kembali ke lajur; sebelum manuver angkanya 0,001 m. Satu
angka gabungan karena itu mengukur hal lain daripada yang disangka. Modul ini
memisahkannya per fase FSM, dan memisahkan LANE_KEEPING sebelum manuver dari
LANE_KEEPING sesudahnya -- dua hal yang sangat berbeda meski namanya sama.

    python metrics.py                       # per fase, dari log run tunggal
    python metrics.py --eksperimen          # per fase, seluruh ulangan Tahap 9
    python metrics.py --layer --eksperimen  # dikelompokkan per layer arsitektur
"""
import argparse

import numpy as np

import config

URUT = ['LANE_KEEPING (sebelum)', 'CHECK_OVERTAKE', 'LANE_CHANGE_OVERTAKE',
        'OVERTAKING', 'LANE_CHANGE_RETURN', 'LANE_KEEPING (sesudah)']
L_SUMBU = 3.0438489732406473


def fase(states):
    """Nama fase tiap tick, memisahkan LANE_KEEPING sebelum dan sesudah manuver."""
    manuver = np.flatnonzero(states != 'LANE_KEEPING')
    keluar = np.array(states, dtype=object).copy()
    if len(manuver):
        keluar[:manuver[0]] = 'LANE_KEEPING (sebelum)'
        keluar[manuver[-1] + 1:] = 'LANE_KEEPING (sesudah)'
    return keluar


def metrik(log, states, kolom):
    """-> {fase: {metrik: nilai}}. Satu run."""
    k = {n: i for i, n in enumerate(kolom)}
    f = fase(states)
    out = {}
    for nama in URUT:
        m = f == nama
        if not m.any():
            continue
        xte = np.abs(log[m, k['dev_lajur']])
        a_lat = log[m, k['v']] ** 2 * np.tan(log[m, k['delta_cmd']]) / L_SUMBU
        d = np.diff(log[m, k['steer']]) if m.sum() > 1 else np.array([0.0])
        # HATI-HATI. `lacak` BUKAN galat pelacakan pengendali, meskipun tampak
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
        lacak = np.abs(log[m, k['y']] - log[m, k['y_ref']])
        out[nama] = dict(
            n=int(m.sum()),
            lacak_rata=float(lacak.mean()), lacak_maks=float(lacak.max()),
            xte_rata=float(xte.mean()), xte_maks=float(xte.max()),
            yaw_maks=float(np.degrees(np.abs(log[m, k['yaw']])).max()),
            v_err=float(np.abs(log[m, k['v']] - log[m, k['v_goal']]).mean()),
            a_lat_maks=float(np.abs(a_lat).max()),
            jitter=float(np.abs(d).sum()))
    return out


def gabung(per_run):
    """Rata-rata dan sd lintas ulangan, per fase per metrik."""
    hasil = {}
    for nama in URUT:
        ada = [r[nama] for r in per_run if nama in r]
        if not ada:
            continue
        hasil[nama] = {kk: (float(np.mean([a[kk] for a in ada])),
                            float(np.std([a[kk] for a in ada]))) for kk in ada[0]}
    return hasil


def cetak(judul, h):
    print(f'\n{judul}')
    print(f'{"fase":<24}{"tick":>6}{"lacak rata":>14}{"lacak maks":>16}'
          f'{"ke tujuan":>15}{"yaw maks":>13}{"v err":>14}{"a_lat maks":>13}{"jitter":>15}')
    for nama in URUT:
        if nama not in h:
            continue
        v = h[nama]
        sd = lambda kk, fmt: (f'{fmt.format(v[kk][0])}' if v[kk][1] < 5e-6
                              else f'{fmt.format(v[kk][0])}±{fmt.format(v[kk][1])}')
        print(f'{nama:<24}{v["n"][0]:>5.0f} {sd("lacak_rata", "{:.5f}"):>14}'
              f'{sd("lacak_maks", "{:.5f}"):>16}{sd("xte_rata", "{:.3f}"):>15}'
              f'{sd("yaw_maks", "{:.2f}"):>13}{sd("v_err", "{:.3f}"):>14}'
              f'{sd("a_lat_maks", "{:.2f}"):>13}{sd("jitter", "{:.3f}"):>15}')


def per_layer(log, states, kolom):
    """Metrik dikelompokkan menurut layer arsitektur, untuk slide evaluasi.

    Localization sengaja TIDAK ada di sini: ia memakai transform ground truth
    CARLA, jadi tidak punya galat terhadap dirinya sendiri. Yang divalidasi
    untuk layer itu adalah PREDICTION MODEL-nya (bagian 5, `validate_model.py`),
    dan itu eksperimen terpisah yang tidak ikut di run skenario.
    """
    import planning
    k = {n: i for i, n in enumerate(kolom)}
    ada = lambda n: n in k
    y, yaw, v = log[:, k['y']], log[:, k['yaw']], log[:, k['v']]
    manuver = states != 'LANE_KEEPING'
    replan = np.arange(len(log)) % 2 == 0          # planner bekerja 10 Hz

    # Zona aman yang benar-benar tercapai: g >= 1 berarti constraint dihormati.
    # Dihitung dari GROUND TRUTH, antar PUSAT bodi -- yang dinilai harus benar.
    xc = log[:, k['x']] + config.SUMBU_KE_PUSAT * np.cos(yaw)
    yc = y + config.SUMBU_KE_PUSAT * np.sin(yaw)
    g = planning.zona_aman(xc - log[:, k['x_tgt']], yc - log[:, k['y_tgt']])

    a_lat = v ** 2 * np.tan(log[:, k['delta_cmd']]) / L_SUMBU
    jerk_lat = np.diff(a_lat) / np.diff(log[:, k['t']])
    lacak = np.abs(y - log[:, k['y_ref']])
    nl = log[replan, k['n_layak']]

    dt = float(log[1, k['t']] - log[0, k['t']])
    lk = states == 'LANE_KEEPING'
    out = {'Controller (MPC)_IAE': {
        'IAE kecepatan |v - v_goal| [m]': float(np.abs(v - log[:, k['v_goal']]).sum() * dt),
        'IAE lateral saat LANE_KEEPING [m.s]': float(np.abs(log[lk, k['dev_lajur']]).sum() * dt),
        'IAE lateral seluruh run [m.s]': float(np.abs(log[:, k['dev_lajur']]).sum() * dt),
    }, 'Planner': {
        'kandidat lolos per replan (dari 9)': nl.mean(),
        'replan tanpa kandidat [%]': 100.0 * (nl == 0).mean(),
        'durasi manuver direncanakan T [s]': (np.nanmean(log[:, k['t_plan']])
                                              if ada('t_plan') else float('nan')),
        'offset terpilih rata-rata [m]': log[manuver & (log[:, k['offset']] > 0),
                                             k['offset']].mean(),
        'jerk lateral RMS [m/s3]': float(np.sqrt((jerk_lat ** 2).mean())),
        'zona aman g minimum (>=1 aman)': float(g.min()),
    }, 'Controller (MPC)': {
        'galat lacak lateral RMS [m]': float(np.sqrt((lacak ** 2).mean())),
        'galat lacak lateral maks [m]': float(lacak.max()),
        'galat kecepatan RMS [m/s]': float(np.sqrt(((v - log[:, k['v_goal']]) ** 2).mean())),
        'sudut hadap maks [deg]': float(np.degrees(np.abs(yaw)).max()),
        'waktu solve rata-rata [ms]': float(log[1:, k['solve_ms']].mean()),
        'waktu solve maksimum [ms]': float(log[1:, k['solve_ms']].max()),
        'iterasi solver rata-rata': (float(log[1:, k['iterasi']].mean())
                                     if ada('iterasi') else float('nan')),
        'solver berhasil [%]': 100.0 * (log[:, k['solver_ok']] == 1).mean(),
        'slack zona aman maks (0 = patuh)': (float(log[:, k['eps']].max())
                                             if ada('eps') else float('nan')),
        'slack batas lateral maks (0 = patuh)': (float(log[:, k['eps_lat']].max())
                                                 if ada('eps_lat') else float('nan')),
        'percepatan lateral maks [m/s2]': float(np.abs(a_lat).max()),
        'jitter kemudi total': float(np.abs(np.diff(log[:, k['steer']])).sum()),
        'usaha kendali |a| rata-rata [m/s2]': float(np.abs(log[:, k['a_cmd']]).mean()),
    }}
    return out


def main_():
    ap = argparse.ArgumentParser()
    ap.add_argument('--eksperimen', action='store_true',
                    help='pakai seluruh ulangan Tahap 9, bukan run tunggal')
    ap.add_argument('--layer', action='store_true',
                    help='kelompokkan per layer arsitektur, bukan per fase')
    args = ap.parse_args()

    if args.layer:
        for mode in ('gt', 'vision'):
            pola = (f'{config.OUT_DIR}/experiment_s1_{mode}.npz' if args.eksperimen
                    else f'{config.OUT_DIR}/run_s1_mpc_{mode}.npz')
            try:
                d = np.load(pola, allow_pickle=True)
            except FileNotFoundError:
                print(f'\n{pola} tidak ada -- lewati')
                continue
            kolom = list(d['kolom'])
            runs = ([(d['log'][i], d['fsm_state'][i]) for i in range(len(d['log']))]
                    if args.eksperimen and 'log' in d else [(d['log'], d['fsm_state'])])
            per = [per_layer(lg, st, kolom) for lg, st in runs]
            print(f'\nMPC + {mode}, {len(per)} run')
            for layer in per[0]:
                print(f'  {layer}')
                for nama in per[0][layer]:
                    v = np.array([r[layer][nama] for r in per])
                    tail = f' ± {v.std():.4g}' if v.std() > 1e-9 else ''
                    print(f'    {nama:<40}{v.mean():>12.4g}{tail}')
        return
    for mode in ('gt', 'vision'):
        pola = (f'{config.OUT_DIR}/experiment_s1_{mode}.npz' if args.eksperimen
                else f'{config.OUT_DIR}/run_s1_mpc_{mode}.npz')
        try:
            d = np.load(pola, allow_pickle=True)
        except FileNotFoundError:
            print(f'\n{pola} tidak ada -- lewati')
            continue
        kolom = list(d['kolom'])
        if args.eksperimen:
            if 'log' not in d:
                print(f'\n{pola} tidak memuat log mentah -- jalankan ulang experiment.py')
                continue
            per_run = [metrik(d['log'][i], d['fsm_state'][i], kolom)
                       for i in range(len(d['log']))]
            cetak(f'MPC + {mode}, {len(per_run)} ulangan (rata-rata±sd)', gabung(per_run))
        else:
            h = {n: {kk: (vv, 0.0) for kk, vv in v.items()}
                 for n, v in metrik(d['log'], d['fsm_state'], kolom).items()}
            cetak(f'MPC + {mode}, satu run', h)
    print('\nlacak = |y - acuan planner|. BUKAN galat pelacakan: acuannya di-anchor ulang')
    print('  di posisi ego tiap replan, jadi yang terukur hanya lookahead 50 ms.')
    print('ke tujuan = |y ego - tengah lajur tujuan FSM|; saat pindah lajur ia mengukur '
          'profil manuver,\n  bukan galat, dan maksimumnya selalu tepat LANE_WIDTH. '
          'Dilaporkan hanya sebagai konteks.')
    print('yaw = sudut hadap terhadap jalan, derajat.')
    print('v err = |v - v_goal| rata-rata, m/s. a_lat = percepatan lateral '
          f'diperintahkan, batas kenyamanan {config.MAX_LATERAL_ACCEL} m/s2.')
    print('jitter steer = total |perubahan steer| antar tick, tanpa satuan.')


if __name__ == '__main__':
    main_()
