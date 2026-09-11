"""Uji MPC. Murni numerik, tidak butuh CARLA."""
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config                                                          # noqa: E402
import control as C                                                    # noqa: E402
from validate_model import bicycle_rk4                                 # noqa: E402

PARAMS = json.load(open(os.path.join(config.OUT_DIR, 'vehicle_params.json')))
V = 13.9
DT = config.FIXED_DELTA_SECONDS

# IPOPT interior-point tidak pernah mendorong eps tepat ke nol; sisa barrier
# terukur 2,2e-06 saat lapang versus 0,999 saat benar-benar melanggar.
SLACK_NOL = 1e-3


def acuan_lurus(x0=0.0, y=0.0, v=V):
    """(4, N+1) garis lurus sepanjang +x pada kecepatan tetap."""
    t = np.arange(config.MPC_N + 1) * config.MPC_DT
    return np.vstack([x0 + v * t, np.full_like(t, y), np.zeros_like(t), np.full_like(t, v)])


def loop_tertutup(mpc, x0, detik, obstacles=None):
    """Plant = model yang sama (perfect model): menguji logika kontroler, bukan fisika."""
    x, jejak = np.array(x0, dtype=float), []
    for _ in range(int(detik / DT)):
        a, delta, ms, ok = mpc.solve(x, acuan_lurus(x0=x[0]), obstacles)
        s = bicycle_rk4(np.array([x[0], x[1], x[2]]), x[3], delta, PARAMS['L'], DT)
        x = np.array([s[0], s[1], s[2], x[3] + a * DT])
        jejak.append((x.copy(), a, delta, ms, ok, mpc.last_iter))
    return jejak


def test_menarik_kembali_ke_acuan():
    """Mobil melenceng 1 m ke kiri -> setir ke kanan, error menyusut."""
    mpc = C.MPCController(PARAMS)
    jejak = loop_tertutup(mpc, [0.0, 1.0, 0.0, V], 2.5)
    assert jejak[0][2] < 0, f'setir harus ke kanan (delta negatif), dapat {jejak[0][2]}'
    y_awal, y_akhir = abs(jejak[0][0][1]), abs(jejak[-1][0][1])
    assert y_akhir < 0.25 * y_awal, (y_awal, y_akhir)
    assert all(ok for *_, ok in jejak), 'solver gagal di tengah loop'


def test_diam_bila_sudah_pas():
    """Sudah di acuan -> setir hampir nol dan tidak berosilasi."""
    mpc = C.MPCController(PARAMS)
    jejak = loop_tertutup(mpc, [0.0, 0.0, 0.0, V], 1.5)
    delta = np.array([d for _, _, d, *_ in jejak])
    assert np.abs(delta).max() < 5e-3, np.abs(delta).max()
    assert np.abs(np.diff(delta)).max() < 2e-3, 'setir berosilasi'


def test_menghormati_batas_laju_perubahan_setir():
    mpc = C.MPCController(PARAMS)
    jejak = loop_tertutup(mpc, [0.0, 2.5, 0.0, V], 2.0)
    delta = np.array([d for _, _, d, *_ in jejak])
    langkah_maks = np.abs(np.diff(delta)).max()
    # perintah keluar tiap 50 ms, batasnya per langkah MPC 100 ms
    assert langkah_maks <= config.DDELTA_MAX * 1.05, langkah_maks
    assert np.abs(delta).max() <= PARAMS['delta_max'] + 1e-6


def test_mengerem_saat_halangan_masuk_horizon():
    """MPC BUKAN penghindar halangan -- dia penjejak dengan batas aman.

    Halangan tepat di depan pada y=0 membuat elipsnya simetris: belok kiri dan
    kanan berbiaya identik, dan Q[Y] menghukum keduanya. Solver diam di titik
    stasioner simetris (delta = 0) dan memperlambat. Yang memutuskan lewat sisi
    mana adalah planner, lewat lintasan acuan yang dihasilkannya.
    """
    mpc = C.MPCController(PARAMS)
    jejak = loop_tertutup(mpc, [0.0, 0.0, 0.0, V], 2.0,
                          obstacles=[[35.0, 0.0, 0.0, 0.0]])
    v = np.array([s[3] for s, *_ in jejak])
    a = np.array([a for _, a, *_ in jejak])
    assert v[-1] < V - 1.0, f'tidak memperlambat: {V:.1f} -> {v[-1]:.2f} m/s'
    assert a.min() < -1.0, a.min()

    tanpa = loop_tertutup(C.MPCController(PARAMS), [0.0, 0.0, 0.0, V], 2.0)
    assert abs(tanpa[-1][0][3] - V) < 0.05, 'tanpa halangan seharusnya jalan terus'


def test_menjejak_lintasan_planner_melewati_halangan():
    """Integrasi Tahap 3 + 5: planner memilih jalur, MPC mengikutinya."""
    import planning
    traj, _ = planning.plan_lane_change(0, 0, 0, 0, V, 0, V,
                                        obstacles=[[45.0, 0.0, 0.0, 0.0]])
    assert traj is not None

    def xref_pada(t0):
        t = t0 + np.arange(config.MPC_N + 1) * config.MPC_DT
        return np.column_stack([traj.sample_at(tt) for tt in t])

    mpc = C.MPCController(PARAMS)
    x, err, g_min = np.array([0.0, 0.0, 0.0, V]), [], []
    for i in range(int(3.0 / DT)):
        t0 = i * DT
        a, delta, _, ok = mpc.solve(x, xref_pada(t0), [[45.0, 0.0, 0.0, 0.0]])
        assert ok, f'solver gagal di tick {i}'
        acuan = traj.sample_at(t0)
        err.append(math.hypot(x[0] - acuan[0], x[1] - acuan[1]))
        g_min.append(((x[0] - 45.0) / config.ELLIPSE_A) ** 2
                     + ((x[1] - 0.0) / config.ELLIPSE_B) ** 2)
        s = bicycle_rk4(x[:3], x[3], delta, PARAMS['L'], DT)
        x = np.array([s[0], s[1], s[2], x[3] + a * DT])

    print(f'      XTE rata-rata {np.mean(err):.3f} m, maks {np.max(err):.3f} m; '
          f'g_min {np.min(g_min):.2f}')
    assert np.max(err) < 0.5, np.max(err)
    assert x[1] < -2.5, f'tidak sampai lajur tujuan, y = {x[1]:.2f}'
    assert np.min(g_min) >= 1.0, np.min(g_min)


def test_tetap_menjawab_saat_tidak_ada_solusi_aman():
    """Kendaraan tepat di depan: solver tidak boleh mengembalikan kosong."""
    mpc = C.MPCController(PARAMS)
    a, delta, ms, ok = mpc.solve(np.array([0.0, 0.0, 0.0, V]), acuan_lurus(),
                                 obstacles=[[3.0, 0.0, 0.0, 0.0]])
    assert np.isfinite(a) and np.isfinite(delta), (a, delta)
    slack = np.array(mpc.opti.debug.value(mpc.eps))
    assert slack.max() > SLACK_NOL, 'slack harusnya aktif saat constraint dilanggar'


def test_kecepatan_sedikit_di_atas_v_max_tetap_terpecahkan():
    """State awal dikunci ke hasil ukur; batas kecepatan tidak boleh berlaku di
    k=0. Kelebihan 0,0003 m/s membuat solver infeasible di run tertutup pertama.
    """
    mpc = C.MPCController(PARAMS)
    for lebih in (0.0003, 0.05, 0.5):
        a, delta, _, ok = mpc.solve(np.array([0.0, 0.0, 0.0, config.V_MAX + lebih]),
                                    acuan_lurus(v=config.V_MAX))
        assert ok, f'gagal pada v = V_MAX + {lebih}'
        assert np.isfinite(a) and np.isfinite(delta)


def test_slot_penuh_menyimpan_halangan_terdekat_dari_ego():
    """Slot terbatas -> harus menyimpan yang terdekat dari EGO.

    Mengurutkan pakai x absolut berarti mengurutkan dari titik asal path. Bedanya
    baru terlihat saat halangan mengapit ego: urutan absolut menyimpan yang di
    belakang dan membuang yang di depan, padahal yang di depan lebih dekat.
    """
    mpc = C.MPCController(PARAMS, n_obs=2)
    x_ego = np.array([500.0, 0.0, 0.0, V])
    obs = [[486.0, 0.0, 9.0, 0.0],      # 14 m di belakang
           [505.0, -3.5, 7.0, 0.0],     #  5 m di depan
           [512.0, -3.5, 7.0, 0.0]]     # 12 m di depan
    disimpan = sorted(mpc._obstacle_matrix(obs, x_ego)[0, :2])
    assert disimpan == [505.0, 512.0], disimpan
    urut_absolut = sorted(np.asarray(obs)[np.argsort([o[0] for o in obs])][:2, 0])
    assert urut_absolut != disimpan, 'kasus uji tidak membedakan kedua urutan'


def test_slack_nol_saat_lapang():
    mpc = C.MPCController(PARAMS)
    mpc.solve(np.array([0.0, 0.0, 0.0, V]), acuan_lurus(),
              obstacles=[[300.0, 0.0, 0.0, 0.0]])
    assert np.array(mpc.opti.debug.value(mpc.eps)).max() < SLACK_NOL


def test_jumlah_iterasi_solver_wajar():
    """Yang di-assert jumlah ITERASI, bukan waktu jam dinding.

    Waktu solve bergantung beban mesin: konfigurasi yang sama terukur 26 ms saat
    mesin senggang dan 70 ms saat CARLA memakai 141% CPU. Uji berbasis waktu
    akan gagal karena lingkungan, bukan karena kode, dan itu melatih orang
    mengabaikan kegagalan. Jumlah iterasi murni algoritmik.

    Waktu solve tetap dicetak sebagai informasi -- klaim real-time untuk skripsi
    diukur terpisah di mesin yang senggang.
    """
    mpc = C.MPCController(PARAMS)
    jejak = loop_tertutup(mpc, [0.0, 1.0, 0.0, V], 2.0)
    ms = np.array([j[3] for j in jejak])
    it = np.array([j[5] for j in jejak])
    print(f'      iterasi: rata-rata {it[1:].mean():.1f}, maks {it[1:].max()}  |  '
          f'solve {ms[1:].mean():.1f} ms (informasi saja)')
    assert it[1:].mean() < 40, it[1:].mean()
    assert it.max() < config.MPC_MAX_ITER, it.max()


def test_warm_start_mempercepat():
    dingin = []
    for _ in range(6):
        m = C.MPCController(PARAMS)          # instance baru = tanpa buffer
        dingin.append(m.solve(np.array([0.0, 1.0, 0.0, V]), acuan_lurus())[2])
    hangat = [t[3] for t in loop_tertutup(C.MPCController(PARAMS), [0.0, 1.0, 0.0, V], 0.5)][1:]
    print(f'      dingin {np.median(dingin):.1f} ms  vs  hangat {np.median(hangat):.1f} ms')
    assert np.median(hangat) < np.median(dingin)


def test_lompatan_sudut_tidak_mengacaukan():
    """psi acuan 0 vs 2*pi itu arah yang sama -> perintah harus sama."""
    xref = acuan_lurus(y=0.5)
    a1, d1, *_ = C.MPCController(PARAMS).solve(np.array([0.0, 0.0, 0.0, V]), xref)
    xref2 = xref.copy(); xref2[2, :] += 2 * math.pi
    a2, d2, *_ = C.MPCController(PARAMS).solve(np.array([0.0, 0.0, 0.0, V]), xref2)
    assert abs(d1 - d2) < 1e-6, (d1, d2)
    assert abs(a1 - a2) < 1e-6, (a1, a2)


def test_steer_membalik_tanda():
    """delta RH positif = belok kiri; steer CARLA positif = belok kanan.

    Salah tanda di sini membuat MPC mengoreksi ke arah yang salah dan error
    membesar sendiri sampai mobil keluar jalan. Terjadi di run tertutup pertama.
    """
    assert C.steer_command(+0.1, V, PARAMS) < 0
    assert C.steer_command(-0.1, V, PARAMS) > 0
    assert abs(C.steer_command(0.0, V, PARAMS)) < 1e-12


def test_steer_memakai_delta_max_fisik():
    """Bagian 7.5 menulis delta/delta_max -- itu salah, understeer ~2,4x."""
    delta = 0.1
    s = abs(C.steer_command(delta, V, PARAMS))
    salah = delta / PARAMS['delta_max']
    assert s < salah, (s, salah)
    kurva = np.asarray(PARAMS['steering_curve'])
    skala = np.interp(V * 3.6, kurva[:, 0], kurva[:, 1])
    assert abs(s - delta / (PARAMS['delta_max_phys'] * skala)) < 1e-9
    assert abs(C.steer_command(10.0, V, PARAMS)) <= 1.0        # ter-clip


def test_throttle_pi():
    pi = C.ThrottlePI()
    t, b = pi.update(-3.0, 0.0, DT)                 # minta perlambatan
    assert t == 0.0 and abs(b - 0.5) < 1e-9, (t, b)
    pi = C.ThrottlePI()
    t, b = pi.update(2.0, 0.0, DT)                  # minta percepatan
    assert t > 0 and b == 0.0
    for _ in range(40):                             # integral menutup error
        t, _ = pi.update(2.0, 0.5, DT)
    assert t > 0.2, t
    assert 0.0 <= t <= 1.0


def test_command_lengkap():
    mpc = C.MPCController(PARAMS)
    cmd = mpc.compute(np.array([0.0, 0.8, 0.0, V]), acuan_lurus(), a_ukur=0.0)
    assert isinstance(cmd, C.ControlCommand) and cmd.solver_ok
    assert -1.0 <= cmd.steer <= 1.0 and 0.0 <= cmd.throttle <= 1.0
    assert 0.0 <= cmd.brake <= 1.0 and cmd.solve_time_ms > 0
    assert not (cmd.throttle > 0 and cmd.brake > 0), 'gas dan rem bersamaan'


if __name__ == '__main__':
    for nama, fn in sorted(globals().items()):
        if nama.startswith('test_'):
            fn()
            print(f'ok  {nama}')
    print('semua lolos')
