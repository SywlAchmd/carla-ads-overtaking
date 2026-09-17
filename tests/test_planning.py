"""Uji local planner. Murni numerik, tidak butuh CARLA."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config                                                        # noqa: E402
from planning import (Trajectory, peak_lateral_accel, plan_lane_change,   # noqa: E402
                      quartic_coeffs, quintic_coeffs, _deriv, _val)


def test_zona_aman_memuat_persegi_terlarang():
    """Constraint g >= 1 harus menjamin JARAK_AMAN (bagian 11.2): seluruh tepi
    persegi terlarang ada di dalam zona (g <= 1), dan berpapasan di tengah lajur
    sebelah tetap boleh (g >= 1). Elips lama A=7, B=2,2 gagal di sudut."""
    from planning import zona_aman
    hl = (config.EGO_PANJANG + config.LAIN_PANJANG) / 2 + config.JARAK_AMAN
    hw = (config.EGO_LEBAR + config.LAIN_LEBAR) / 2 + config.JARAK_AMAN
    for s in np.linspace(0.0, 1.0, 11):
        assert zona_aman(hl * s, hw) <= 1.0 + 1e-9, (hl * s, hw)
        assert zona_aman(hl, hw * s) <= 1.0 + 1e-9, (hl, hw * s)
    assert zona_aman(0.0, config.LANE_WIDTH) >= 1.0


def test_dimensi_ego_di_config_sama_dengan_hasil_ukur():
    import json
    p = json.load(open(config.VEHICLE_PARAMS_JSON))
    assert abs(config.EGO_PANJANG - p['length']) < 1e-3
    assert abs(config.EGO_LEBAR - p['width']) < 1e-3
    assert abs(config.SUMBU_KE_PUSAT + p['rear_axle_offset_x']) < 1e-3


def test_quintic_memenuhi_syarat_batas():
    T = 3.0
    c = quintic_coeffs(0.5, 0.2, -0.1, -3.5, 0.0, 0.0, T)
    d1, d2 = _deriv(c), _deriv(c, 2)
    for got, want, nama in [(_val(c, 0), 0.5, 'y(0)'), (_val(d1, 0), 0.2, "y'(0)"),
                            (_val(d2, 0), -0.1, "y''(0)"), (_val(c, T), -3.5, 'y(T)'),
                            (_val(d1, T), 0.0, "y'(T)"), (_val(d2, T), 0.0, "y''(T)")]:
        assert abs(got - want) < 1e-9, (nama, got, want)


def test_percepatan_lateral_puncak_2_24():
    """Rencana kerja bagian 5.2: dy=3.5, T=3 -> |y''|max ~ 2.24 m/s²."""
    analitik = peak_lateral_accel(3.5, 3.0)
    assert abs(analitik - 2.245) < 0.005, analitik

    c = quintic_coeffs(0, 0, 0, 3.5, 0, 0, 3.0)
    t = np.linspace(0, 3.0, 2001)
    numerik = np.abs(_val(_deriv(c, 2), t)).max()
    assert abs(numerik - analitik) < 1e-3, (numerik, analitik)
    assert analitik < config.MAX_LATERAL_ACCEL


def test_bentuk_tertutup_syarat_awal_nol():
    """a3 = 10dy/T^3, a4 = -15dy/T^4, a5 = 6dy/T^5."""
    dy, T = 3.5, 3.0
    c = quintic_coeffs(0, 0, 0, dy, 0, 0, T)
    for got, want in zip(c[3:], [10 * dy / T**3, -15 * dy / T**4, 6 * dy / T**5]):
        assert abs(got - want) < 1e-9, (got, want)


def test_quartic_mengejar_kecepatan_bukan_posisi():
    T = 3.0
    c = quartic_coeffs(0.0, 10.0, 0.5, 13.9, 0.0, T)
    assert abs(_val(_deriv(c), T) - 13.9) < 1e-9
    assert abs(_val(_deriv(c, 2), T)) < 1e-9
    assert len(c) == 5                      # derajat 4: posisi akhir bebas


def test_tanpa_halangan_pilih_tengah_lajur():
    best, feasible = plan_lane_change(0, 0, 0, 0, 13.9, 0, 13.9)
    assert best is not None and feasible
    assert feasible[0][3] is best                   # terurut, termurah pertama
    assert abs(feasible[0][1] - config.LANE_WIDTH) < 1e-9, feasible[0][1]
    y_akhir = best.states[1, -1]
    assert abs(y_akhir - config.SIDE_SIGN * config.LANE_WIDTH) < 1e-6, y_akhir


def test_arah_menyalip_ikut_side_sign():
    for sign in (-1, +1):
        best, _ = plan_lane_change(0, 0, 0, 0, 13.9, 0, 13.9, side_sign=sign)
        assert np.sign(best.states[1, -1]) == sign


def test_semua_kandidat_layak_hormati_batas_kenyamanan():
    _, feasible = plan_lane_change(0, 0, 0, 0, 13.9, 0, 13.9)
    for _, _, T, traj in feasible:
        ddy = np.gradient(np.gradient(traj.states[1], traj.dt), traj.dt)
        assert np.abs(ddy).max() < config.MAX_LATERAL_ACCEL * 1.05


def test_halangan_di_lajur_tujuan_ditolak_semua():
    """Kendaraan diam 30 m di depan pada lajur tujuan -> tidak ada kandidat lolos."""
    obstacle = [[30.0, config.SIDE_SIGN * config.LANE_WIDTH, 0.0, 0.0]]
    best, feasible = plan_lane_change(0, 0, 0, 0, 13.9, 0, 13.9, obstacles=obstacle)
    assert best is None and feasible == [], len(feasible)


def test_halangan_jauh_tidak_menolak():
    obstacle = [[500.0, config.SIDE_SIGN * config.LANE_WIDTH, 0.0, 0.0]]
    best, feasible = plan_lane_change(0, 0, 0, 0, 13.9, 0, 13.9, obstacles=obstacle)
    assert best is not None and len(feasible) > 0


def test_sample_at_interpolasi():
    states = np.vstack([np.arange(11.0), np.arange(11.0) * -0.35,
                        np.zeros(11), np.full(11, 13.9)])
    tr = Trajectory(states, dt=0.1)
    got = tr.sample_at(0.55)
    assert abs(got[0] - 5.5) < 1e-9 and abs(got[1] + 1.925) < 1e-9
    assert abs(tr.sample_at(0.0)[0]) < 1e-9
    assert abs(tr.sample_at(1.0)[0] - 10.0) < 1e-9


def test_psi_dan_v_konsisten_dengan_lintasan():
    best, _ = plan_lane_change(0, 0, 0, 0, 13.9, 0, 13.9)
    x, y, psi, v = best.states
    dx, dy = np.gradient(x, best.dt), np.gradient(y, best.dt)
    assert np.abs(np.arctan2(dy, dx) - psi).max() < 5e-3
    assert np.abs(np.hypot(dx, dy) - v).max() < 5e-2


def test_ttc_trigger_cukup_untuk_zona_aman():
    """Pemicu menyalip harus menyisakan celah yang masih bisa direncanakan.

    `TTC_TRIGGER` diturunkan dari waktu dwell FSM (`config.py`), sedangkan zona
    aman menuntut penyeberangan selesai selagi celah masih besar -- dua syarat
    yang diturunkan terpisah. Bagian 15.4 sudah sekali kejadian: kriteria dan
    constraint yang tidak saling diturunkan kebetulan cocok di kasus uji longgar.
    Uji ini yang membuat kebetulan itu tidak lagi diandalkan.

    Diperiksa pada dv terkecil yang masih memicu (`DV_TRIGGER`), yaitu kasus
    terketat: celah saat pemicu = TTC_TRIGGER * dv mengecil bersama dv, sementara
    celah minimum yang dibutuhkan tidak mengecil sebanding.
    """
    for dv in (config.DV_TRIGGER, 6.4):
        v_target = config.V_REF - dv
        obs_di = lambda gap: np.array([[gap, 0.0, v_target, 0.0]])
        perlu = next(g for g in np.arange(4.0, 60.0, 0.5)
                     if plan_lane_change(0.0, 0.0, 0.0, 0.0, config.V_REF, 0.0,
                                           config.V_REF, obstacles=obs_di(g))[0] is not None)
        tersedia = config.TTC_TRIGGER * dv
        assert tersedia >= perlu, (
            f'dv {dv}: pemicu memberi celah {tersedia:.1f} m, planner butuh {perlu:.1f} m')


def test_rencana_tidak_meledak_di_luar_durasinya():
    """Rencana dipertahankan saat replan gagal (bagian 19.14), jadi ia disampel
    melewati durasinya. Polinomial quintic meledak di luar selang -- harus dijepit."""
    traj, _ = plan_lane_change(0.0, 0.0, 0.0, 0.0, config.V_REF, 0.0, config.V_REF)
    assert traj is not None
    akhir = traj.lateral_at(traj.durasi())
    for lewat in (0.1, 2.0, 30.0):
        assert traj.lateral_at(traj.durasi() + lewat) == akhir
        assert abs(traj.sample_at(traj.durasi() + lewat)[1] - akhir[0]) < 1e-6


if __name__ == '__main__':
    for nama, fn in sorted(globals().items()):
        if nama.startswith('test_'):
            fn()
            print(f'ok  {nama}')
    print('semua lolos')
