"""Uji local planner. Murni numerik, tidak butuh CARLA."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config                                                        # noqa: E402
from planning import (Trajectory, peak_lateral_accel, plan_lane_change,   # noqa: E402
                      quartic_coeffs, quintic_coeffs, _deriv, _val)


def test_safety_zone_contains_forbidden_rectangle():
    """Constraint g >= 1 harus menjamin SAFE_DISTANCE (bagian 11.2): seluruh tepi
    persegi terlarang ada di dalam zona (g <= 1), dan berpapasan di tengah lajur
    sebelah tetap boleh (g >= 1). Elips lama A=7, B=2,2 gagal di sudut."""
    from planning import safety_zone
    hl = (config.EGO_LENGTH + config.OTHER_LENGTH) / 2 + config.SAFE_DISTANCE
    hw = (config.EGO_WIDTH + config.OTHER_WIDTH) / 2 + config.SAFE_DISTANCE
    for s in np.linspace(0.0, 1.0, 11):
        assert safety_zone(hl * s, hw) <= 1.0 + 1e-9, (hl * s, hw)
        assert safety_zone(hl, hw * s) <= 1.0 + 1e-9, (hl, hw * s)
    assert safety_zone(0.0, config.LANE_WIDTH) >= 1.0


def test_ego_dimensions_in_config_match_measured():
    import json
    p = json.load(open(config.VEHICLE_PARAMS_JSON))
    assert abs(config.EGO_LENGTH - p['length']) < 1e-3
    assert abs(config.EGO_WIDTH - p['width']) < 1e-3
    assert abs(config.AXLE_TO_CENTER + p['rear_axle_offset_x']) < 1e-3


def test_quintic_meets_boundary_conditions():
    T = 3.0
    c = quintic_coeffs(0.5, 0.2, -0.1, -3.5, 0.0, 0.0, T)
    d1, d2 = _deriv(c), _deriv(c, 2)
    for got, want, name in [(_val(c, 0), 0.5, 'y(0)'), (_val(d1, 0), 0.2, "y'(0)"),
                            (_val(d2, 0), -0.1, "y''(0)"), (_val(c, T), -3.5, 'y(T)'),
                            (_val(d1, T), 0.0, "y'(T)"), (_val(d2, T), 0.0, "y''(T)")]:
        assert abs(got - want) < 1e-9, (name, got, want)


def test_peak_lateral_acceleration_2_24():
    """Rencana kerja bagian 5.2: dy=3.5, T=3 -> |y''|max ~ 2.24 m/s²."""
    analytic = peak_lateral_accel(3.5, 3.0)
    assert abs(analytic - 2.245) < 0.005, analytic

    c = quintic_coeffs(0, 0, 0, 3.5, 0, 0, 3.0)
    t = np.linspace(0, 3.0, 2001)
    numeric = np.abs(_val(_deriv(c, 2), t)).max()
    assert abs(numeric - analytic) < 1e-3, (numeric, analytic)
    assert analytic < config.MAX_LATERAL_ACCEL


def test_closed_form_zero_initial_conditions():
    """a3 = 10dy/T^3, a4 = -15dy/T^4, a5 = 6dy/T^5."""
    dy, T = 3.5, 3.0
    c = quintic_coeffs(0, 0, 0, dy, 0, 0, T)
    for got, want in zip(c[3:], [10 * dy / T**3, -15 * dy / T**4, 6 * dy / T**5]):
        assert abs(got - want) < 1e-9, (got, want)


def test_quartic_targets_speed_not_position():
    T = 3.0
    c = quartic_coeffs(0.0, 10.0, 0.5, 13.9, 0.0, T)
    assert abs(_val(_deriv(c), T) - 13.9) < 1e-9
    assert abs(_val(_deriv(c, 2), T)) < 1e-9
    assert len(c) == 5                      # derajat 4: posisi akhir bebas


def test_no_obstacle_chooses_lane_center():
    best, feasible = plan_lane_change(0, 0, 0, 0, 13.9, 0, 13.9)
    assert best is not None and feasible
    assert feasible[0][3] is best                   # terurut, termurah pertama
    assert abs(feasible[0][1] - config.LANE_WIDTH) < 1e-9, feasible[0][1]
    y_end = best.states[1, -1]
    assert abs(y_end - config.SIDE_SIGN * config.LANE_WIDTH) < 1e-6, y_end


def test_road_edge_rejects_candidates_leaving_road():
    """Bodi ego (lebar 1,88 m) harus tetap di dalam tepi area jalan."""
    edge = config.SIDE_SIGN * 4.6                  # 0,66 m di luar tengah lajur tujuan
    road = tuple(sorted((edge, -edge)))
    _, feasible = plan_lane_change(0, 0, 0, 0, 13.9, 0, 13.9, road=road)
    offsets = {f[1] for f in feasible}
    assert 4.0 not in offsets and 3.5 in offsets, offsets
    tight = tuple(sorted((config.SIDE_SIGN * 3.0, -config.SIDE_SIGN * 3.0)))
    assert plan_lane_change(0, 0, 0, 0, 13.9, 0, 13.9, road=tight) == (None, [])


def test_overtake_direction_follows_side_sign():
    for sign in (-1, +1):
        best, _ = plan_lane_change(0, 0, 0, 0, 13.9, 0, 13.9, side_sign=sign)
        assert np.sign(best.states[1, -1]) == sign


def test_all_feasible_candidates_respect_comfort_limit():
    _, feasible = plan_lane_change(0, 0, 0, 0, 13.9, 0, 13.9)
    for _, _, T, traj in feasible:
        ddy = np.gradient(np.gradient(traj.states[1], traj.dt), traj.dt)
        assert np.abs(ddy).max() < config.MAX_LATERAL_ACCEL * 1.05


def test_obstacle_in_target_lane_rejects_all():
    """Kendaraan diam 30 m di depan pada lajur tujuan -> tidak ada kandidat lolos."""
    obstacle = [[30.0, config.SIDE_SIGN * config.LANE_WIDTH, 0.0, 0.0]]
    best, feasible = plan_lane_change(0, 0, 0, 0, 13.9, 0, 13.9, obstacles=obstacle)
    assert best is None and feasible == [], len(feasible)


def test_far_obstacle_does_not_reject():
    obstacle = [[500.0, config.SIDE_SIGN * config.LANE_WIDTH, 0.0, 0.0]]
    best, feasible = plan_lane_change(0, 0, 0, 0, 13.9, 0, 13.9, obstacles=obstacle)
    assert best is not None and len(feasible) > 0


def test_sample_at_interpolates():
    states = np.vstack([np.arange(11.0), np.arange(11.0) * -0.35,
                        np.zeros(11), np.full(11, 13.9)])
    tr = Trajectory(states, dt=0.1)
    got = tr.sample_at(0.55)
    assert abs(got[0] - 5.5) < 1e-9 and abs(got[1] + 1.925) < 1e-9
    assert abs(tr.sample_at(0.0)[0]) < 1e-9
    assert abs(tr.sample_at(1.0)[0] - 10.0) < 1e-9


def test_psi_and_v_consistent_with_path():
    best, _ = plan_lane_change(0, 0, 0, 0, 13.9, 0, 13.9)
    x, y, psi, v = best.states
    dx, dy = np.gradient(x, best.dt), np.gradient(y, best.dt)
    assert np.abs(np.arctan2(dy, dx) - psi).max() < 5e-3
    assert np.abs(np.hypot(dx, dy) - v).max() < 5e-2


def test_ttc_trigger_sufficient_for_safety_zone():
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
        obs_in = lambda gap: np.array([[gap, 0.0, v_target, 0.0]])
        needed = next(g for g in np.arange(4.0, 60.0, 0.5)
                     if plan_lane_change(0.0, 0.0, 0.0, 0.0, config.V_REF, 0.0,
                                           config.V_REF, obstacles=obs_in(g))[0] is not None)
        available = config.TTC_TRIGGER * dv
        assert available >= needed, (
            f'dv {dv}: trigger leaves a gap of {available:.1f} m, planner needs {needed:.1f} m')


def test_plan_does_not_blow_up_past_its_duration():
    """Rencana dipertahankan saat replan gagal (bagian 19.14), jadi ia disampel
    melewati durasinya. Polinomial quintic meledak di luar selang -- harus dijepit."""
    traj, _ = plan_lane_change(0.0, 0.0, 0.0, 0.0, config.V_REF, 0.0, config.V_REF)
    assert traj is not None
    end = traj.lateral_at(traj.duration())
    for passed in (0.1, 2.0, 30.0):
        assert traj.lateral_at(traj.duration() + passed) == end
        assert abs(traj.sample_at(traj.duration() + passed)[1] - end[0]) < 1e-6


def test_middle_candidate_exactly_at_lane_center():
    """REGRESI bagian 29. Kisi kandidat harus memuat tengah lajur tujuan, berapa
    pun lebar lajur yang DIUKUR. Sempat dikurangi lebar hasil ukur, sehingga
    seluruh kisi bergeser sebesar galat ukur dan tidak ada kandidat yang jatuh di
    tengah -- planner membidik 0,125 m dari tengah dan MPC mengikutinya dengan
    tepat."""
    import planning as P
    for width_meas in (3.2, 3.38, 3.5, 3.62):
        traj, feasible = P.plan_lane_change(0.0, 0.0, 0.0, 0.0, 13.4, 0.0, 13.4,
                                         y_goal=0.0, lane_width=width_meas)
        assert feasible, width_meas
        end = [abs(t.states[1, -1]) for _, _, _, t in feasible]
        assert min(end) < 0.02, (width_meas, min(end))


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            fn()
            print(f'ok  {name}')
    print('all passed')
