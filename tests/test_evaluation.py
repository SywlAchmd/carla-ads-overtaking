"""Uji kriteria keberhasilan bagian 11.2 dengan run sintetis. Tanpa CARLA."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config                                                          # noqa: E402
import evaluation as E                                                 # noqa: E402

DIM_EGO, DIM_TGT = (5.01, 1.88), (5.50, 2.10)
DT, V_EGO, V_TGT = 0.05, 13.4, 7.0


def run_synthetic(seconds=20.0, overtake=True, delay_return=0.0, gap_initial=60.0):
    """Ego menyalip target: lurus -> geser ke -3,5 -> lewat -> kembali."""
    t = np.arange(0, seconds, DT)
    x = V_EGO * t
    x_tgt = gap_initial + V_TGT * t
    y_tgt = np.zeros_like(t)
    y = np.zeros_like(t)
    states = np.array(['LANE_KEEPING'] * len(t), dtype=object)
    if overtake:
        start, T = 3.0, 4.0
        for i, tt in enumerate(t):
            s = np.clip((tt - start) / T, 0, 1)
            out = -3.5 * s ** 3 * (10 - 15 * s + 6 * s ** 2)
            return_start = start + T + 3.0 + delay_return
            u = np.clip((tt - return_start) / T, 0, 1)
            y[i] = out + 3.5 * u ** 3 * (10 - 15 * u + 6 * u ** 2)
            if start <= tt < return_start + T:
                states[i] = 'OVERTAKING'
    return t, x, y, states, x_tgt, y_tgt


def test_clean_run_declared_success():
    t, x, y, st, xt, yt = run_synthetic()
    ok, cat, r = E.evaluate_run(t, x, y, st, xt, yt, False, DIM_EGO, DIM_TGT)
    assert ok and cat == 'success', (cat, r)
    assert r['min_dist'] > config.SAFE_DISTANCE
    assert r['duration'] < config.MANEUVER_LIMIT


def test_collision_fails_run():
    t, x, y, st, xt, yt = run_synthetic()
    ok, cat, _ = E.evaluate_run(t, x, y, st, xt, yt, True, DIM_EGO, DIM_TGT)
    assert not ok and cat == 'collision'


def test_never_overtaking_counted_as_abort():
    """Bagian 11.2: abort BUKAN kegagalan sistem, melainkan keputusan FSM."""
    t, x, y, st, xt, yt = run_synthetic(overtake=False)
    ok, cat, _ = E.evaluate_run(t, x, y, st, xt, yt, False, DIM_EGO, DIM_TGT)
    assert not ok and cat == 'abort'


def test_not_returning_to_lane_counted_as_lane_departure():
    t, x, y, st, xt, yt = run_synthetic(seconds=12.0, delay_return=90.0)
    ok, cat, _ = E.evaluate_run(t, x, y, st, xt, yt, False, DIM_EGO, DIM_TGT)
    assert not ok and cat == 'lane_departure'


def test_maneuver_too_long_counted_as_timeout():
    t, x, y, st, xt, yt = run_synthetic(seconds=40.0, delay_return=22.0)
    ok, cat, r = E.evaluate_run(t, x, y, st, xt, yt, False, DIM_EGO, DIM_TGT)
    assert not ok and cat == 'timeout', (cat, r)
    assert r['duration'] > config.MANEUVER_LIMIT


def test_distance_measured_body_to_body_not_center_to_center():
    """Pusat berjarak 1,5 m lateral berarti kedua bodi SUDAH bertumpuk."""
    d = E.box_distance(np.array([0.0]), np.array([1.5]), DIM_EGO, DIM_TGT)
    assert d[0] == 0.0, d
    # berdampingan satu lajur penuh: bersih
    d = E.box_distance(np.array([0.0]), np.array([3.5]), DIM_EGO, DIM_TGT)
    assert abs(d[0] - (3.5 - (1.88 + 2.10) / 2)) < 1e-9, d
    # berurutan sejajar
    d = E.box_distance(np.array([10.0]), np.array([0.0]), DIM_EGO, DIM_TGT)
    assert abs(d[0] - (10.0 - (5.01 + 5.50) / 2)) < 1e-9, d


def test_side_by_side_too_close_fails():
    """Menyalip dengan simpangan lateral cuma 2,5 m -> bodi berjarak 0,51 m."""
    t, x, y, st, xt, yt = run_synthetic()
    y = y * (2.5 / 3.5)
    ok, cat, r = E.evaluate_run(t, x, y, st, xt, yt, False, DIM_EGO, DIM_TGT)
    assert not ok and cat == 'collision', (cat, r)
    assert r['min_dist'] <= config.SAFE_DISTANCE


def test_ego_yaw_reduces_body_distance():
    """Kotak ego ikut berputar. Mengabaikan yaw melebihkan jarak: pada 5 derajat
    dan simpangan lateral 3,5 m, selisihnya ~0,2 m."""
    straight = E.box_distance(np.array([0.0]), np.array([3.5]), DIM_EGO, DIM_TGT)[0]
    skewed = E.box_distance(np.array([0.0]), np.array([3.5]), DIM_EGO, DIM_TGT,
                           np.radians(5.0))[0]
    assert straight - skewed > 0.15, (straight, skewed)
    # sejajar sumbu harus tetap sama dengan rumus analitik
    assert abs(straight - (3.5 - (DIM_EGO[1] + DIM_TGT[1]) / 2)) < 1e-9
    assert E.box_distance(np.array([10.0]), np.array([0.0]), DIM_EGO, DIM_TGT,
                         np.radians(5.0))[0] > 0.0


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            fn()
            print(f'ok  {name}')
    print('all passed')
