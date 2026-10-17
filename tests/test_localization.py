"""Uji frame ego -> frame jalan. Tanpa simulator.

Mengunci titik acuan halangan, yang sudah tiga kali jadi sumber bias 1,433 m
(bias XTE Tahap 1, kotak penilai jarak, dan konversi ini sendiri).
"""
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import config
import localization


def _ego(x=100.0, y=0.0, yaw=0.0, v=13.4):
    return localization.EgoState(x, y, yaw, v, 0.0, 0.0)


def test_obstacle_anchored_at_ego_body_center():
    """Halangan 40 m di depan pusat bodi -> pusat bodi ego + 40 di frame jalan."""
    ego = _ego()
    out = localization.obstacles_ego_to_road([[40.0, 0.0, -6.4, 0.0]], ego)
    assert abs(out[0, 0] - (ego.x + config.AXLE_TO_CENTER + 40.0)) < 1e-9, out


def test_offset_to_ego_body_center_round_trips():
    """Yang dipakai zona planner/MPC: pusat bodi ego dikurangkan lagi, harus
    memberi kembali jarak relatif yang dilihat perception -- tanpa sisa."""
    for yaw in (0.0, 0.2, -0.35):
        ego = _ego(yaw=yaw)
        out = localization.obstacles_ego_to_road([[40.0, -3.5, 0.0, 0.0]], ego)
        xc = ego.x + config.AXLE_TO_CENTER * math.cos(yaw)
        yc = ego.y + config.AXLE_TO_CENTER * math.sin(yaw)
        dx, dy = out[0, 0] - xc, out[0, 1] - yc
        assert abs(math.hypot(dx, dy) - math.hypot(40.0, 3.5)) < 1e-9, (yaw, dx, dy)


def test_fsm_gap_measured_from_rear_axle():
    """main.py mengurangkan ego.x (sumbu belakang) untuk FSM; `_v_follow`
    mendokumentasikan celahnya diukur dari sumbu belakang. Harus cocok."""
    ego = _ego()
    out = localization.obstacles_ego_to_road([[40.0, 0.0, 0.0, 0.0]], ego)
    assert abs((out[0, 0] - ego.x) - (40.0 + config.AXLE_TO_CENTER)) < 1e-9


def test_dead_reckon_closes_circle():
    """Laju belok konstan satu putaran penuh harus kembali ke titik awal.
    Euler (yaw AWAL langkah saja) meleset ~v*dt*pi = 2,1 m di sini; trapesium
    menutup karena suku cos/sin awal dan akhir langkah simetris sepanjang putaran.
    Ini sifat simetri, bukan bukti eksak: panjang tiap langkah tetap aproksimasi."""
    n, v, dt = 420, 13.4, 0.05
    r = 2 * math.pi / (n * dt)              # tepat satu putaran dalam n langkah
    x, y, yaw = 5.0, -2.0, 0.4
    for _ in range(n):
        x, y, yaw = localization.dead_reckon(x, y, yaw, v, v, r, r, dt)
    assert math.hypot(x - 5.0, y + 2.0) < 1e-6, (x, y)
    assert abs(localization.wrap(yaw - 0.4)) < 1e-9


def test_dead_reckon_trapezoid_exact_for_linear_inputs():
    """Trapesium eksak untuk masukan yang berubah linear dalam waktu -- inilah
    integral pers. (5)-(6) Javed dkk. yang dihitung, bukan rumus lain.
    Lurus, v 10 -> 14 m/s dalam 1 s: jarak tepat 12 m. Yaw rate 0 -> 0,2 rad/s
    dalam 1 s: yaw bertambah tepat 0,1 rad."""
    dt, n = 0.05, 20
    x = y = yaw = 0.0
    for k in range(n):
        v0, v1 = 10.0 + 4.0 * k * dt, 10.0 + 4.0 * (k + 1) * dt
        x, y, yaw = localization.dead_reckon(x, y, yaw, v0, v1, 0.0, 0.0, dt)
    assert abs(x - 12.0) < 1e-9 and abs(y) < 1e-12, (x, y)
    yaw = 0.0
    for k in range(n):
        r0, r1 = 0.2 * k * dt, 0.2 * (k + 1) * dt
        _, _, yaw = localization.dead_reckon(0.0, 0.0, yaw, 0.0, 0.0, r0, r1, dt)
    assert abs(yaw - 0.1) < 1e-12, yaw


def test_empty_keeps_shape():
    assert localization.obstacles_ego_to_road([], _ego()).shape == (0, 4)


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            fn()
            print(f'ok  {name}')
    print('all passed')
