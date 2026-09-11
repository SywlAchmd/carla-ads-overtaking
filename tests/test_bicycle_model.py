"""Uji tanda dan geometri prediction model. Tidak butuh CARLA berjalan."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from validate_model import bicycle_rk4        # noqa: E402

L, DT, V = 2.9, 0.1, 10.0


def roll(delta, n):
    s = np.zeros(3)
    for _ in range(n):
        s = bicycle_rk4(s, V, delta, L, DT)
    return s


def test_straight():
    s = roll(0.0, 50)
    assert abs(s[1]) < 1e-9 and abs(s[2]) < 1e-9, s
    assert abs(s[0] - V * 5.0) < 1e-6, s


def test_turn_sign():
    # right-handed: delta positif -> belok kiri -> psi naik, y naik
    left = roll(0.1, 20)
    assert left[2] > 0 and left[1] > 0, left
    right = roll(-0.1, 20)
    assert right[2] < 0 and right[1] < 0, right


def test_turn_radius():
    delta = 0.15
    r_expected = L / math.tan(delta)
    # seperempat lingkaran: psi = pi/2 setelah jarak r*pi/2
    n = int(round((r_expected * math.pi / 2) / (V * DT)))
    s = roll(delta, n)
    r_actual = math.hypot(s[0], s[1] - r_expected)   # pusat lingkaran di (0, r)
    # 1%, bukan 5%: Euler meleset 3,47% pada kasus ini sehingga masih lolos di
    # toleransi lama. RK4 tepat sampai tiga desimal.
    assert abs(r_actual - r_expected) < 0.01 * r_expected, (r_actual, r_expected)
    assert abs(s[2] - math.pi / 2) < 0.05, s


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            fn()
            print(f'ok  {name}')
    print('semua lolos')
