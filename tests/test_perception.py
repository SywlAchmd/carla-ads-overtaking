"""Uji koreksi permukaan -> pusat bodi. Tanpa simulator, tanpa torch.

Yang dikunci: geseran berpindah sumbu mengikuti permukaan yang terlihat. Versi
pertama menambahkan setengah panjang tanpa syarat dan melaporkan target sampai
+3,82 m terlalu jauh ke depan saat ego berdampingan (bagian 19.6). Versi kedua
menebaknya dari rasio kotak dan gagal justru saat berdampingan, karena kotak
terpotong tepi citra (bagian 26.2). Versi ketiga memakai sudut garis pandang.
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import config
import perception


def test_dead_ahead_shifts_longitudinally_only():
    dx, dy = perception.face_correction(0.0)
    assert abs(dx - config.OTHER_LENGTH / 2) < 1e-9, dx
    assert dy == 0.0


def test_alongside_shifts_laterally_only():
    dx, dy = perception.face_correction(math.pi / 2)
    assert abs(dx) < 1e-9, dx
    assert abs(dy - config.OTHER_WIDTH / 2) < 1e-9, dy


def test_oblique_mixes_both_by_silhouette_width():
    """Pada 45 deg sisi menyumbang PANJANG/(PANJANG+LEBAR) lebar siluet, bukan 1/2:
    kendaraan lebih panjang daripada lebar, jadi sisinya mendominasi lebih awal."""
    dx, dy = perception.face_correction(math.pi / 4)
    f = config.OTHER_LENGTH / (config.OTHER_LENGTH + config.OTHER_WIDTH)
    assert abs(dy - f * config.OTHER_WIDTH / 2) < 1e-9, dy
    assert abs(dx - (1 - f) * config.OTHER_LENGTH / 2) < 1e-9, dx


def test_monotonic_and_bounded():
    dy_prev = -1.0
    for deg in range(0, 91, 5):
        dx, dy = perception.face_correction(math.radians(deg))
        assert 0.0 <= dx <= config.OTHER_LENGTH / 2 + 1e-9, (deg, dx)
        assert 0.0 <= dy <= config.OTHER_WIDTH / 2 + 1e-9, (deg, dy)
        assert dy > dy_prev - 1e-12, (deg, dy, dy_prev)
        dy_prev = dy


def test_left_right_symmetric():
    for deg in (12.0, 24.2, 61.0):
        assert (perception.face_correction(math.radians(deg))
                == perception.face_correction(math.radians(-deg)))


def test_regression_ghost_obstacle_side_by_side():
    """REGRESI bagian 26.2. Pada t = 8,15 s run S1 vision, target ada di
    (7,28 , 3,28) m -- sudut pandang 24,2 deg. Versi rasio-kotak memberi f ~ 0 di
    situ karena kotak terpotong tepi citra, jadi koreksi melintang tidak pernah
    diterapkan: galat -1,06 m ke arah ego, zona aman yang DILIHAT MPC jatuh ke
    0,921 padahal yang sebenarnya tidak pernah di bawah 1,169.

    Setengah dari koreksi penuh sudah cukup menutup selisih itu.
    """
    dx, dy = perception.face_correction(math.atan2(3.28, 7.28))
    assert dy > 0.4, dy
    assert dy > 0.45 * config.OTHER_WIDTH / 2, dy
    assert dx > 0.3, dx        # buritan masih terlihat pada 24 deg; jangan nol


def test_zero_angle_length_does_not_blow_up():
    perception.face_correction(0.0)
    perception.face_correction(math.pi)


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            fn()
            print(f'ok  {name}')
    print('all passed')
