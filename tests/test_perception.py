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

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, AKAR)

import config
import perception


def test_tepat_di_depan_menggeser_memanjang_saja():
    dx, dy = perception.koreksi_muka(0.0)
    assert abs(dx - config.LAIN_PANJANG / 2) < 1e-9, dx
    assert dy == 0.0


def test_tepat_di_samping_menggeser_melintang_saja():
    dx, dy = perception.koreksi_muka(math.pi / 2)
    assert abs(dx) < 1e-9, dx
    assert abs(dy - config.LAIN_LEBAR / 2) < 1e-9, dy


def test_serong_mencampur_keduanya_menurut_lebar_siluet():
    """Pada 45 deg sisi menyumbang PANJANG/(PANJANG+LEBAR) lebar siluet, bukan 1/2:
    kendaraan lebih panjang daripada lebar, jadi sisinya mendominasi lebih awal."""
    dx, dy = perception.koreksi_muka(math.pi / 4)
    f = config.LAIN_PANJANG / (config.LAIN_PANJANG + config.LAIN_LEBAR)
    assert abs(dy - f * config.LAIN_LEBAR / 2) < 1e-9, dy
    assert abs(dx - (1 - f) * config.LAIN_PANJANG / 2) < 1e-9, dx


def test_monoton_dan_terbatas():
    dy_lalu = -1.0
    for deg in range(0, 91, 5):
        dx, dy = perception.koreksi_muka(math.radians(deg))
        assert 0.0 <= dx <= config.LAIN_PANJANG / 2 + 1e-9, (deg, dx)
        assert 0.0 <= dy <= config.LAIN_LEBAR / 2 + 1e-9, (deg, dy)
        assert dy > dy_lalu - 1e-12, (deg, dy, dy_lalu)
        dy_lalu = dy


def test_simetris_kiri_kanan():
    for deg in (12.0, 24.2, 61.0):
        assert (perception.koreksi_muka(math.radians(deg))
                == perception.koreksi_muka(math.radians(-deg)))


def test_regresi_halangan_hantu_saat_berdampingan():
    """REGRESI bagian 26.2. Pada t = 8,15 s run S1 vision, target ada di
    (7,28 , 3,28) m -- sudut pandang 24,2 deg. Versi rasio-kotak memberi f ~ 0 di
    situ karena kotak terpotong tepi citra, jadi koreksi melintang tidak pernah
    diterapkan: galat -1,06 m ke arah ego, zona aman yang DILIHAT MPC jatuh ke
    0,921 padahal yang sebenarnya tidak pernah di bawah 1,169.

    Setengah dari koreksi penuh sudah cukup menutup selisih itu.
    """
    dx, dy = perception.koreksi_muka(math.atan2(3.28, 7.28))
    assert dy > 0.4, dy
    assert dy > 0.45 * config.LAIN_LEBAR / 2, dy
    assert dx > 0.3, dx        # buritan masih terlihat pada 24 deg; jangan nol


def test_sudut_nol_panjang_tidak_meledak():
    perception.koreksi_muka(0.0)
    perception.koreksi_muka(math.pi)


if __name__ == '__main__':
    for nama, fn in sorted(globals().items()):
        if nama.startswith('test_'):
            fn()
            print(f'ok  {nama}')
    print('semua lolos')
