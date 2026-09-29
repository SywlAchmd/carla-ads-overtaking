"""Uji penaksir dimensi kendaraan dari kotak deteksi. Tanpa CARLA, tanpa torch.

Kotak dibangkitkan dari dimensi yang diketahui lewat model kamera yang sama,
lalu dicek apakah penaksir mengembalikannya. Yang dikunci: panjang BARU teramati
setelah sudut pandang menyapu, dan kotak yang terpotong tepi citra tidak boleh
ikut menarik taksiran.
"""
import math
import os
import sys

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, AKAR)

import config                                                         # noqa: E402
import perception                                                     # noqa: E402

PANJANG, LEBAR, TINGGI = 4.605, 1.932, 1.855


def _kotak(theta, d, panjang=PANJANG, lebar=LEBAR, tinggi=TINGGI, u_pusat=None):
    """Kotak deteksi untuk target berdimensi diketahui pada (d, theta)."""
    siluet = panjang * abs(math.sin(theta)) + lebar * abs(math.cos(theta))
    w_px = siluet * perception.F_PIKSEL / d
    h_px = tinggi * perception.F_PIKSEL / d
    u = config.KAMERA_LEBAR / 2.0 if u_pusat is None else u_pusat
    v = config.KAMERA_TINGGI / 2.0
    return [u - w_px / 2, v - h_px / 2, u + w_px / 2, v + h_px / 2, 0.9]


def _sapu(dim, sudut_deg, d=18.0, ulang=6):
    for _ in range(ulang):
        for deg in sudut_deg:
            dim.amati(_kotak(math.radians(deg), d), d, math.radians(deg))


def test_prior_dipakai_sebelum_ada_amatan():
    dim = perception.DimensiKendaraan()
    pj, lb = dim.ukuran()
    assert abs(pj - config.PRIOR_PANJANG) < 1e-6, pj
    assert abs(lb - config.PRIOR_LEBAR) < 1e-6, lb
    assert dim.tinggi is None


def test_prior_bukan_dimensi_simulator():
    """Kalau priornya LAIN_PANJANG/LAIN_LEBAR, jalur vision diam-diam meminjam
    ground truth dan seluruh maksud bagian 28 batal."""
    assert abs(config.PRIOR_PANJANG - config.LAIN_PANJANG) > 0.3
    assert config.PRIOR_PANJANG > config.LAIN_PANJANG      # prior konservatif


def test_lebar_teramati_walau_target_selalu_di_depan():
    """theta ~ 0: siluet = lebar saja. Lebar harus benar, panjang TIDAK."""
    dim = perception.DimensiKendaraan()
    _sapu(dim, [0.0, 1.0, -1.0], d=25.0, ulang=10)
    pj, lb = dim.ukuran()
    assert abs(lb - LEBAR) < 0.10, lb
    assert dim.teramati < 0.02, dim.teramati       # panjang belum teramati


def test_panjang_teramati_setelah_sudut_menyapu():
    """Sudut menyapu 0-42 deg, seperti selama menyalip sungguhan sebelum target
    keluar dari fov 90 deg. Panjang bergerak dari prior 5,38 ke arah 4,61."""
    dim = perception.DimensiKendaraan()
    for _ in range(8):
        for deg in (0.0, 8.0, 16.0, 25.0, 35.0, 42.0):
            for d in (30.0, 20.0, 12.0):
                dim.amati(_kotak(math.radians(deg), d), d, math.radians(deg))
    pj, lb = dim.ukuran()
    assert abs(pj - PANJANG) < 0.40, pj
    assert abs(lb - LEBAR) < 0.15, lb
    assert dim.teramati > 0.05, dim.teramati


def test_sisa_bias_panjang_mengarah_ke_sisi_aman():
    """Prior lebih panjang daripada target, dan panjang hanya teramati lemah pada
    theta <= 42 deg. Sisa biasnya karena itu TERLALU PANJANG -- zona aman jadi
    sedikit lebih besar daripada perlu, bukan lebih kecil. Kalau suatu saat tanda
    ini berbalik, zona menyusut diam-diam dan uji ini harus menahannya."""
    dim = perception.DimensiKendaraan()
    for _ in range(8):
        for deg in (0.0, 8.0, 16.0, 25.0, 35.0, 42.0):
            for d in (30.0, 20.0, 12.0):
                dim.amati(_kotak(math.radians(deg), d), d, math.radians(deg))
    assert dim.ukuran()[0] >= PANJANG, dim.ukuran()


def test_tinggi_terukur_langsung_tanpa_bergantung_sudut():
    """Tinggi tidak pernah bergantung sudut pandang -- itu yang membuatnya
    berguna sebagai bukti bahwa balik-proyeksinya benar."""
    dim = perception.DimensiKendaraan()
    for deg in (0.0, 20.0, 40.0):
        for d in (10.0, 20.0, 35.0):
            dim.amati(_kotak(math.radians(deg), d), d, math.radians(deg))
    assert abs(dim.tinggi - TINGGI) < 0.05, dim.tinggi


def test_kotak_terpotong_tepi_citra_ditolak():
    """REGRESI bagian 28.2. Kotak yang menyentuh tepi punya lebar terpotong, dan
    itu terjadi persis saat ego berdampingan -- sudut pandang yang paling
    informatif. Memakainya menarik taksiran panjang ke bawah."""
    dim = perception.DimensiKendaraan()
    th = math.radians(35.0)
    penuh = _kotak(th, 12.0)
    lebar_px = penuh[2] - penuh[0]
    mepet = _kotak(th, 12.0, u_pusat=lebar_px / 2.0 - 5.0)   # x1 keluar kiri
    assert mepet[0] <= 1.0, mepet
    assert dim.amati(mepet, 12.0, th) is False
    assert dim.n_amatan == 0
    assert dim.amati(penuh, 12.0, th) is True
    assert dim.n_amatan == 1


def test_amatan_tak_masuk_akal_ditolak():
    dim = perception.DimensiKendaraan()
    th = math.radians(20.0)
    assert dim.amati(_kotak(th, 18.0), 0.2, th) is False          # terlalu dekat
    assert dim.amati(_kotak(th, 18.0), 1e4, th) is False          # di luar jangkauan
    assert dim.n_amatan == 0


def test_taksiran_dijepit_ke_rentang_yang_mungkin():
    dim = perception.DimensiKendaraan()
    th = math.radians(30.0)
    for _ in range(50):
        dim.amati(_kotak(th, 15.0, panjang=40.0, lebar=9.0), 15.0, th)
    pj, lb = dim.ukuran()
    assert 3.0 <= pj <= 12.0, pj
    assert 1.4 <= lb <= 2.1, lb      # batas PP 55/2012


def test_zona_dari_dimensi_mempertahankan_nilai_lama():
    """Jalur ground truth tidak boleh bergeser sedikit pun oleh refactor ini."""
    a, b = config.zona_dari_dimensi(config.LAIN_PANJANG, config.LAIN_LEBAR)
    assert abs(a - config.ELLIPSE_A) < 1e-12, a
    assert abs(b - config.ELLIPSE_B) < 1e-12, b


def test_zona_tidak_pernah_kompleks():
    """REGRESI: B harus melebihi setengah lebar, kalau tidak akar pangkat empat
    dari bilangan negatif memberi hasil kompleks dan run mati di tengah."""
    for lebar_lajur in (2.6, 3.0, 3.4, 3.5, 4.2):
        for lain_lebar in (1.4, 1.9, 2.1):
            a, b = config.zona_dari_dimensi(config.PRIOR_PANJANG, lain_lebar, lebar_lajur)
            assert isinstance(a, float) and isinstance(b, float)
            assert a > 0 and b > 0, (lebar_lajur, lain_lebar, a, b)
            assert b > (config.EGO_LEBAR + lain_lebar) / 2 + config.JARAK_AMAN


def test_zona_membesar_untuk_kendaraan_lebih_besar():
    kecil = config.zona_dari_dimensi(4.0, 1.7)
    besar = config.zona_dari_dimensi(6.0, 2.3)
    assert besar[0] > kecil[0], (kecil, besar)
    assert besar[1] > kecil[1], (kecil, besar)


def test_koreksi_muka_memakai_dimensi_yang_diberikan():
    dx, dy = perception.koreksi_muka(0.0, panjang=6.0, lebar=2.0)
    assert abs(dx - 3.0) < 1e-9, dx
    dx, dy = perception.koreksi_muka(math.pi / 2, panjang=6.0, lebar=2.0)
    assert abs(dy - 1.0) < 1e-9, dy


if __name__ == '__main__':
    for nama, fn in sorted(globals().items()):
        if nama.startswith('test_'):
            fn()
            print('ok ', nama)
    print('semua lolos')
