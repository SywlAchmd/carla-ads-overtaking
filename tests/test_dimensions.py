"""Uji penaksir dimensi kendaraan dari kotak deteksi. Tanpa CARLA, tanpa torch.

Kotak dibangkitkan dari dimensi yang diketahui lewat model kamera yang sama,
lalu dicek apakah penaksir mengembalikannya. Yang dikunci: panjang BARU teramati
setelah sudut pandang menyapu, dan kotak yang terpotong tepi citra tidak boleh
ikut menarik taksiran.
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import config                                                         # noqa: E402
import perception                                                     # noqa: E402

LENGTH, WIDTH, HEIGHT = 4.605, 1.932, 1.855


def _box(theta, d, length=LENGTH, width=WIDTH, height=HEIGHT, u_center=None):
    """Kotak deteksi untuk target berdimensi diketahui pada (d, theta)."""
    silhouette = length * abs(math.sin(theta)) + width * abs(math.cos(theta))
    w_px = silhouette * perception.F_PIXEL / d
    h_px = height * perception.F_PIXEL / d
    u = config.CAMERA_WIDTH / 2.0 if u_center is None else u_center
    v = config.CAMERA_HEIGHT / 2.0
    return [u - w_px / 2, v - h_px / 2, u + w_px / 2, v + h_px / 2, 0.9]


def _sweep(dim, angle_deg, d=18.0, repeat=6):
    for _ in range(repeat):
        for deg in angle_deg:
            dim.observe(_box(math.radians(deg), d), d, math.radians(deg))


def test_prior_used_before_any_observation():
    dim = perception.VehicleDimensions()
    pj, lb = dim.size()
    assert abs(pj - config.PRIOR_LENGTH) < 1e-6, pj
    assert abs(lb - config.PRIOR_WIDTH) < 1e-6, lb
    assert dim.height is None


def test_prior_is_not_simulator_dimensions():
    """Kalau priornya OTHER_LENGTH/OTHER_WIDTH, jalur vision diam-diam meminjam
    ground truth dan seluruh maksud bagian 28 batal."""
    assert abs(config.PRIOR_LENGTH - config.OTHER_LENGTH) > 0.3
    assert config.PRIOR_LENGTH > config.OTHER_LENGTH      # prior konservatif


def test_width_observed_with_target_always_ahead():
    """theta ~ 0: siluet = lebar saja. Lebar harus benar, panjang TIDAK."""
    dim = perception.VehicleDimensions()
    _sweep(dim, [0.0, 1.0, -1.0], d=25.0, repeat=10)
    pj, lb = dim.size()
    assert abs(lb - WIDTH) < 0.10, lb
    assert dim.observed < 0.02, dim.observed       # panjang belum teramati


def test_length_observed_after_angle_sweeps():
    """Sudut menyapu 0-42 deg, seperti selama menyalip sungguhan sebelum target
    keluar dari fov 90 deg. Panjang bergerak dari prior 5,38 ke arah 4,61."""
    dim = perception.VehicleDimensions()
    for _ in range(8):
        for deg in (0.0, 8.0, 16.0, 25.0, 35.0, 42.0):
            for d in (30.0, 20.0, 12.0):
                dim.observe(_box(math.radians(deg), d), d, math.radians(deg))
    pj, lb = dim.size()
    assert abs(pj - LENGTH) < 0.40, pj
    assert abs(lb - WIDTH) < 0.15, lb
    assert dim.observed > 0.05, dim.observed


def test_residual_length_bias_toward_safe_side():
    """Prior lebih panjang daripada target, dan panjang hanya teramati lemah pada
    theta <= 42 deg. Sisa biasnya karena itu TERLALU PANJANG -- zona aman jadi
    sedikit lebih besar daripada perlu, bukan lebih kecil. Kalau suatu saat tanda
    ini berbalik, zona menyusut diam-diam dan uji ini harus menahannya."""
    dim = perception.VehicleDimensions()
    for _ in range(8):
        for deg in (0.0, 8.0, 16.0, 25.0, 35.0, 42.0):
            for d in (30.0, 20.0, 12.0):
                dim.observe(_box(math.radians(deg), d), d, math.radians(deg))
    assert dim.size()[0] >= LENGTH, dim.size()


def test_height_measured_directly_regardless_of_angle():
    """Tinggi tidak pernah bergantung sudut pandang -- itu yang membuatnya
    berguna sebagai bukti bahwa balik-proyeksinya benar."""
    dim = perception.VehicleDimensions()
    for deg in (0.0, 20.0, 40.0):
        for d in (10.0, 20.0, 35.0):
            dim.observe(_box(math.radians(deg), d), d, math.radians(deg))
    assert abs(dim.height - HEIGHT) < 0.05, dim.height


def test_box_clipped_by_image_edge_rejected():
    """REGRESI bagian 28.2. Kotak yang menyentuh tepi punya lebar terpotong, dan
    itu terjadi persis saat ego berdampingan -- sudut pandang yang paling
    informatif. Memakainya menarik taksiran panjang ke bawah."""
    dim = perception.VehicleDimensions()
    th = math.radians(35.0)
    full = _box(th, 12.0)
    width_px = full[2] - full[0]
    tight = _box(th, 12.0, u_center=width_px / 2.0 - 5.0)   # x1 keluar kiri
    assert tight[0] <= 1.0, tight
    assert dim.observe(tight, 12.0, th) is False
    assert dim.n_observations == 0
    assert dim.observe(full, 12.0, th) is True
    assert dim.n_observations == 1


def test_implausible_observation_rejected():
    dim = perception.VehicleDimensions()
    th = math.radians(20.0)
    assert dim.observe(_box(th, 18.0), 0.2, th) is False          # terlalu dekat
    assert dim.observe(_box(th, 18.0), 1e4, th) is False          # di luar jangkauan
    assert dim.n_observations == 0


def test_estimate_clamped_to_plausible_range():
    dim = perception.VehicleDimensions()
    th = math.radians(30.0)
    for _ in range(50):
        dim.observe(_box(th, 15.0, length=40.0, width=9.0), 15.0, th)
    pj, lb = dim.size()
    assert 3.0 <= pj <= 12.0, pj
    assert 1.4 <= lb <= 2.1, lb      # batas PP 55/2012


def test_zone_from_dimensions_keeps_old_values():
    """Jalur ground truth tidak boleh bergeser sedikit pun oleh refactor ini."""
    a, b = config.zone_from_dimensions(config.OTHER_LENGTH, config.OTHER_WIDTH)
    assert abs(a - config.ELLIPSE_A) < 1e-12, a
    assert abs(b - config.ELLIPSE_B) < 1e-12, b


def test_zone_never_complex():
    """REGRESI: B harus melebihi setengah lebar, kalau tidak akar pangkat empat
    dari bilangan negatif memberi hasil kompleks dan run mati di tengah."""
    for lane_width in (2.6, 3.0, 3.4, 3.5, 4.2):
        for other_width in (1.4, 1.9, 2.1):
            a, b = config.zone_from_dimensions(config.PRIOR_LENGTH, other_width, lane_width)
            assert isinstance(a, float) and isinstance(b, float)
            assert a > 0 and b > 0, (lane_width, other_width, a, b)
            assert b > (config.EGO_WIDTH + other_width) / 2 + config.SAFE_DISTANCE


def test_zone_grows_for_larger_vehicle():
    small = config.zone_from_dimensions(4.0, 1.7)
    large = config.zone_from_dimensions(6.0, 2.3)
    assert large[0] > small[0], (small, large)
    assert large[1] > small[1], (small, large)


def test_face_correction_uses_given_dimensions():
    dx, dy = perception.face_correction(0.0, length=6.0, width=2.0)
    assert abs(dx - 3.0) < 1e-9, dx
    dx, dy = perception.face_correction(math.pi / 2, length=6.0, width=2.0)
    assert abs(dy - 1.0) < 1e-9, dy


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            fn()
            print('ok ', name)
    print('all passed')
