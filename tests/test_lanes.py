"""Uji geometri lajur dari masker segmentasi. Tanpa CARLA, tanpa torch, tanpa GPU.

Maskernya dibangkitkan sendiri dari geometri yang diketahui, lalu dicek apakah
`lanes` mengembalikan angka yang sama. Yang dikunci bukan ketelitian terhadap
YOLOPX -- itu tugas `check_lanes.py` yang butuh simulator -- melainkan bahwa
balik-proyeksi dan pencocokan kisinya benar.
"""
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import config                                                         # noqa: E402
import lanes                                                          # noqa: E402

IMAGE_SHAPE = (config.CAMERA_HEIGHT, config.CAMERA_WIDTH)
MASK_SHAPE = (384, 640)


def test_letterbox_1280x720_gives_12_row_pad():
    """1280x720 dikecilkan 0,5 jadi 640x360, lalu dibingkai ke 640x384.

    Kalau padding 12 baris ini terlupa, seluruh hasil bergeser beberapa meter --
    dan tidak ada yang meledak, hanya salah diam-diam.
    """
    r, pad_u, pad_v = lanes.letterbox_to_image(MASK_SHAPE, IMAGE_SHAPE)
    assert abs(r - 0.5) < 1e-12, r
    assert abs(pad_u) < 1e-12, pad_u
    assert abs(pad_v - 12.0) < 1e-12, pad_v


def _pixels(x, y):
    """Kebalikan `lanes.ipm`: titik jalan -> piksel citra."""
    f = lanes.F_PIXEL
    return (config.CAMERA_WIDTH / 2.0 - y * f / x,
            config.CAMERA_HEIGHT / 2.0 + f * config.CAMERA_Z / x)


def test_ipm_round_trip():
    for x, y in ((10.0, 0.0), (25.0, 1.75), (7.0, -3.5), (40.0, 2.0)):
        u, v = _pixels(x, y)
        xx, yy = lanes.ipm(u, v)
        assert abs(xx - x) < 1e-6, (x, y, xx)
        assert abs(yy - y) < 1e-6, (x, y, yy)


def test_ipm_above_horizon_is_infinite():
    """Baris di atas titik hilang tidak memotong permukaan jalan."""
    x, _ = lanes.ipm(640.0, config.CAMERA_HEIGHT / 2.0 - 5.0)
    assert not np.isfinite(x), x


def test_lattice_finds_width_with_one_line_missing():
    """Justru inilah alasan kisi dipakai: median jarak antar garis memberi 5,25 m
    di sini (rata-rata 3,5 dan 7,0), sedangkan kisinya tetap 3,50 m."""
    offset = np.array([-7.0, -3.5, 3.5])          # 0,0 sengaja dihilangkan
    w, _ = lanes._lattice(offset, np.ones(3))
    assert abs(w - 3.5) < 0.02, w


def test_lattice_robust_to_one_fake_line():
    """Marka palsu berbobot kecil tidak boleh menggeser lebar lajur."""
    offset = np.array([-7.0, -3.5, 0.0, 3.5, -5.1])
    weight = np.array([300.0, 400.0, 500.0, 300.0, 25.0])
    w, _ = lanes._lattice(offset, weight)
    assert abs(w - 3.5) < 0.05, w


def test_lane_dev_follows_ego_shift():
    """Ego digeser 0,7 m ke kiri dari tengah lajur -> lane_dev = +0,7 m."""
    for shift in (-1.2, -0.7, 0.0, 0.7, 1.2):
        lines = np.array([-5.25, -1.75, 1.75, 5.25]) - shift
        g = lanes.LaneGeometry(lines, np.full(4, 300.0), 0.0, 4000)
        assert abs(g.lane_width - 3.5) < 0.02, (shift, g.lane_width)
        assert abs(g.lane_dev - shift) < 0.02, (shift, g.lane_dev)


def test_adjacent_lane_center_one_width_away():
    lines = np.array([-5.25, -1.75, 1.75, 5.25])
    g = lanes.LaneGeometry(lines, np.full(4, 300.0), 0.0, 4000)
    assert abs(g.lane_center(+1) - 3.5) < 0.02, g.lane_center(+1)
    assert abs(g.lane_center(-1) + 3.5) < 0.02, g.lane_center(-1)


def test_lattice_residual_small_on_fit_large_off_fit():
    tidy = lanes.LaneGeometry(np.array([-3.5, 0.0, 3.5]), np.full(3, 300.0), 0.0, 3000)
    assert tidy.lattice_residual < 0.01, tidy.lattice_residual
    messy = lanes.LaneGeometry(np.array([-3.5, 0.0, 2.1]), np.full(3, 300.0), 0.0, 3000)
    assert messy.lattice_residual > tidy.lattice_residual, (messy.lattice_residual, tidy.lattice_residual)


def _synthetic_mask(line_y, yaw_deg=0.0, x0=8.0, x1=40.0):
    """Masker biner berisi garis lurus pada offset `line_y`, frame ego."""
    m = np.zeros(MASK_SHAPE, dtype=np.uint8)
    r, pad_u, pad_v = lanes.letterbox_to_image(MASK_SHAPE, IMAGE_SHAPE)
    b = -math.tan(math.radians(yaw_deg))
    for c in line_y:
        for x in np.arange(x0, x1, 0.05):
            u, v = _pixels(x, c + b * x)
            um, vm = u * r + pad_u, v * r + pad_v
            if 0 <= int(vm) < MASK_SHAPE[0] and 0 <= int(um) < MASK_SHAPE[1]:
                m[int(vm), int(um)] = 1
    return m


def _synthetic_area(y_lo, y_hi, hole=None):
    """Masker area jalan: drivable bila y titik jalan di [y_lo, y_hi].

    `hole` = (x0, x1, y0, y1): petak bukan-jalan, seperti kendaraan.
    """
    r, pad_u, pad_v = lanes.letterbox_to_image(MASK_SHAPE, IMAGE_SHAPE)
    vm, um = np.mgrid[0:MASK_SHAPE[0], 0:MASK_SHAPE[1]]
    with np.errstate(invalid='ignore'):
        x, y = lanes.ipm((um - pad_u) / r, (vm - pad_v) / r)
        m = np.isfinite(x) & (y >= y_lo) & (y <= y_hi)
        if hole is not None:
            m &= ~((x >= hole[0]) & (x <= hole[1]) & (y >= hole[2]) & (y <= hole[3]))
    return m


def test_lines_outside_drivable_area_dropped():
    """Garis palsu di luar jalan (rel, jalur seberang) tidak boleh ikut kisi."""
    real = np.array([-5.25, -1.75, 1.75])
    m = _synthetic_mask(np.append(real, 8.75))
    area = _synthetic_area(-5.6, 2.1)                       # garis tepi ikut area jalan
    assert np.any(np.abs(lanes.from_mask(m, IMAGE_SHAPE).offset - 8.75) < 0.3)
    g = lanes.from_mask(m, IMAGE_SHAPE, drivable=area)
    assert not np.any(np.abs(g.offset - 8.75) < 0.3), g.offset
    assert abs(g.lane_width - 3.5) < 0.12, g.lane_width


def test_drivable_fraction_tells_lane_exists_from_not():
    area = _synthetic_area(-5.25, 1.75)                     # lajur ego + lajur kanan
    right = lanes.drivable_fraction(area, IMAGE_SHAPE, -3.5, 0.0, 3.5 / 4)
    left = lanes.drivable_fraction(area, IMAGE_SHAPE, 3.5, 0.0, 3.5 / 4)
    assert right > 0.95 and left < 0.05, (right, left)


def test_lane_marked_tells_lane_from_shoulder():
    """Lajur kanan diapit dua garis; sisi kiri cuma punya garis tepi (bahu jalan)."""
    g = lanes.from_mask(_synthetic_mask(np.array([-5.25, -1.75, 1.75])), IMAGE_SHAPE)
    assert g.lane_marked(-1) and not g.lane_marked(+1)


def test_road_edges_ignore_vehicle_beside_ego():
    """Kendaraan memotong area jalan di barisnya; tepi jalan tidak boleh ikut."""
    area = _synthetic_area(-5.25, 1.75, hole=(6.0, 22.0, -1.0, 1.75))
    right, left = lanes.road_edges(area, IMAGE_SHAPE, -3.5, 0.0)
    assert abs(right + 5.25) < 0.15 and abs(left - 1.75) < 0.15, (right, left)


def test_full_pipeline_from_synthetic_mask():
    """Tiga garis berjarak 3,5 m, ego 0,4 m di kanan tengah lajur, hadap +3 deg."""
    shift, yaw = -0.4, 3.0
    m = _synthetic_mask(np.array([-5.25, -1.75, 1.75, 5.25]) - shift, yaw_deg=yaw)
    g = lanes.from_mask(m, IMAGE_SHAPE)
    assert g is not None
    assert abs(g.lane_width - 3.5) < 0.12, g.lane_width
    assert abs(g.lane_dev - shift) < 0.12, g.lane_dev
    assert abs(math.degrees(g.yaw) - yaw) < 0.25, math.degrees(g.yaw)


def test_lines_only_on_slots_backed_by_markings():
    """Kisi itu tak berhingga. Yang boleh digambar hanya slot yang punya marka --
    kalau tidak, garis lajur ditarik di atas tanggul dan pembatas."""
    lines = np.array([-5.25, -1.75, 1.75, 5.25])
    g = lanes.LaneGeometry(lines, np.full(4, 300.0), 0.0, 4000)
    assert len(g.lines()) == 4, len(g.lines())
    # satu marka hilang -> tiga garis, bukan empat, dan bukan pula tak berhingga
    g2 = lanes.LaneGeometry(lines[[0, 1, 3]], np.full(3, 300.0), 0.0, 3000)
    assert len(g2.lines()) == 3, len(g2.lines())
    assert abs(g2.lane_width - 3.5) < 0.05


def test_refinement_beats_histogram_argmax():
    """REGRESI bagian 28.1. Argmax histogram kasar karena skornya dihitung pada
    bin 0,10 m; penghalusan kuadrat terkecil menggantikan langkah terakhirnya.

    Diukur pada sapuan sudut: RMS 0,210 -> 0,095 deg, maksimum 0,305 -> 0,102.
    Memperhalus langkah PENCARIAN tidak menolong -- itu sudah diuji dan malah
    memburuk -- jadi uji ini menjaga penyelesaiannya, bukan kisinya.
    """
    coarse, smooth = [], []
    for yaw in (-1.7, -0.4, 0.0, 0.8, 2.7):
        m = _synthetic_mask(np.array([-5.25, -1.75, 1.75, 5.25]), yaw_deg=yaw)
        t = lanes.lane_points(m, IMAGE_SHAPE)
        b0 = lanes._shared_slope(t)
        coarse.append(math.degrees(-math.atan(b0)) - yaw)
        smooth.append(math.degrees(-math.atan(lanes._refine(t, b0))) - yaw)
    coarse, smooth = np.array(coarse), np.array(smooth)
    assert np.abs(smooth).max() < np.abs(coarse).max(), (coarse, smooth)
    assert np.abs(smooth).max() < 0.15, smooth


def test_empty_mask_returns_none():
    assert lanes.from_mask(np.zeros(MASK_SHAPE, dtype=np.uint8), IMAGE_SHAPE) is None


def test_single_line_gives_no_width():
    """Satu marka tidak cukup menentukan kisi; jangan mengarang angka."""
    g = lanes.from_mask(_synthetic_mask([1.75]), IMAGE_SHAPE)
    if g is not None:                       # boleh terbaca, tapi lebarnya tak tentu
        assert g.lane_width is None or len(g.offset) >= 2
        if g.lane_width is None:
            assert g.lane_dev is None


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            fn()
            print('ok ', name)
    print('all passed')
