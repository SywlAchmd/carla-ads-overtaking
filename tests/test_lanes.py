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

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, AKAR)

import config                                                         # noqa: E402
import lanes                                                          # noqa: E402

BENTUK_CITRA = (config.KAMERA_TINGGI, config.KAMERA_LEBAR)
BENTUK_MASKER = (384, 640)


def test_letterbox_1280x720_memberi_pad_12_baris():
    """1280x720 dikecilkan 0,5 jadi 640x360, lalu dibingkai ke 640x384.

    Kalau padding 12 baris ini terlupa, seluruh hasil bergeser beberapa meter --
    dan tidak ada yang meledak, hanya salah diam-diam.
    """
    r, pad_u, pad_v = lanes.letterbox_ke_citra(BENTUK_MASKER, BENTUK_CITRA)
    assert abs(r - 0.5) < 1e-12, r
    assert abs(pad_u) < 1e-12, pad_u
    assert abs(pad_v - 12.0) < 1e-12, pad_v


def _piksel(x, y):
    """Kebalikan `lanes.ipm`: titik jalan -> piksel citra."""
    f = lanes.F_PIKSEL
    return (config.KAMERA_LEBAR / 2.0 - y * f / x,
            config.KAMERA_TINGGI / 2.0 + f * config.KAMERA_Z / x)


def test_ipm_bolak_balik():
    for x, y in ((10.0, 0.0), (25.0, 1.75), (7.0, -3.5), (40.0, 2.0)):
        u, v = _piksel(x, y)
        xx, yy = lanes.ipm(u, v)
        assert abs(xx - x) < 1e-6, (x, y, xx)
        assert abs(yy - y) < 1e-6, (x, y, yy)


def test_ipm_di_atas_horizon_tak_hingga():
    """Baris di atas titik hilang tidak memotong permukaan jalan."""
    x, _ = lanes.ipm(640.0, config.KAMERA_TINGGI / 2.0 - 5.0)
    assert not np.isfinite(x), x


def test_kisi_menemukan_lebar_walau_satu_garis_hilang():
    """Justru inilah alasan kisi dipakai: median jarak antar garis memberi 5,25 m
    di sini (rata-rata 3,5 dan 7,0), sedangkan kisinya tetap 3,50 m."""
    offset = np.array([-7.0, -3.5, 3.5])          # 0,0 sengaja dihilangkan
    w, _ = lanes._kisi(offset, np.ones(3))
    assert abs(w - 3.5) < 0.02, w


def test_kisi_bertahan_terhadap_satu_garis_palsu():
    """Marka palsu berbobot kecil tidak boleh menggeser lebar lajur."""
    offset = np.array([-7.0, -3.5, 0.0, 3.5, -5.1])
    bobot = np.array([300.0, 400.0, 500.0, 300.0, 25.0])
    w, _ = lanes._kisi(offset, bobot)
    assert abs(w - 3.5) < 0.05, w


def test_dev_lajur_mengikuti_pergeseran_ego():
    """Ego digeser 0,7 m ke kiri dari tengah lajur -> dev_lajur = +0,7 m."""
    for geser in (-1.2, -0.7, 0.0, 0.7, 1.2):
        garis = np.array([-5.25, -1.75, 1.75, 5.25]) - geser
        g = lanes.GeometriLajur(garis, np.full(4, 300.0), 0.0, 4000)
        assert abs(g.lebar_lajur - 3.5) < 0.02, (geser, g.lebar_lajur)
        assert abs(g.dev_lajur - geser) < 0.02, (geser, g.dev_lajur)


def test_tengah_lajur_sebelah_berjarak_satu_lebar():
    garis = np.array([-5.25, -1.75, 1.75, 5.25])
    g = lanes.GeometriLajur(garis, np.full(4, 300.0), 0.0, 4000)
    assert abs(g.tengah_lajur(+1) - 3.5) < 0.02, g.tengah_lajur(+1)
    assert abs(g.tengah_lajur(-1) + 3.5) < 0.02, g.tengah_lajur(-1)


def test_sisa_kisi_kecil_saat_cocok_besar_saat_tidak():
    rapi = lanes.GeometriLajur(np.array([-3.5, 0.0, 3.5]), np.full(3, 300.0), 0.0, 3000)
    assert rapi.sisa_kisi < 0.01, rapi.sisa_kisi
    kacau = lanes.GeometriLajur(np.array([-3.5, 0.0, 2.1]), np.full(3, 300.0), 0.0, 3000)
    assert kacau.sisa_kisi > rapi.sisa_kisi, (kacau.sisa_kisi, rapi.sisa_kisi)


def _masker_buatan(garis_y, yaw_deg=0.0, x0=8.0, x1=40.0):
    """Masker biner berisi garis lurus pada offset `garis_y`, frame ego."""
    m = np.zeros(BENTUK_MASKER, dtype=np.uint8)
    r, pad_u, pad_v = lanes.letterbox_ke_citra(BENTUK_MASKER, BENTUK_CITRA)
    b = -math.tan(math.radians(yaw_deg))
    for c in garis_y:
        for x in np.arange(x0, x1, 0.05):
            u, v = _piksel(x, c + b * x)
            um, vm = u * r + pad_u, v * r + pad_v
            if 0 <= int(vm) < BENTUK_MASKER[0] and 0 <= int(um) < BENTUK_MASKER[1]:
                m[int(vm), int(um)] = 1
    return m


def test_pipeline_lengkap_dari_masker_buatan():
    """Tiga garis berjarak 3,5 m, ego 0,4 m di kanan tengah lajur, hadap +3 deg."""
    geser, yaw = -0.4, 3.0
    m = _masker_buatan(np.array([-5.25, -1.75, 1.75, 5.25]) - geser, yaw_deg=yaw)
    g = lanes.dari_masker(m, BENTUK_CITRA)
    assert g is not None
    assert abs(g.lebar_lajur - 3.5) < 0.12, g.lebar_lajur
    assert abs(g.dev_lajur - geser) < 0.12, g.dev_lajur
    assert abs(math.degrees(g.yaw) - yaw) < 0.25, math.degrees(g.yaw)


def test_garis_hanya_pada_slot_yang_didukung_marka():
    """Kisi itu tak berhingga. Yang boleh digambar hanya slot yang punya marka --
    kalau tidak, garis lajur ditarik di atas tanggul dan pembatas."""
    garis = np.array([-5.25, -1.75, 1.75, 5.25])
    g = lanes.GeometriLajur(garis, np.full(4, 300.0), 0.0, 4000)
    assert len(g.garis()) == 4, len(g.garis())
    # satu marka hilang -> tiga garis, bukan empat, dan bukan pula tak berhingga
    g2 = lanes.GeometriLajur(garis[[0, 1, 3]], np.full(3, 300.0), 0.0, 3000)
    assert len(g2.garis()) == 3, len(g2.garis())
    assert abs(g2.lebar_lajur - 3.5) < 0.05


def test_penghalusan_mengalahkan_argmax_histogram():
    """REGRESI bagian 28.1. Argmax histogram kasar karena skornya dihitung pada
    bin 0,10 m; penghalusan kuadrat terkecil menggantikan langkah terakhirnya.

    Diukur pada sapuan sudut: RMS 0,210 -> 0,095 deg, maksimum 0,305 -> 0,102.
    Memperhalus langkah PENCARIAN tidak menolong -- itu sudah diuji dan malah
    memburuk -- jadi uji ini menjaga penyelesaiannya, bukan kisinya.
    """
    kasar, halus = [], []
    for yaw in (-1.7, -0.4, 0.0, 0.8, 2.7):
        m = _masker_buatan(np.array([-5.25, -1.75, 1.75, 5.25]), yaw_deg=yaw)
        t = lanes.titik_lajur(m, BENTUK_CITRA)
        b0 = lanes._kemiringan_bersama(t)
        kasar.append(math.degrees(-math.atan(b0)) - yaw)
        halus.append(math.degrees(-math.atan(lanes._haluskan(t, b0))) - yaw)
    kasar, halus = np.array(kasar), np.array(halus)
    assert np.abs(halus).max() < np.abs(kasar).max(), (kasar, halus)
    assert np.abs(halus).max() < 0.15, halus


def test_masker_kosong_mengembalikan_none():
    assert lanes.dari_masker(np.zeros(BENTUK_MASKER, dtype=np.uint8), BENTUK_CITRA) is None


def test_satu_garis_saja_tidak_memberi_lebar():
    """Satu marka tidak cukup menentukan kisi; jangan mengarang angka."""
    g = lanes.dari_masker(_masker_buatan([1.75]), BENTUK_CITRA)
    if g is not None:                       # boleh terbaca, tapi lebarnya tak tentu
        assert g.lebar_lajur is None or len(g.offset) >= 2
        if g.lebar_lajur is None:
            assert g.dev_lajur is None


if __name__ == '__main__':
    for nama, fn in sorted(globals().items()):
        if nama.startswith('test_'):
            fn()
            print('ok ', nama)
    print('semua lolos')
