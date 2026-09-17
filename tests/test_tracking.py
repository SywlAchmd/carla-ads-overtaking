"""Uji pelacakan dan Kalman filter. Tanpa simulator, tanpa torch.

Yang diuji di sini bukan "apakah kodenya jalan", melainkan tiga klaim yang
dipakai docstring `tracking.py` sebagai alasan modul ini ada:

  1. kompensasi gerak ego benar -- translasi meniadakan, rotasi tidak;
  2. kecepatan relatif bisa dipulihkan dari deretan posisi;
  3. derau depth 0,5 m tidak menembus jadi derau kecepatan yang mengganggu FSM.
"""
import math
import os
import sys

import numpy as np

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, AKAR)

import config
import tracking


def _kotak(cx, lebar=60.0):
    """Kotak citra berpusat di cx; isinya tidak penting selain untuk IoU."""
    return [cx - lebar / 2, 300.0, cx + lebar / 2, 300.0 + lebar]


def _R(sigma_d=config.TRACK_SIGMA_D, sigma_y=0.3):
    return np.diag([sigma_d ** 2, sigma_y ** 2])


def _suapi(pel, posisi, dt=0.1, conf=0.9, cx=640.0, **ego):
    """Satu frame berisi satu deteksi di `posisi` = (x, y) frame ego."""
    return pel.update([_kotak(cx)], [conf], [posisi], [_R()], dt, **ego)


def test_iou_matriks():
    a = np.array([[0.0, 0, 10, 10]])
    b = np.array([[0.0, 0, 10, 10], [5.0, 0, 15, 10], [20.0, 0, 30, 10]])
    m = tracking.iou_matriks(a, b)
    assert abs(m[0, 0] - 1.0) < 1e-9
    assert abs(m[0, 1] - 50 / 150) < 1e-9        # potong 50, gabung 150
    assert m[0, 2] == 0.0
    assert tracking.iou_matriks(np.empty((0, 4)), b).shape == (0, 3)


def test_tugaskan_menghormati_gerbang_dan_tidak_memasangkan_ganda():
    # kolom 0 termurah untuk kedua baris; baris 1 tidak boleh ikut merebutnya
    biaya = np.array([[0.1, 0.9], [0.2, 0.95]])
    assert tracking.tugaskan(biaya, 0.5) == [(0, 0)]
    assert tracking.tugaskan(biaya, 1.0) == [(0, 0), (1, 1)]
    assert tracking.tugaskan(biaya, 0.05) == []          # semua di atas gerbang
    assert tracking.tugaskan(np.empty((0, 3)), 1.0) == []


def test_translasi_ego_meniadakan():
    """Halangan yang diam RELATIF terhadap ego harus tetap di tempatnya, berapa
    pun laju ego. Ini inti turunan di docstring: suku v_ego saling menghapus."""
    t = tracking.Track(1, [40.0, 0.0], _R(), _kotak(640))
    t.x = np.array([40.0, 0.0, 0.0, 0.0])
    t.prediksi(0.1, u_ego=13.4, a_ego=0.0, w_ego=0.0)
    assert np.allclose(t.x, [40.0, 0.0, 0.0, 0.0], atol=1e-12), t.x


def test_percepatan_ego_mengubah_kecepatan_relatif():
    """Ego menambah laju 3 m/s² selama 0,1 s -> halangan tampak melambat 0,3 m/s."""
    t = tracking.Track(1, [40.0, 0.0], _R(), _kotak(640))
    t.x = np.array([40.0, 0.0, 0.0, 0.0])
    t.prediksi(0.1, u_ego=13.4, a_ego=3.0, w_ego=0.0)
    assert abs(t.x[2] - (-0.3)) < 1e-12, t.x


def test_rotasi_ego_memutar_halangan():
    """Ego berbelok kiri (yaw rate positif RH) -> halangan bergeser ke y negatif."""
    t = tracking.Track(1, [10.0, 0.0], _R(), _kotak(640))
    t.x = np.array([10.0, 0.0, 0.0, 0.0])
    t.prediksi(0.1, u_ego=13.4, a_ego=0.0, w_ego=0.1)
    assert abs(t.x[0] - 10.0 * math.cos(0.01)) < 1e-9
    assert abs(t.x[1] + 10.0 * math.sin(0.01)) < 1e-9
    assert t.x[1] < 0.0


def test_track_baru_belum_dilaporkan_sebelum_n_init():
    pel = tracking.Pelacak()
    for k in range(config.TRACK_N_INIT - 1):
        assert len(_suapi(pel, (60.0, 0.0))) == 0, f'dilaporkan terlalu dini di frame {k}'
    assert len(_suapi(pel, (60.0, 0.0))) == 1


def test_memulihkan_kecepatan_relatif():
    """Skenario S1: target 7,0 m/s, ego 13,4 m/s -> kecepatan relatif -6,4 m/s.

    Kamera hanya melihat posisi; kecepatan harus keluar dari deretnya."""
    pel, dt, v_rel = tracking.Pelacak(), 0.1, 7.0 - 13.4
    keluar = np.empty((0, 4))
    for k in range(40):
        keluar = _suapi(pel, (60.0 + v_rel * k * dt, 0.0), dt=dt, u_ego=13.4)
    assert len(keluar) == 1
    assert abs(keluar[0, 2] - v_rel) < 0.05, keluar[0]
    assert abs(keluar[0, 3]) < 0.05, keluar[0]


def test_derau_depth_tidak_menembus_ke_kecepatan():
    """Derau ukur 0,5 m di 10 Hz -> beda mentah dua frame ~7 m/s. Yang dilaporkan
    harus jauh di bawah DV_TRIGGER, kalau tidak FSM akan memicu karena derau."""
    acak = np.random.default_rng(0)
    pel, dt = tracking.Pelacak(), 0.1
    laju = []
    for k in range(80):
        x = 40.0 + acak.normal(0.0, config.TRACK_SIGMA_D)
        keluar = _suapi(pel, (x, acak.normal(0.0, 0.3)), dt=dt, u_ego=13.4)
        if len(keluar) and k > 20:
            laju.append(np.hypot(keluar[0, 2], keluar[0, 3]))
    assert laju, 'track tidak pernah dilaporkan'
    assert max(laju) < config.DV_TRIGGER, f'derau kecepatan {max(laju):.2f} m/s'


def test_deteksi_lemah_tidak_melahirkan_track():
    pel = tracking.Pelacak()
    for _ in range(10):
        keluar = _suapi(pel, (60.0, 0.0), conf=config.TRACK_CONF_RENDAH)
    assert len(keluar) == 0, 'deteksi berkeyakinan rendah tidak boleh jadi halangan baru'
    assert not pel.track


def test_deteksi_lemah_menahan_track_yang_memudar():
    """Kendaraan di 50 m: keyakinan jatuh ke 0,62 lalu ke bawah 0,5 (bagian 18.2).
    Track-nya harus bertahan, bukan hilang-timbul."""
    pel = tracking.Pelacak()
    for _ in range(5):
        _suapi(pel, (50.0, 0.0))
    for k in range(20):                       # keyakinan memudar di bawah ambang tinggi
        keluar = _suapi(pel, (50.0, 0.0), conf=0.35)
    assert len(keluar) == 1, 'track hilang padahal masih terdeteksi lemah'
    assert pel.track[0].hilang == 0


def test_track_melayang_lalu_dibuang():
    pel = tracking.Pelacak()
    for _ in range(5):
        _suapi(pel, (40.0, 0.0))
    kosong = dict(dt=0.1, u_ego=13.4)
    for k in range(config.TRACK_MAX_HILANG):
        keluar = pel.update(np.empty((0, 4)), [], np.empty((0, 2)), np.empty((0, 2, 2)),
                            **kosong)
        assert len(keluar) == 1, f'halangan lenyap di frame melayang ke-{k + 1}'
    keluar = pel.update(np.empty((0, 4)), [], np.empty((0, 2)), np.empty((0, 2, 2)), **kosong)
    assert len(keluar) == 0 and not pel.track


def test_dua_kendaraan_tidak_tertukar():
    """S3: satu di lajur ego, satu di lajur menyalip. Kotaknya terpisah di citra."""
    pel, dt = tracking.Pelacak(), 0.1
    for k in range(15):
        t = k * dt
        keluar = pel.update([_kotak(500.0), _kotak(800.0)], [0.95, 0.9],
                            [(60.0 - 6.4 * t, 0.0), (-10.0 + 0.5 * t, -3.5)],
                            [_R(), _R()], dt, u_ego=13.4)
    assert len(keluar) == 2
    depan = keluar[np.argmax(keluar[:, 0])]
    belakang = keluar[np.argmin(keluar[:, 0])]
    assert abs(depan[1]) < 0.5 and abs(belakang[1] + 3.5) < 0.5, keluar
    assert depan[2] < -3.0 and belakang[2] > 0.0, keluar


if __name__ == '__main__':
    for nama, fn in sorted(globals().items()):
        if nama.startswith('test_'):
            fn()
            print(f'ok  {nama}')
    print('semua lolos')
