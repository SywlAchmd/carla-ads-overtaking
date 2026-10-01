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

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import config
import tracking


def _box(cx, width=60.0):
    """Kotak citra berpusat di cx; isinya tidak penting selain untuk IoU."""
    return [cx - width / 2, 300.0, cx + width / 2, 300.0 + width]


def _R(sigma_d=config.TRACK_SIGMA_D, sigma_y=0.3):
    return np.diag([sigma_d ** 2, sigma_y ** 2])


def _feed(trk, positions, dt=0.1, conf=0.9, cx=640.0, **ego):
    """Satu frame berisi satu deteksi di `positions` = (x, y) frame ego."""
    return trk.update([_box(cx)], [conf], [positions], [_R()], dt, **ego)


def test_iou_matrix():
    a = np.array([[0.0, 0, 10, 10]])
    b = np.array([[0.0, 0, 10, 10], [5.0, 0, 15, 10], [20.0, 0, 30, 10]])
    m = tracking.iou_matrix(a, b)
    assert abs(m[0, 0] - 1.0) < 1e-9
    assert abs(m[0, 1] - 50 / 150) < 1e-9        # potong 50, gabung 150
    assert m[0, 2] == 0.0
    assert tracking.iou_matrix(np.empty((0, 4)), b).shape == (0, 3)


def test_assign_respects_gate_and_never_double_pairs():
    # kolom 0 termurah untuk kedua baris; baris 1 tidak boleh ikut merebutnya
    cost = np.array([[0.1, 0.9], [0.2, 0.95]])
    assert tracking.assign(cost, 0.5) == [(0, 0)]
    assert tracking.assign(cost, 1.0) == [(0, 0), (1, 1)]
    assert tracking.assign(cost, 0.05) == []          # semua di atas gerbang
    assert tracking.assign(np.empty((0, 3)), 1.0) == []


def test_ego_translation_cancels():
    """Halangan yang diam RELATIF terhadap ego harus tetap di tempatnya, berapa
    pun laju ego. Ini inti turunan di docstring: suku v_ego saling menghapus."""
    t = tracking.Track(1, [40.0, 0.0], _R(), _box(640))
    t.x = np.array([40.0, 0.0, 0.0, 0.0])
    t.prediction(0.1, u_ego=13.4, a_ego=0.0, w_ego=0.0)
    assert np.allclose(t.x, [40.0, 0.0, 0.0, 0.0], atol=1e-12), t.x


def test_ego_acceleration_changes_relative_velocity():
    """Ego menambah laju 3 m/s² selama 0,1 s -> halangan tampak melambat 0,3 m/s."""
    t = tracking.Track(1, [40.0, 0.0], _R(), _box(640))
    t.x = np.array([40.0, 0.0, 0.0, 0.0])
    t.prediction(0.1, u_ego=13.4, a_ego=3.0, w_ego=0.0)
    assert abs(t.x[2] - (-0.3)) < 1e-12, t.x


def test_ego_rotation_rotates_obstacle():
    """Ego berbelok kiri (yaw rate positif RH) -> halangan bergeser ke y negatif."""
    t = tracking.Track(1, [10.0, 0.0], _R(), _box(640))
    t.x = np.array([10.0, 0.0, 0.0, 0.0])
    t.prediction(0.1, u_ego=13.4, a_ego=0.0, w_ego=0.1)
    assert abs(t.x[0] - 10.0 * math.cos(0.01)) < 1e-9
    assert abs(t.x[1] + 10.0 * math.sin(0.01)) < 1e-9
    assert t.x[1] < 0.0


def test_new_track_not_reported_before_n_init():
    trk = tracking.Tracker()
    for k in range(config.TRACK_N_INIT - 1):
        assert len(_feed(trk, (60.0, 0.0))) == 0, f'reported too early at frame {k}'
    assert len(_feed(trk, (60.0, 0.0))) == 1


def test_recovers_relative_velocity():
    """Skenario S1: target 7,0 m/s, ego 13,4 m/s -> kecepatan relatif -6,4 m/s.

    Kamera hanya melihat posisi; kecepatan harus keluar dari deretnya."""
    trk, dt, v_rel = tracking.Tracker(), 0.1, 7.0 - 13.4
    out = np.empty((0, 4))
    for k in range(40):
        out = _feed(trk, (60.0 + v_rel * k * dt, 0.0), dt=dt, u_ego=13.4)
    assert len(out) == 1
    assert abs(out[0, 2] - v_rel) < 0.05, out[0]
    assert abs(out[0, 3]) < 0.05, out[0]


def test_depth_noise_does_not_leak_into_velocity():
    """Derau ukur 0,5 m di 10 Hz -> beda mentah dua frame ~7 m/s. Yang dilaporkan
    harus jauh di bawah DV_TRIGGER, kalau tidak FSM akan memicu karena derau."""
    rng = np.random.default_rng(0)
    trk, dt = tracking.Tracker(), 0.1
    speed = []
    for k in range(80):
        x = 40.0 + rng.normal(0.0, config.TRACK_SIGMA_D)
        out = _feed(trk, (x, rng.normal(0.0, 0.3)), dt=dt, u_ego=13.4)
        if len(out) and k > 20:
            speed.append(np.hypot(out[0, 2], out[0, 3]))
    assert speed, 'track never reported'
    assert max(speed) < config.DV_TRIGGER, f'velocity noise {max(speed):.2f} m/s'


def test_weak_detection_does_not_spawn_track():
    trk = tracking.Tracker()
    for _ in range(10):
        out = _feed(trk, (60.0, 0.0), conf=config.TRACK_CONF_LOW)
    assert len(out) == 0, 'a low-confidence detection must not become a new obstacle'
    assert not trk.track


def test_weak_detection_keeps_fading_track():
    """Kendaraan di 50 m: keyakinan jatuh ke 0,62 lalu ke bawah 0,5 (bagian 18.2).
    Track-nya harus bertahan, bukan hilang-timbul."""
    trk = tracking.Tracker()
    for _ in range(5):
        _feed(trk, (50.0, 0.0))
    for k in range(20):                       # keyakinan memudar di bawah ambang tinggi
        out = _feed(trk, (50.0, 0.0), conf=0.35)
    assert len(out) == 1, 'track lost although still weakly detected'
    assert trk.track[0].lost == 0


def test_track_coasts_then_dropped():
    trk = tracking.Tracker()
    for _ in range(5):
        _feed(trk, (40.0, 0.0))
    empty = dict(dt=0.1, u_ego=13.4)
    for k in range(config.TRACK_MAX_LOST):
        out = trk.update(np.empty((0, 4)), [], np.empty((0, 2)), np.empty((0, 2, 2)),
                            **empty)
        assert len(out) == 1, f'obstacle vanished at coasting frame {k + 1}'
    out = trk.update(np.empty((0, 4)), [], np.empty((0, 2)), np.empty((0, 2, 2)), **empty)
    assert len(out) == 0 and not trk.track


def test_two_vehicles_not_swapped():
    """S3: satu di lajur ego, satu di lajur menyalip. Kotaknya terpisah di citra."""
    trk, dt = tracking.Tracker(), 0.1
    for k in range(15):
        t = k * dt
        out = trk.update([_box(500.0), _box(800.0)], [0.95, 0.9],
                            [(60.0 - 6.4 * t, 0.0), (-10.0 + 0.5 * t, -3.5)],
                            [_R(), _R()], dt, u_ego=13.4)
    assert len(out) == 2
    front = out[np.argmax(out[:, 0])]
    rear = out[np.argmin(out[:, 0])]
    assert abs(front[1]) < 0.5 and abs(rear[1] + 3.5) < 0.5, out
    assert front[2] < -3.0 and rear[2] > 0.0, out


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            fn()
            print(f'ok  {name}')
    print('all passed')
