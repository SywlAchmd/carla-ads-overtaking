"""Pelacakan antar frame dan penapisan Kalman (rencana kerja bagian 10.3).

Murni numerik: tidak mengimpor carla, torch, maupun cv2 -- bisa diuji di terminal
tanpa menyalakan simulator (aturan 2.4, ditegakkan `tests/test_architecture.py`).

**Kenapa perlu.** Satu frame kamera hanya memberi POSISI. Kecepatan halangan --
yang dipakai FSM untuk TTC dan MPC untuk memprediksi elips sepanjang horizon --
hanya bisa datang dari beda antar frame. Beda mentah dua frame berturut pada
derau depth 0,5 m (bagian 18.4) di 10 Hz memberi derau kecepatan 5 m/s, sepuluh
kali lipat `DV_TRIGGER`. Kalman filter constant-velocity yang meredamnya.

**Asosiasi: BYTE dari ByteTrack, apa adanya.** Zhang dkk., "ByteTrack:
Multi-Object Tracking by Associating Every Detection Box", ECCV 2022,
Algorithm 1. Penerapannya pada kendaraan dengan prosedur dan hyperparameter
bawaan paper yang sama: Bihanda, Fatichah & Yuniarti, "Multi-Vehicle Tracking
and Counting Framework in Average Daily Traffic Survey Using RT-DETR and
ByteTrack", IEEE Access 12:121723-121737, 2024. Per frame:

  1. deteksi dipisah: D_high (skor > tau) dan D_low (sisanya);
  2. Kalman filter memprediksi kotak SEMUA track, termasuk yang hilang;
  3. asosiasi pertama: semua track vs D_high, kemiripan IoU antara kotak deteksi
     dan kotak PREDIKSI track, diselesaikan dengan algoritma Hungarian;
  4. asosiasi kedua: track sisa vs D_low, IoU dan Hungarian lagi. Deteksi lemah
     yang tak tercocokkan dibuang sebagai latar;
  5. track yang masih tersisa menjadi hilang, dan dihapus setelah 30 frame;
  6. D_high yang tak tercocokkan melahirkan track baru;
  7. keluaran = semua track KECUALI yang hilang.

Pasangan dengan IoU < 0,2 ditolak di kedua asosiasi. Seluruh parameter tidak
diubah dari paper; lihat `config.py`.

**Dua Kalman filter per track, untuk dua tugas berbeda.**

- *Kotak citra* [cx, cy, a, h, turunannya] -- bagian ByteTrack. Hanya dipakai
  untuk memprediksi kotak yang diasosiasikan. Derau dan dt = 1 frame dari kode
  resmi ByteTrack, tanpa perubahan.
- *Metrik frame ego* [x, y, vx, vy] -- BUKAN bagian ByteTrack, yang tidak tahu
  soal meter. Dipertahankan karena FSM dan MPC butuh posisi dan kecepatan dalam
  m dan m/s. Ia dikoreksi oleh deteksi yang sama yang dipasangkan BYTE.

**Rumus kecepatan: Baek, Kim & Yi (2024).** "Robust tracking and detection
based on radar camera fusion filtering in urban autonomous driving",
*Intelligent Service Robotics* 17:1125-1141, doi:10.1007/s11370-024-00563-0.
State [px, py, vx, vy] di frame ego SAAT INI; posisi relatif terhadap ego,
kecepatan ABSOLUT kendaraan lain. Tiap tick, urutan sama dengan paper:

1. Kompensasi gerak ego, pers. (3). Ego bergerak sepanjang busur dengan laju v
   dan yaw rate g selama dt; posisi track dipindah ke frame ego yang baru:

       p' = [[ cos(g dt), sin(g dt)],      [ -(v/g) sin(g dt)       ]
             [-sin(g dt), cos(g dt)]] p  +  [  (v/g) (1 - cos(g dt)) ]

   **Koreksi salah cetak paper:** komponen kedua translasi tercetak
   (v/g) - (v/g) sin(g dt); geometri busur memberi (v/g)(1 - cos(g dt)) --
   lihat `tests/test_tracking.py::test_ego_motion_matches_arc_geometry`.
   Paper hanya mentransformasikan posisi; kecepatan absolut di sini ikut DIPUTAR
   dengan matriks yang sama karena ia dinyatakan di frame ego (tambahan, bukan
   dari paper).
2. Prediksi target, pers. (4), constant velocity: p += v dt, v tetap; + Q.

Karena kecepatan absolut, percepatan ego tidak masuk prediksi sama sekali.
Kontrak keluaran `perception.py` tetap: `update` mengembalikan kecepatan
RELATIF, v_rel = v_abs - [v_ego, 0]; `localization.obstacles_ego_to_road`
menambahkan kecepatan ego kembali. Modul ini tetap tidak perlu tahu posisi ego,
cukup laju dan yaw rate-nya.
"""
import math

import numpy as np
from scipy.optimize import linear_sum_assignment

import config


def iou_matrix(a, b):
    """IoU tiap pasangan kotak [x1,y1,x2,y2]. a (n,4), b (m,4) -> (n,m)."""
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    if len(a) == 0 or len(b) == 0:
        return np.empty((len(a), len(b)))
    x1 = np.maximum(a[:, None, 0], b[None, :, 0])
    y1 = np.maximum(a[:, None, 1], b[None, :, 1])
    x2 = np.minimum(a[:, None, 2], b[None, :, 2])
    y2 = np.minimum(a[:, None, 3], b[None, :, 3])
    inter = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    area_a = (a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1])
    area_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    combine = area_a[:, None] + area_b[None, :] - inter
    return np.where(combine > 0, inter / np.where(combine > 0, combine, 1.0), 0.0)


def hungarian(iou, iou_min=None):
    """Penugasan Hungarian pada biaya 1 - IoU -> [(baris, kolom), ...].

    Meminimalkan TOTAL biaya seluruh pasangan sekaligus, lalu menolak pasangan
    dengan IoU < `iou_min`: lebih baik satu track kehilangan pembaruan daripada
    dikoreksi oleh deteksi milik kendaraan lain.
    """
    iou_min = config.TRACK_IOU_MIN if iou_min is None else iou_min
    iou = np.asarray(iou, dtype=float)
    if iou.size == 0:
        return []
    rows, columns = linear_sum_assignment(1.0 - iou)
    return [(int(i), int(j)) for i, j in zip(rows, columns) if iou[i, j] >= iou_min]


def _xyah(box):
    """[x1,y1,x2,y2] -> [cx, cy, a = w/h, h], ukuran Kalman filter ByteTrack."""
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    return np.array([x1 + w / 2, y1 + h / 2, w / h, h], dtype=float)


def _tlbr(xyah):
    cx, cy, a, h = xyah[:4]
    w = a * h
    return np.array([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2])


class BoxFilter:
    """Kalman filter kotak citra milik ByteTrack: state [cx, cy, a, h, vcx, vcy,
    va, vh], constant-velocity, dt = 1 frame. Rumus dan derau dari kode resmi
    ByteTrack (`yolox/tracker/kalman_filter.py`), tanpa perubahan."""

    F = np.eye(8)
    F[:4, 4:] = np.eye(4)
    H = np.eye(4, 8)

    def __init__(self, box):
        z = _xyah(box)
        wp, wv, h = config.TRACK_STD_WEIGHT_POSITION, config.TRACK_STD_WEIGHT_VELOCITY, z[3]
        self.x = np.concatenate([z, np.zeros(4)])
        std = [2 * wp * h, 2 * wp * h, 1e-2, 2 * wp * h,
               10 * wv * h, 10 * wv * h, 1e-5, 10 * wv * h]
        self.P = np.diag(np.square(std))

    def prediction(self):
        wp, wv, h = config.TRACK_STD_WEIGHT_POSITION, config.TRACK_STD_WEIGHT_VELOCITY, self.x[3]
        std = [wp * h, wp * h, 1e-2, wp * h, wv * h, wv * h, 1e-5, wv * h]
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + np.diag(np.square(std))

    def correction(self, box):
        wp, h = config.TRACK_STD_WEIGHT_POSITION, self.x[3]
        R = np.diag(np.square([wp * h, wp * h, 1e-1, wp * h]))
        S = self.H @ self.P @ self.H.T + R
        K = np.linalg.solve(S, self.H @ self.P).T          # P H^T S^-1, S simetris
        self.x = self.x + K @ (_xyah(box) - self.H @ self.x)
        self.P = self.P - K @ S @ K.T

    def box(self):
        return _tlbr(self.x)


def ego_motion(u_ego, w_ego, dt):
    """Pers. (3) Baek dkk. (2024), translasi dikoreksi -> (rot 2x2, translasi 2).

    Titik yang diam di frame ego lama berpindah ke `rot @ p + t` di frame ego
    baru, setelah ego menempuh busur laju `u_ego`, yaw rate `w_ego`, selama `dt`.
    """
    th = w_ego * dt
    c, s = math.cos(th), math.sin(th)
    rot = np.array([[c, s], [-s, c]])
    # v/g sin(g dt) dan v/g (1 - cos(g dt)), ditulis lewat th supaya tetap
    # terdefinisi saat g -> 0 (limitnya: ego maju lurus u dt)
    if abs(th) < 1e-9:
        t = np.array([-u_ego * dt, 0.0])
    else:
        r = u_ego * dt / th
        t = np.array([-r * s, r * (1.0 - c)])
    return rot, t


class Track:
    """Satu kendaraan yang sedang dilacak. `x` = state metrik [px, py, vx, vy]
    frame ego, posisi relatif dan kecepatan ABSOLUT; `kf` = Kalman filter kotak
    citra untuk asosiasi."""

    def __init__(self, id_, z, R, box, conf=0.0, u_ego=0.0):
        self.id, self.conf = id_, float(conf)
        # Kecepatan awal = ikut laju ego (relatif nol), prior yang sama dengan
        # sebelum rumus Baek dipakai.
        self.x = np.array([z[0], z[1], u_ego, 0.0], dtype=float)
        # Kecepatan awal tidak terukur sama sekali dari satu frame: ragunya
        # dipasang selebar rentang yang mungkin, supaya koreksi frame kedua
        # hampir seluruhnya menentukan kecepatan dan tidak tertahan prior.
        self.P = np.diag([R[0, 0], R[1, 1],
                          config.TRACK_SIGMA_V0 ** 2, config.TRACK_SIGMA_V0 ** 2])
        self.kf = BoxFilter(box)
        self.box = np.asarray(box, dtype=float)
        self.lost = 0                       # frame berturut tanpa pasangan

    def prediction(self, dt, u_ego, w_ego):
        self.kf.prediction()
        self.box = self.kf.box()
        # 1. pers. (3): kompensasi gerak ego; kecepatan diputar ikut frame
        rot, t = ego_motion(u_ego, w_ego, dt)
        J = np.block([[rot, np.zeros((2, 2))], [np.zeros((2, 2)), rot]])
        self.x = J @ self.x + np.concatenate([t, np.zeros(2)])
        # 2. pers. (4): constant velocity, p += v dt
        i2 = np.eye(2)
        F = np.block([[i2, dt * i2], [np.zeros((2, 2)), i2]])
        self.x = F @ self.x
        # derau proses = percepatan putih diskret; kendaraan lain boleh berubah
        # kecepatan sebesar TRACK_SIGMA_A tanpa membuat track kehilangan jejak
        Q = config.TRACK_SIGMA_A ** 2 * np.block(
            [[dt ** 4 / 4 * i2, dt ** 3 / 2 * i2], [dt ** 3 / 2 * i2, dt ** 2 * i2]])
        self.P = F @ J @ self.P @ J.T @ F.T + Q

    def relative(self, u_ego):
        """State keluaran: posisi relatif, kecepatan RELATIF terhadap ego."""
        return self.x - np.array([0.0, 0.0, u_ego, 0.0])

    def correction(self, z, R, box, conf=None):
        if conf is not None:
            self.conf = float(conf)
        H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (np.asarray(z, dtype=float) - H @ self.x)
        self.P = (np.eye(4) - K @ H) @ self.P
        self.kf.correction(box)
        self.box = self.kf.box()
        self.lost = 0


class Tracker:
    """Pelacak multi-objek BYTE. Satu instance hidup selama satu run."""

    def __init__(self):
        self.track, self._last_id = [], 0
        self._u_ego = 0.0                   # laju ego tick terakhir, untuk `visible`

    def update(self, box, conf, z, R, dt, u_ego=0.0, w_ego=0.0):
        """Satu frame. Kembali (M, 4) = [x, y, vx, vy] frame ego, relatif ego.

        `box` (n,4) piksel, `conf` (n,), `z` (n,2) posisi terukur frame ego,
        `R` (n,2,2) kovarians ukur tiap deteksi (bergantung jarak -- dibangun di
        `perception.py`, yang tahu soal sensornya).
        """
        self._u_ego = float(u_ego)
        box = np.asarray(box, dtype=float).reshape(-1, 4)
        conf = np.asarray(conf, dtype=float).reshape(-1)
        z = np.asarray(z, dtype=float).reshape(-1, 2)
        R = np.asarray(R, dtype=float).reshape(-1, 2, 2)

        # Algorithm 1 baris 3-13: D_high dan D_low
        high = conf > config.TRACK_CONF_HIGH
        # baris 14-16: prediksi SEMUA track, termasuk yang hilang
        for t in self.track:
            t.prediction(dt, u_ego, w_ego)
        # baris 17-19: asosiasi pertama, semua track vs D_high
        d_remain, t_remain = self._match(np.flatnonzero(high), box, z, R, conf, self.track)
        # baris 20-21: asosiasi kedua, track sisa vs D_low. Deteksi lemah yang
        # tak tercocokkan dibuang -- dianggap latar.
        _, t_re_remain = self._match(np.flatnonzero(~high), box, z, R, conf, t_remain)
        # baris 22: track tanpa pasangan menjadi hilang; dihapus setelah 30 frame
        for t in t_re_remain:
            t.lost += 1
        self.track = [t for t in self.track if t.lost <= config.TRACK_MAX_LOST]
        # baris 23-25: D_high sisa melahirkan track baru
        for i in d_remain:
            self._last_id += 1
            self.track.append(Track(self._last_id, z[i], R[i], box[i], conf[i], u_ego))
        # keluaran: T tanpa T_lost
        out = [t.relative(u_ego) for t in self.track if t.lost == 0]
        return np.array(out) if out else np.empty((0, 4))

    def visible(self):
        """[(kotak, state, id, hilang, conf), ...] untuk track yang dilaporkan. Hanya
        untuk visualisasi -- kendali cukup memakai nilai kembalian `update`."""
        return [(t.box, t.relative(self._u_ego), t.id, t.lost, t.conf)
                for t in self.track if t.lost == 0]

    @staticmethod
    def _match(idx, box, z, R, conf, candidates):
        """Cocokkan deteksi `idx` ke `candidates` lewat IoU + Hungarian.

        Kembali (indeks deteksi tak tercocokkan, track tak tercocokkan)."""
        if len(idx) == 0 or not candidates:
            return list(idx), list(candidates)
        iou = iou_matrix(box[idx], np.array([t.box for t in candidates]))
        used_d, used_t = set(), set()
        for i, j in hungarian(iou):
            candidates[j].correction(z[idx[i]], R[idx[i]], box[idx[i]], conf[idx[i]])
            used_d.add(i)
            used_t.add(j)
        return ([idx[i] for i in range(len(idx)) if i not in used_d],
                [t for j, t in enumerate(candidates) if j not in used_t])
