"""Pelacakan antar frame dan penapisan Kalman (rencana kerja bagian 10.3).

Murni numerik: tidak mengimpor carla, torch, maupun cv2 -- bisa diuji di terminal
tanpa menyalakan simulator (aturan 2.4, ditegakkan `tests/test_arsitektur.py`).

**Kenapa perlu.** Satu frame kamera hanya memberi POSISI. Kecepatan halangan --
yang dipakai FSM untuk TTC dan MPC untuk memprediksi elips sepanjang horizon --
hanya bisa datang dari beda antar frame. Beda mentah dua frame berturut pada
derau depth 0,5 m (bagian 18.4) di 10 Hz memberi derau kecepatan 5 m/s, sepuluh
kali lipat `DV_TRIGGER`. Kalman filter constant-velocity yang meredamnya.

**Asosiasi dua tahap, strategi ByteTrack.** Deteksi berkeyakinan tinggi
dicocokkan lebih dulu; deteksi berkeyakinan rendah TIDAK melahirkan track baru,
ia hanya menahan track yang sudah ada. Justifikasinya ada di data sendiri:
bagian 18.2 mengukur keyakinan meluruh terhadap jarak dan model fine-tuned mulai
kehilangan kendaraan di 40-50 m (0,62 pada 50 m). Tanpa tahap kedua, track di
tepi jangkauan hilang-timbul tiap beberapa frame dan estimasi kecepatannya ikut
rusak tepat ketika FSM mulai menghitung TTC.

**State di frame ego, relatif terhadap ego** -- sama dengan kontrak
`perception.py`. Constant-velocity berlaku untuk gerak MUTLAK kendaraan lain,
bukan gerak relatifnya, jadi langkah prediksi wajib mengompensasi gerak ego:

    p' = Rot(-w dt) (p + v dt)
    v' = Rot(-w dt) (v + [u, 0]) - [u + a dt, 0]

dengan p, v = posisi dan kecepatan relatif, u = laju ego, a = percepatan ego,
w = yaw rate ego. Suku translasi ego **saling meniadakan**: objek dan ego
bergeser di frame yang sama, jadi p + (v + v_ego) dt - v_ego dt = p + v dt. Yang
tersisa cuma rotasi ego dan perubahan laju ego. Karena itu modul ini tidak perlu
tahu posisi ego sama sekali -- cukup kinematikanya sendiri, persis yang dimiliki
kendaraan tanpa peta (lihat kontrak di docstring `perception.py`).

Penugasan memakai algoritma serakah menurut biaya menaik, bukan Hungarian:
skenario bagian 11.1 paling banyak memuat 2 kendaraan lain dan keduanya terpisah
jauh di citra, jadi keduanya memberi hasil sama tanpa menambah dependensi.
"""
import math

import numpy as np

import config


def iou_matriks(a, b):
    """IoU tiap pasangan kotak [x1,y1,x2,y2]. a (n,4), b (m,4) -> (n,m)."""
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    if len(a) == 0 or len(b) == 0:
        return np.empty((len(a), len(b)))
    x1 = np.maximum(a[:, None, 0], b[None, :, 0])
    y1 = np.maximum(a[:, None, 1], b[None, :, 1])
    x2 = np.minimum(a[:, None, 2], b[None, :, 2])
    y2 = np.minimum(a[:, None, 3], b[None, :, 3])
    potong = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    luas_a = (a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1])
    luas_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    gabung = luas_a[:, None] + luas_b[None, :] - potong
    return np.where(gabung > 0, potong / np.where(gabung > 0, gabung, 1.0), 0.0)


def tugaskan(biaya, batas):
    """Penugasan serakah menurut biaya menaik -> [(baris, kolom), ...].

    Pasangan dengan biaya di atas `batas` tidak pernah dibentuk: lebih baik satu
    track kehilangan pembaruan (ia masih melayang) daripada dikoreksi oleh
    deteksi milik kendaraan lain.
    """
    biaya = np.asarray(biaya, dtype=float)
    if biaya.size == 0:
        return []
    pasang, baris, kolom = [], set(), set()
    for i, j in zip(*np.unravel_index(np.argsort(biaya, axis=None), biaya.shape)):
        if biaya[i, j] > batas:
            break
        if i in baris or j in kolom:
            continue
        pasang.append((int(i), int(j)))
        baris.add(i)
        kolom.add(j)
    return pasang


class Track:
    """Satu kendaraan yang sedang dilacak. State [x, y, vx, vy] frame ego."""

    def __init__(self, id_, z, R, kotak, conf=0.0):
        self.id, self.conf = id_, float(conf)
        self.x = np.array([z[0], z[1], 0.0, 0.0], dtype=float)
        # Kecepatan awal tidak terukur sama sekali dari satu frame: ragunya
        # dipasang selebar rentang yang mungkin, supaya koreksi frame kedua
        # hampir seluruhnya menentukan kecepatan dan tidak tertahan prior.
        self.P = np.diag([R[0, 0], R[1, 1],
                          config.TRACK_SIGMA_V0 ** 2, config.TRACK_SIGMA_V0 ** 2])
        self.kotak = np.asarray(kotak, dtype=float)
        self.hit, self.hilang, self.tampil = 1, 0, False

    def prediksi(self, dt, u_ego, a_ego, w_ego):
        c, s = math.cos(-w_ego * dt), math.sin(-w_ego * dt)
        rot = np.array([[c, -s], [s, c]])
        F = np.block([[rot, rot * dt], [np.zeros((2, 2)), rot]])
        geser = rot @ np.array([u_ego, 0.0]) - np.array([u_ego + a_ego * dt, 0.0])
        self.x = F @ self.x + np.concatenate([np.zeros(2), geser])
        # derau proses = percepatan putih diskret; kendaraan lain boleh berubah
        # kecepatan sebesar TRACK_SIGMA_A tanpa membuat track kehilangan jejak
        i2 = np.eye(2)
        Q = config.TRACK_SIGMA_A ** 2 * np.block(
            [[dt ** 4 / 4 * i2, dt ** 3 / 2 * i2], [dt ** 3 / 2 * i2, dt ** 2 * i2]])
        self.P = F @ self.P @ F.T + Q

    def koreksi(self, z, R, kotak, conf=None):
        if conf is not None:
            self.conf = float(conf)
        H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (np.asarray(z, dtype=float) - H @ self.x)
        self.P = (np.eye(4) - K @ H) @ self.P
        self.kotak = np.asarray(kotak, dtype=float)
        self.hit += 1
        self.hilang = 0


class Pelacak:
    """Pelacak multi-objek. Satu instance hidup selama satu run."""

    def __init__(self):
        self.track, self._id_terakhir = [], 0

    def update(self, kotak, conf, z, R, dt, u_ego=0.0, a_ego=0.0, w_ego=0.0):
        """Satu frame. Kembali (M, 4) = [x, y, vx, vy] frame ego, relatif ego.

        `kotak` (n,4) piksel, `conf` (n,), `z` (n,2) posisi terukur frame ego,
        `R` (n,2,2) kovarians ukur tiap deteksi (bergantung jarak -- dibangun di
        `perception.py`, yang tahu soal sensornya).
        """
        kotak = np.asarray(kotak, dtype=float).reshape(-1, 4)
        conf = np.asarray(conf, dtype=float).reshape(-1)
        z = np.asarray(z, dtype=float).reshape(-1, 2)
        R = np.asarray(R, dtype=float).reshape(-1, 2, 2)

        for t in self.track:
            t.prediksi(dt, u_ego, a_ego, w_ego)
            t.hilang += 1                     # dibatalkan koreksi() bila nanti cocok

        tinggi = conf >= config.TRACK_CONF_TINGGI
        sisa = self._cocokkan(np.flatnonzero(tinggi), kotak, z, R, self.track, conf)
        # Tahap dua: deteksi lemah hanya menahan track yang belum tercocokkan.
        # Sengaja TIDAK melahirkan track baru -- di ambang 0,3 model fine-tuned
        # memang nol positif palsu (bagian 18.3), tapi menjadikan deteksi lemah
        # sebagai sumber halangan baru berarti mempertaruhkan rem mendadak MPC
        # pada bukti paling tipis yang dimiliki sistem.
        self._cocokkan(np.flatnonzero(~tinggi), kotak, z, R,
                       [t for t in self.track if t.hilang > 0], conf)
        for i in sisa:
            self._id_terakhir += 1
            self.track.append(Track(self._id_terakhir, z[i], R[i], kotak[i], conf[i]))

        self.track = [t for t in self.track if t.hilang <= config.TRACK_MAX_HILANG]
        for t in self.track:
            t.tampil = t.tampil or t.hit >= config.TRACK_N_INIT
        # Track yang sedang melayang IKUT dilaporkan. Itu justru gunanya: halangan
        # yang sekejap tidak terdeteksi tidak boleh lenyap dari pandangan FSM.
        keluar = [t.x for t in self.track if t.tampil]
        return np.array(keluar) if keluar else np.empty((0, 4))

    def terlihat(self):
        """[(kotak, state, id, hilang, conf), ...] untuk track yang dilaporkan. Hanya
        untuk visualisasi -- kendali cukup memakai nilai kembalian `update`."""
        return [(t.kotak, t.x, t.id, t.hilang, t.conf) for t in self.track if t.tampil]

    def _cocokkan(self, idx, kotak, z, R, kandidat, conf=None):
        """Cocokkan deteksi `idx` ke `kandidat`; kembali indeks yang tak tercocokkan."""
        if len(idx) == 0 or not kandidat:
            return list(idx)
        biaya = 1.0 - iou_matriks(kotak[idx], np.array([t.kotak for t in kandidat]))
        terpakai = set()
        for i, j in tugaskan(biaya, 1.0 - config.TRACK_IOU_MIN):
            kandidat[j].koreksi(z[idx[i]], R[idx[i]], kotak[idx[i]],
                                None if conf is None else conf[idx[i]])
            terpakai.add(i)
        return [idx[i] for i in range(len(idx)) if i not in terpakai]
