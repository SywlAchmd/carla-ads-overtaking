"""Overlay video untuk run loop tertutup berbasis vision (bagian 11.5).

Menggambar apa yang DILIHAT dan apa yang DIPUTUSKAN di atas citra kamera yang
sama yang dipakai kendali -- bukan kamera penonton terpisah. Isinya:

  * kotak deteksi YOLOPX, berikut jarak dan kecepatan hasil Kalman filter;
  * seluruh kandidat lintasan planner yang lolos (abu-abu) dan yang sedang
    dieksekusi (hijau), diproyeksikan ke citra;
  * state FSM, laju ego, jumlah kandidat, offset terpilih.

Lintasan planner hidup di frame jalan; proyeksinya frame jalan -> right-handed
-> CARLA -> kamera. Ketinggiannya diambil dari z reference path, bukan dari z
ego, supaya garisnya menempel di aspal saat ego mengangguk.
"""
import glob
import os
import shutil
import tempfile

import cv2
import numpy as np

import config
import perception

PUTIH, KUNING, HIJAU, ABU, GELAP = ((255, 255, 255), (0, 255, 255), (0, 230, 0),
                                    (150, 150, 150), (0, 0, 0))


def _teks(img, baris, pojok, skala=0.5, warna=PUTIH):
    """Tulis beberapa baris dengan latar gelap supaya terbaca di aspal maupun langit."""
    x, y = pojok
    for i, s in enumerate(baris):
        (w, h), _ = cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, skala, 1)
        yy = y + i * (h + 8)
        cv2.rectangle(img, (x - 3, yy - h - 4), (x + w + 3, yy + 4), GELAP, -1)
        cv2.putText(img, s, (x, yy), cv2.FONT_HERSHEY_SIMPLEX, skala, warna, 1,
                    cv2.LINE_AA)


class Perekam:
    """Kumpulkan frame beranotasi selama run, encode sekali di akhir."""

    def __init__(self, path_frame, ref5):
        self.pf, self.ref5 = path_frame, ref5
        self.dir = tempfile.mkdtemp(prefix='rekam_')
        self.n = 0

    def _piksel(self, w2c, xs, ys):
        """Titik frame jalan -> piksel citra. NaN untuk yang di belakang kamera."""
        rx, ry = self.pf.ke_rh(xs, ys)
        z = np.interp(xs, self.ref5[0], self.ref5[4]) + 0.10
        dunia = np.column_stack([rx, -ry, z, np.ones(len(z))])       # RH -> CARLA
        p = dunia @ np.asarray(w2c).T
        kam = np.column_stack([p[:, 1], -p[:, 2], p[:, 0]])          # UE -> kamera baku
        uv = np.full((len(kam), 2), np.nan)
        d = kam[:, 2] > 0.5
        uv[d, 0] = perception.F_PIKSEL * kam[d, 0] / kam[d, 2] + config.KAMERA_LEBAR / 2
        uv[d, 1] = perception.F_PIKSEL * kam[d, 1] / kam[d, 2] + config.KAMERA_TINGGI / 2
        return uv

    def _garis(self, img, w2c, traj, warna, tebal):
        uv = self._piksel(w2c, traj.states[0], traj.states[1])
        titik = uv[~np.isnan(uv[:, 0])].astype(np.int32)
        if len(titik) > 1:
            cv2.polylines(img, [titik], False, warna, tebal, cv2.LINE_AA)

    def tambah(self, rgb, w2c, terlihat, layak, terpilih, hud, v_ego):
        img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        for _, _, _, traj in (layak or []):                 # kandidat yang lolos
            self._garis(img, w2c, traj, ABU, 1)
        if terpilih is not None:
            self._garis(img, w2c, terpilih, HIJAU, 3)

        for kotak, x, id_, hilang, conf in terlihat:
            x1, y1, x2, y2 = [int(v) for v in kotak]
            warna = KUNING if hilang == 0 else ABU
            cv2.rectangle(img, (x1, y1), (x2, y2), warna, 2)
            jarak = float(np.hypot(x[0], x[1]))
            absolute = (v_ego + float(x[2])) * 3.6          # laju kendaraan itu sendiri
            label = [f'vehicle {conf:.2f}   #{id_}',
                     f'jarak    {jarak:5.1f} m',
                     f'rel {float(x[2]):+.1f} m/s  absolute {absolute:.0f} km/j']
            if hilang:
                label.append(f'melayang {hilang}')
            _teks(img, label, (x1, max(y1 - 48, 52)), 0.5, warna)

        _teks(img, hud, (14, 30), 0.6)
        _teks(img, ['kandidat planner', 'dieksekusi'], (14, config.KAMERA_TINGGI - 46), 0.45)
        cv2.line(img, (170, config.KAMERA_TINGGI - 54), (215, config.KAMERA_TINGGI - 54), ABU, 2)
        cv2.line(img, (170, config.KAMERA_TINGGI - 26), (215, config.KAMERA_TINGGI - 26), HIJAU, 3)

        cv2.imwrite(f'{self.dir}/{self.n:05d}.png', img)
        self.n += 1

    def simpan(self, nama, dir_=None):
        """Encode lewat cv2.VideoWriter, bukan ffmpeg: OpenCV sudah jadi
        dependensi perception, sedangkan ffmpeg dependensi sistem yang belum
        tentu ada (dan memang tidak ada di mesin ini)."""
        dir_ = dir_ or self.dir
        berkas = sorted(glob.glob(f'{dir_}/*.png'))
        if not berkas:
            raise RuntimeError(f'tidak ada frame di {dir_}')
        out = os.path.join(config.OUT_DIR, nama)
        h, w = cv2.imread(berkas[0]).shape[:2]
        tulis = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'),
                                1.0 / config.FIXED_DELTA_SECONDS, (w, h))
        if not tulis.isOpened():
            raise RuntimeError(f'VideoWriter gagal membuka {out}')
        for f in berkas:
            tulis.write(cv2.imread(f))
        tulis.release()
        shutil.rmtree(dir_, ignore_errors=True)
        print(f'{len(berkas)} frame -> {out}  ({os.path.getsize(out) / 1e6:.1f} MB, '
              f'{len(berkas) * config.FIXED_DELTA_SECONDS:.0f} detik)')
        return out
