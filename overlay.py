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
import lanes
import perception

WHITE, YELLOW, GREEN, GRAY, DARK = ((255, 255, 255), (0, 255, 255), (0, 230, 0),
                                    (150, 150, 150), (0, 0, 0))
ROAD, MARKING = (60, 200, 60), (40, 40, 235)      # area jalan, garis lajur (BGR)
LATTICE, AXIS = (255, 170, 40), (255, 90, 200)     # garis lajur tercocok, sumbu lajur ego


def _segmentation(img, mask, alpha=0.35):
    """Tumpangkan area jalan & garis lajur dari kepala segmentasi YOLOPX.

    Masker keluar pada ukuran masukan jaringan (384x640) setelah letterbox, jadi
    bingkainya dibuang dulu sebelum diregangkan balik -- kalau tidak, seluruh
    lapisannya bergeser 24 piksel ke atas terhadap citranya.

    Kembalikan masker area jalan seukuran citra, dipakai memotong garis lajur.
    """
    if mask is None:
        return None
    da, ll = mask
    h, w = np.shape(ll)
    r, pad_u, pad_v = lanes.letterbox_to_image((h, w), img.shape)
    u0, v0 = int(round(pad_u)), int(round(pad_v))
    u1, v1 = w - u0 if u0 else w, h - v0 if v0 else h
    layer, da_large = img.copy(), None
    for m, color in ((da, ROAD), (ll, MARKING)):
        inter = np.asarray(m)[v0:v1, u0:u1]
        large = cv2.resize(inter.astype(np.uint8), (img.shape[1], img.shape[0]),
                           interpolation=cv2.INTER_NEAREST)
        if color is ROAD:
            da_large = large > 0
        layer[large > 0] = color
    cv2.addWeighted(layer, alpha, img, 1.0 - alpha, 0.0, dst=img)
    return da_large


def _text(img, rows, corner, scale=0.5, color=WHITE):
    """Tulis beberapa baris dengan latar gelap supaya terbaca di aspal maupun langit."""
    x, y = corner
    for i, t in enumerate(rows):
        (w, h), _ = cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
        yy = y + i * (h + 8)
        cv2.rectangle(img, (x - 3, yy - h - 4), (x + w + 3, yy + 4), DARK, -1)
        cv2.putText(img, t, (x, yy), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1,
                    cv2.LINE_AA)


def _legend(img, entry, scale=0.45):
    """Panel legenda di KANAN ATAS. `entry` = [(label, warna, tebal), ...].

    Label dan contoh warnanya digambar dari SATU daftar pasangan. Versi sebelumnya
    menulis labelnya di satu tempat dan menggambar garisnya di loop terpisah, jadi
    menambah satu baris di salah satunya menggeser seluruh pasangan -- dan itu
    sempat terjadi.
    """
    if not entry:
        return
    measure = [cv2.getTextSize(e[0], cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0] for e in entry]
    text_width = max(w for w, _ in measure)
    height = max(h for _, h in measure)
    step, sample, pause, edges = height + 12, 46, 10, 14
    x_text = config.CAMERA_WIDTH - edges - text_width
    x_sample = x_text - pause - sample
    y0 = edges + height + 4
    cv2.rectangle(img, (x_sample - 8, y0 - height - 8),
                  (config.CAMERA_WIDTH - edges + 4, y0 + step * (len(entry) - 1) + 8),
                  DARK, -1)
    for i, ((name, color, thickness), (w, h)) in enumerate(zip(entry, measure)):
        yy = y0 + i * step
        cv2.line(img, (x_sample, yy - h // 2), (x_sample + sample, yy - h // 2),
                 color, thickness, cv2.LINE_AA)
        cv2.putText(img, name, (config.CAMERA_WIDTH - edges - w, yy),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, WHITE, 1, cv2.LINE_AA)


def _lane_lines(img, lane, da=None):
    """Gambar KISI hasil cocokan -- tersambung penuh, bukan penggal maskernya.

    Marka putus-putus tidak perlu disambung: semua penggal pada satu garis punya
    offset melintang yang sama, jadi mereka mengelompokkan diri sendiri di
    histogram. Yang digambar di sini hasil cocokannya, dan itulah yang dipakai
    kendali.

    `da` = masker area jalan pada ukuran citra. Garis dipotong ke sana: kisi itu
    lurus tak berhingga, dan tanpa potongan ini ia terlihat merayap naik ke
    tanggul dan dinding -- mengklaim lajur di tempat yang jelas bukan jalan.
    """
    if lane is None or lane.lane_width is None:
        return

    def draw(x, y, color, thickness, dashed=False):
        u, v = lanes.to_pixel(x, y)
        points = np.column_stack([u, v]).astype(np.int32)
        h, w = img.shape[:2]
        for i in range(len(points) - 1):
            if dashed and i % 2:
                continue
            a, b = points[i], points[i + 1]
            if not (0 <= a[0] < w and 0 <= a[1] < h and 0 <= b[0] < w and 0 <= b[1] < h):
                continue
            if da is not None and not (da[a[1], a[0]] or da[b[1], b[0]]):
                continue
            cv2.line(img, tuple(a), tuple(b), color, thickness, cv2.LINE_AA)

    for x, y in lane.lines():
        draw(x, y, LATTICE, 2)
    center = lane.center_line()
    if center is not None:
        draw(*center, AXIS, 2, dashed=True)


class Recorder:
    """Kumpulkan frame beranotasi selama run, encode sekali di akhir."""

    def __init__(self, path_frame, ref5):
        self.pf, self.ref5 = path_frame, ref5
        self.dir = tempfile.mkdtemp(prefix='record_')
        self.n = 0

    def _pixels(self, w2c, xs, ys):
        """Titik frame jalan -> piksel citra. NaN untuk yang di belakang kamera."""
        rx, ry = self.pf.to_rh(xs, ys)
        z = np.interp(xs, self.ref5[0], self.ref5[4]) + 0.10
        world = np.column_stack([rx, -ry, z, np.ones(len(z))])       # RH -> CARLA
        p = world @ np.asarray(w2c).T
        cam = np.column_stack([p[:, 1], -p[:, 2], p[:, 0]])          # UE -> kamera baku
        uv = np.full((len(cam), 2), np.nan)
        d = cam[:, 2] > 0.5
        uv[d, 0] = perception.F_PIXEL * cam[d, 0] / cam[d, 2] + config.CAMERA_WIDTH / 2
        uv[d, 1] = perception.F_PIXEL * cam[d, 1] / cam[d, 2] + config.CAMERA_HEIGHT / 2
        return uv

    def _polyline(self, img, w2c, traj, color, thickness):
        uv = self._pixels(w2c, traj.states[0], traj.states[1])
        points = uv[~np.isnan(uv[:, 0])].astype(np.int32)
        if len(points) > 1:
            cv2.polylines(img, [points], False, color, thickness, cv2.LINE_AA)

    def extra(self, rgb, w2c, visible, feasible, chosen, hud, v_ego, mask=None,
               lane=None):
        img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        da_mask = _segmentation(img, mask)                 # paling bawah
        _lane_lines(img, lane, da_mask)
        for _, _, _, traj in (feasible or []):                 # kandidat yang lolos
            self._polyline(img, w2c, traj, GRAY, 1)
        if chosen is not None:
            self._polyline(img, w2c, chosen, GREEN, 3)

        for box, x, id_, lost, conf in visible:
            x1, y1, x2, y2 = [int(v) for v in box]
            color = YELLOW if lost == 0 else GRAY
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            dist = float(np.hypot(x[0], x[1]))
            absolute = (v_ego + float(x[2])) * 3.6          # laju kendaraan itu sendiri
            label = [f'vehicle {conf:.2f}   #{id_}',
                     f'dist     {dist:5.1f} m',
                     f'rel {float(x[2]):+.1f} m/s  absolute {absolute:.0f} km/h']
            if lost:
                label.append(f'coasting {lost}')
            # Jepit ke dalam citra: kotak yang terpotong tepi kiri -- persis saat
            # berdampingan -- membuat labelnya keluar layar seluruhnya.
            _text(img, label, (max(x1, 14), max(y1 - 48, 52)), 0.5, color)

        if lane is not None and lane.lane_width is not None:
            hud = list(hud) + [f'lane width  {lane.lane_width:.2f} m  '
                               f'({len(lane.offset)} markings)',
                               f'offset      {lane.lane_dev:+.2f} m',
                               f'lattice res {lane.lattice_residual:.3f} m']
        _text(img, hud, (14, 30), 0.6)
        _legend(img, [('road area', ROAD, 6),
                       ('detected markings', MARKING, 6),
                       ('fitted lane lines', LATTICE, 2),
                       ('ego lane axis', AXIS, 2),
                       ('planner candidates', GRAY, 2),
                       ('executed', GREEN, 3)])

        cv2.imwrite(f'{self.dir}/{self.n:05d}.png', img)
        self.n += 1

    def save(self, name, dir_=None):
        """Encode lewat cv2.VideoWriter, bukan ffmpeg: OpenCV sudah jadi
        dependensi perception, sedangkan ffmpeg dependensi sistem yang belum
        tentu ada (dan memang tidak ada di mesin ini)."""
        dir_ = dir_ or self.dir
        filename = sorted(glob.glob(f'{dir_}/*.png'))
        if not filename:
            raise RuntimeError(f'no frames in {dir_}')
        out = os.path.join(config.OUT_DIR, name)
        h, w = cv2.imread(filename[0]).shape[:2]
        write = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'),
                                1.0 / config.FIXED_DELTA_SECONDS, (w, h))
        if not write.isOpened():
            raise RuntimeError(f'VideoWriter failed to open {out}')
        for f in filename:
            write.write(cv2.imread(f))
        write.release()
        shutil.rmtree(dir_, ignore_errors=True)
        print(f'{len(filename)} frame -> {out}  ({os.path.getsize(out) / 1e6:.1f} MB, '
              f'{len(filename) * config.FIXED_DELTA_SECONDS:.0f} s)')
        return out
