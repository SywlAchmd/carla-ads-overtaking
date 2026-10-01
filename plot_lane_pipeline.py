"""Tiga tahap `lanes.py` divisualkan, dua model anotasi berdampingan (bagian 30).

Satu frame kamera yang sama diberikan ke dua checkpoint YOLOPX -- satu dilatih
dengan anotasi lajur per-MARKA, satu dengan anotasi MENERUS -- lalu tiap tahap
digambar apa adanya:

  1. IPM        piksel masker -> titik di bidang jalan, meter
  2. Kemiringan satu arah dipakai BERSAMA semua garis; dicari lewat ketajaman
                histogram, lalu dihaluskan kuadrat terkecil
  3. Kisi       marka berjarak sama, jadi yang dicari satu jarak dan satu fase

Gambar ini menjelaskan KENAPA angka sapuan 320 frame keluar seperti itu; ia
bukan penggantinya. Kesimpulan tidak boleh diambil dari satu frame.

    python plot_lane_pipeline.py
    python plot_lane_pipeline.py --frame out/lanes_camera.png
"""
import argparse
import math
import os

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import config
import lanes

INK, ACCENT, MUTED, SECONDARY = '#22303f', '#b5462f', '#94a3ad', '#2f6f9f'
MODEL = (('marking', 'per-marking labels'), ('continuous', 'continuous labels'))


def stages(ll, shape):
    """Kembalikan besaran antara tiap tahap, bukan hanya hasil akhirnya."""
    points = lanes.lane_points(ll, shape)
    edges = np.arange(-12.0, 12.0 + lanes.BIN_C, lanes.BIN_C)
    candidate = np.arange(-lanes.SEARCH_B, lanes.SEARCH_B + 0.005, 0.005)
    score = np.array([float((np.histogram(points[:, 1] - b * points[:, 0],
                                         bins=edges)[0].astype(float) ** 2).sum())
                     for b in candidate])
    b_coarse = float(candidate[int(np.argmax(score))])
    b = lanes._refine(points, b_coarse)
    c = points[:, 1] - b * points[:, 0]
    offset, weight = lanes._peaks(c)
    g = lanes.LaneGeometry(offset, weight, b, len(points))
    return dict(points=points, candidate=candidate, score=score, b_coarse=b_coarse, b=b,
                c=c, edges=edges, offset=offset, g=g)


def draw(rows, t, title):
    a, b_, c_ = rows
    g = t['g']

    # --- 1. IPM: piksel -> bidang jalan
    a.scatter(t['points'][:, 1], t['points'][:, 0], s=1.2, c=MUTED, linewidths=0,
              label='lane pixels after IPM')
    for x, y in g.lines():
        a.plot(y, x, c=ACCENT, lw=1.6, zorder=3)
    a.plot([], [], c=ACCENT, lw=1.6, label='fitted lattice lines')
    a.set_xlim(9, -9); a.set_ylim(lanes.RANGE)       # x terbalik: kiri = kiri ego
    a.set_xlabel('lateral y (m)'); a.set_ylabel('forward x (m)')
    a.set_title(f'1. IPM — {title}', fontsize=9, color=INK)

    # --- 2. Kemiringan bersama
    b_.plot(np.degrees(-np.arctan(t['candidate'])), t['score'] / t['score'].max(),
            c=INK, lw=1.2, label='histogram sharpness')
    b_.axvline(math.degrees(-math.atan(t['b_coarse'])), c=MUTED, ls=':', lw=1.4,
               label=f'argmax  {math.degrees(-math.atan(t["b_coarse"])):+.2f}°')
    b_.axvline(math.degrees(g.yaw), c=ACCENT, lw=1.6,
               label=f'least squares  {math.degrees(g.yaw):+.2f}°')
    b_.set_xlim(6, -6); b_.set_xlabel('heading vs road (deg)')
    b_.set_ylabel('sharpness (normalised)')
    b_.set_title('2. One slope shared by every line', fontsize=9, color=INK)

    # --- 3. Kisi
    counts, _ = np.histogram(t['c'], bins=t['edges'])
    center = (t['edges'][:-1] + t['edges'][1:]) / 2
    c_.fill_between(center, counts, step='mid', color=MUTED, alpha=.55,
                    label='histogram of c = y − b·x')
    c_.plot(t['offset'], np.zeros_like(t['offset']), 'o', ms=6, mfc='none',
            mec=ACCENT, mew=1.6, label=f'{len(t["offset"])} lines found')
    if g.lane_width:
        n = np.arange(-4, 5)
        c_.plot(g.phase + n * g.lane_width, np.full(9, counts.max() * .06), '|',
                ms=14, c=SECONDARY, mew=1.8,
                label=f'lattice, w = {g.lane_width:.3f} m')
        c_.text(.02, .97, f'residual {g.lattice_residual:.3f} m', transform=c_.transAxes,
                va='top', fontsize=9, color=ACCENT, weight='bold')
    c_.set_xlim(9, -9); c_.set_xlabel('lateral offset c (m)')
    c_.set_ylabel('pixel count')
    c_.set_title('3. Equally spaced lattice', fontsize=9, color=INK)

    for ax in rows:
        ax.legend(loc='upper right', frameon=False, fontsize=7, labelcolor=INK)
        ax.tick_params(colors=INK, labelsize=7); ax.grid(alpha=.15)
        for s in ax.spines.values():
            s.set_color(MUTED)
        ax.xaxis.label.set_color(INK); ax.yaxis.label.set_color(INK)


def main_():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frame', default=f'{config.OUT_DIR}/sensor_rgb.png')
    ap.add_argument('--model-dir', default=os.path.expanduser('~/sawal/model'))
    args = ap.parse_args()

    import yolopx
    rgb = cv2.cvtColor(cv2.imread(args.frame), cv2.COLOR_BGR2RGB)
    fig, ax = plt.subplots(2, 3, figsize=(14.5, 8.2))
    for i, (name, title) in enumerate(MODEL):
        net = yolopx.YOLOPX(f'{args.model_dir}/yolopx-{name}.pt')
        _, _, ll = net.infer(rgb)
        draw(ax[i], stages(ll, rgb.shape), title)
        del net
    fig.suptitle('From lane-line pixels to metres — the three stages of lanes.py',
                 fontsize=12, color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    out = f'{config.OUT_DIR}/lane_pipeline.png'
    fig.savefig(out, dpi=150)
    print(out)


if __name__ == '__main__':
    main_()
