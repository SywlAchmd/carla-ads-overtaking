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

GARIS, AKSEN, REDUP, KEDUA = '#22303f', '#b5462f', '#94a3ad', '#2f6f9f'
MODEL = (('marking', 'per-marking labels'), ('continuous', 'continuous labels'))


def tahapan(ll, bentuk):
    """Kembalikan besaran antara tiap tahap, bukan hanya hasil akhirnya."""
    titik = lanes.titik_lajur(ll, bentuk)
    tepi = np.arange(-12.0, 12.0 + lanes.BIN_C, lanes.BIN_C)
    calon = np.arange(-lanes.CARI_B, lanes.CARI_B + 0.005, 0.005)
    skor = np.array([float((np.histogram(titik[:, 1] - b * titik[:, 0],
                                         bins=tepi)[0].astype(float) ** 2).sum())
                     for b in calon])
    b_kasar = float(calon[int(np.argmax(skor))])
    b = lanes._haluskan(titik, b_kasar)
    c = titik[:, 1] - b * titik[:, 0]
    offset, bobot = lanes._puncak(c)
    g = lanes.GeometriLajur(offset, bobot, b, len(titik))
    return dict(titik=titik, calon=calon, skor=skor, b_kasar=b_kasar, b=b,
                c=c, tepi=tepi, offset=offset, g=g)


def gambar(baris, t, judul):
    a, b_, c_ = baris
    g = t['g']

    # --- 1. IPM: piksel -> bidang jalan
    a.scatter(t['titik'][:, 1], t['titik'][:, 0], s=1.2, c=REDUP, linewidths=0,
              label='lane pixels after IPM')
    for x, y in g.garis():
        a.plot(y, x, c=AKSEN, lw=1.6, zorder=3)
    a.plot([], [], c=AKSEN, lw=1.6, label='fitted lattice lines')
    a.set_xlim(9, -9); a.set_ylim(lanes.JANGKAUAN)       # x terbalik: kiri = kiri ego
    a.set_xlabel('lateral y (m)'); a.set_ylabel('forward x (m)')
    a.set_title(f'1. IPM — {judul}', fontsize=9, color=GARIS)

    # --- 2. Kemiringan bersama
    b_.plot(np.degrees(-np.arctan(t['calon'])), t['skor'] / t['skor'].max(),
            c=GARIS, lw=1.2, label='histogram sharpness')
    b_.axvline(math.degrees(-math.atan(t['b_kasar'])), c=REDUP, ls=':', lw=1.4,
               label=f'argmax  {math.degrees(-math.atan(t["b_kasar"])):+.2f}°')
    b_.axvline(math.degrees(g.yaw), c=AKSEN, lw=1.6,
               label=f'least squares  {math.degrees(g.yaw):+.2f}°')
    b_.set_xlim(6, -6); b_.set_xlabel('heading vs road (deg)')
    b_.set_ylabel('sharpness (normalised)')
    b_.set_title('2. One slope shared by every line', fontsize=9, color=GARIS)

    # --- 3. Kisi
    cacah, _ = np.histogram(t['c'], bins=t['tepi'])
    tengah = (t['tepi'][:-1] + t['tepi'][1:]) / 2
    c_.fill_between(tengah, cacah, step='mid', color=REDUP, alpha=.55,
                    label='histogram of c = y − b·x')
    c_.plot(t['offset'], np.zeros_like(t['offset']), 'o', ms=6, mfc='none',
            mec=AKSEN, mew=1.6, label=f'{len(t["offset"])} lines found')
    if g.lebar_lajur:
        n = np.arange(-4, 5)
        c_.plot(g.fase + n * g.lebar_lajur, np.full(9, cacah.max() * .06), '|',
                ms=14, c=KEDUA, mew=1.8,
                label=f'lattice, w = {g.lebar_lajur:.3f} m')
        c_.text(.02, .97, f'residual {g.sisa_kisi:.3f} m', transform=c_.transAxes,
                va='top', fontsize=9, color=AKSEN, weight='bold')
    c_.set_xlim(9, -9); c_.set_xlabel('lateral offset c (m)')
    c_.set_ylabel('pixel count')
    c_.set_title('3. Equally spaced lattice', fontsize=9, color=GARIS)

    for ax in baris:
        ax.legend(loc='upper right', frameon=False, fontsize=7, labelcolor=GARIS)
        ax.tick_params(colors=GARIS, labelsize=7); ax.grid(alpha=.15)
        for s in ax.spines.values():
            s.set_color(REDUP)
        ax.xaxis.label.set_color(GARIS); ax.yaxis.label.set_color(GARIS)


def main_():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frame', default=f'{config.OUT_DIR}/sensor_rgb.png')
    ap.add_argument('--model-dir', default=os.path.expanduser('~/sawal/model'))
    args = ap.parse_args()

    import yolopx
    rgb = cv2.cvtColor(cv2.imread(args.frame), cv2.COLOR_BGR2RGB)
    fig, ax = plt.subplots(2, 3, figsize=(14.5, 8.2))
    for i, (nama, judul) in enumerate(MODEL):
        net = yolopx.YOLOPX(f'{args.model_dir}/yolopx-{nama}.pt')
        _, _, ll = net.infer(rgb)
        gambar(ax[i], tahapan(ll, rgb.shape), judul)
        del net
    fig.suptitle('From lane-line pixels to metres — the three stages of lanes.py',
                 fontsize=12, color=GARIS)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    keluar = f'{config.OUT_DIR}/lane_pipeline.png'
    fig.savefig(keluar, dpi=150)
    print(keluar)


if __name__ == '__main__':
    main_()
