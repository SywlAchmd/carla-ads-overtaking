"""Kemiringan bersama dan pencocokan kisi, digambar di atas pandangan atas (bagian 30).

Dua gagasan yang membuat `lanes.py` bekerja, keduanya punya arti geometris yang
bisa dilihat langsung -- bukan sekadar rumus:

KEMIRINGAN BERSAMA.  `c = y - b*x` adalah **pergeseran miring** (shear) pada
bidang jalan. Di jalan lurus semua marka sejajar, jadi ADA satu `b` yang
menegakkan semuanya sekaligus. Setelah digeser dengan `b` yang benar, tiap garis
lajur jatuh pada satu nilai `c`, dan meruntuhkan gambar ke sumbu mendatar
memberi histogram yang memuncak tajam. Dengan `b` yang salah, garisnya tetap
miring dan puncaknya terseret melebar.

PENCOCOKAN KISI.  Puncak-puncak itu tidak sembarang letaknya: marka lajur
berjarak SAMA. Jadi yang dicari cukup satu jarak dan satu fase. Marka yang
terlewat menyisakan lubang di kisi, bukan celah dua kali lebar lajur -- dan
itulah yang membuat median jarak antar tetangga gagal sementara kisi bertahan.

Kolom kiri memakai kemiringan yang SENGAJA salah, kolom kanan yang dicari
algoritma. Frame bawaannya ruas lurus, yang jawabannya sudah diverifikasi
terpisah (sisa kisi 0,0024 m), supaya yang dibandingkan memang mekanismenya.

    python plot_lane_fit.py
"""
import argparse
import math

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import config
import lanes
import plot_ipm

GARIS, AKSEN, REDUP, KEDUA = '#22303f', '#b5462f', '#94a3ad', '#2f6f9f'
HIJAU = (60, 200, 60)


def geser(bev, b):
    """Geser miring kanvas BEV dengan kemiringan `b`, yaitu c = y - b*x."""
    h, w = bev.shape[:2]
    # baris i = x turun dari BEV_X[1]; kolom j = y turun dari BEV_Y[1]
    x_baris = plot_ipm.BEV_X[1] - np.arange(h) / plot_ipm.BEV_PX
    geser_px = (-b * x_baris) * plot_ipm.BEV_PX          # c = y - b*x -> kolom digeser
    keluar = np.zeros_like(bev)
    for i in range(h):
        M = np.float32([[1, 0, geser_px[i]], [0, 1, 0]])
        keluar[i:i + 1] = cv2.warpAffine(bev[i:i + 1], M, (w, 1))
    return keluar


def kolom(fig, gs, kol, bev, titik, b, judul, g=None):
    ax_img = fig.add_subplot(gs[0, kol])
    ax_his = fig.add_subplot(gs[1, kol])

    img = geser(bev, b).copy()
    c = titik[:, 1] - b * titik[:, 0]
    px = (plot_ipm.BEV_Y[1] - c) * plot_ipm.BEV_PX
    py = (plot_ipm.BEV_X[1] - titik[:, 0]) * plot_ipm.BEV_PX
    for a, d in zip(px.astype(int), py.astype(int)):
        if 0 <= d < img.shape[0] and 0 <= a < img.shape[1]:
            cv2.circle(img, (a, d), 2, HIJAU, -1)
    ax_img.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB),
                  extent=(plot_ipm.BEV_Y[1], plot_ipm.BEV_Y[0],
                          plot_ipm.BEV_X[0], plot_ipm.BEV_X[1]), aspect='auto')
    ax_img.set_title(judul, fontsize=10, color=GARIS)
    ax_img.set_ylabel('forward x (m)')

    tepi = np.arange(-12.0, 12.0 + lanes.BIN_C, lanes.BIN_C)
    cacah, _ = np.histogram(c, bins=tepi)
    tengah = (tepi[:-1] + tepi[1:]) / 2
    tajam = float((cacah.astype(float) ** 2).sum())
    ax_his.fill_between(tengah, cacah, step='mid', color=REDUP, alpha=.6)
    ax_his.set_xlim(plot_ipm.BEV_Y[1], plot_ipm.BEV_Y[0])
    ax_his.set_xlabel('lateral offset c = y - b·x   (m)')
    ax_his.set_ylabel('pixel count')
    ax_his.text(.02, .95, f'sharpness  {tajam / 1e5:.2f} x 10^5',
                transform=ax_his.transAxes, va='top', fontsize=9,
                color=AKSEN, weight='bold')

    if g is not None and g.lebar_lajur:
        n = np.arange(-4, 5)
        ax_his.plot(g.fase + n * g.lebar_lajur,
                    np.full(9, cacah.max() * .08), '|', ms=15, c=KEDUA, mew=2,
                    label=f'lattice  w = {g.lebar_lajur:.3f} m')
        ax_his.plot(g.offset, np.zeros_like(g.offset), 'o', ms=6, mfc='none',
                    mec=AKSEN, mew=1.7, label=f'{len(g.offset)} peaks -> lane lines')
        ax_his.legend(loc='upper right', frameon=False, fontsize=8, labelcolor=GARIS)
        ax_his.text(.02, .80, f'lattice residual  {g.sisa_kisi:.4f} m',
                    transform=ax_his.transAxes, va='top', fontsize=9,
                    color=AKSEN, weight='bold')
    return ax_img, ax_his


def main_():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frame', default=f'{config.OUT_DIR}/sensor_rgb.png')
    ap.add_argument('--b-salah', type=float, default=-0.10,
                    help='kemiringan yang SENGAJA salah untuk pembanding')
    ap.add_argument('--weight', default=None)
    args = ap.parse_args()

    import yolopx
    bgr = cv2.imread(args.frame)
    net = yolopx.YOLOPX(args.weight)
    _, _, ll = net.infer(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    bev = plot_ipm.luruskan(bgr)
    titik = lanes.titik_lajur(ll, bgr.shape)
    g = lanes.dari_masker(ll, bgr.shape)

    fig = plt.figure(figsize=(13.6, 8.2))
    gs = fig.add_gridspec(2, 2, height_ratios=(1.45, 1.0), hspace=.26, wspace=.18)
    ax = []
    ax += kolom(fig, gs, 0, bev, titik, args.b_salah,
                f'Wrong shear  (b = {args.b_salah:+.4f})  —  lines left slanted')
    ax += kolom(fig, gs, 1, bev, titik, g.kemiringan,
                f'Sheared by the common slope  '
                f'(b = {g.kemiringan:+.4f},  {math.degrees(g.yaw):+.2f}°)', g)
    for a in ax:
        a.tick_params(colors=GARIS, labelsize=8)
        a.xaxis.label.set_color(GARIS); a.yaxis.label.set_color(GARIS)
        for s in a.spines.values():
            s.set_color(REDUP)
    ax[1].grid(alpha=.15); ax[3].grid(alpha=.15)
    fig.suptitle('c = y − b·x is a shear: find the one slope that stands every lane '
                 'line upright, then fit an equally spaced lattice to the peaks',
                 fontsize=12, color=GARIS)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    keluar = f'{config.OUT_DIR}/lane_fit_explained.png'
    fig.savefig(keluar, dpi=150)
    print(keluar)


if __name__ == '__main__':
    main_()
