"""IPM dijelaskan di atas citra kamera CARLA yang sebenarnya (bagian 30).

Inverse perspective mapping hanya butuh tiga hal, dan ketiganya diketahui dari
pemasangan kamera -- bukan dari simulator:

    tinggi kamera z, panjang fokus f, dan asumsi permukaan jalan DATAR

Kamera menghadap lurus (pitch nol), jadi berkas yang keluar lewat baris `v`
menunjuk ke bawah sebesar (v - c_v)/f radian, dan memotong permukaan jalan pada

    x = f * z / (v - c_v)          y = -x * (u - c_u) / f

Baris tepat di titik hilang (v = c_v) memberi x tak hingga: berkasnya sejajar
jalan dan tidak pernah memotongnya. Itu sebabnya ketelitian runtuh mendekati
horizon, dan itulah yang digambar panel ketiga.

    python plot_ipm.py
"""
import argparse

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import config
import lanes

GARIS, AKSEN, REDUP, KEDUA = '#22303f', '#b5462f', '#94a3ad', '#2f6f9f'
X_GRID = (8, 12, 16, 22, 30, 45)                       # m di depan
Y_GRID = (-5.25, -1.75, 1.75, 5.25)                    # batas lajur, m
CONTOH = ((12.0, 1.75), (30.0, -1.75))                 # titik yang dianotasi


def _rentang(x0, x1, n=80):
    return np.linspace(x0, x1, n)


ZOOM = (160, 1130, 700, 346)        # xlim lalu ylim: dizoom, BUKAN dipotong --
                                    # koordinat piksel tetap apa adanya


def panel_citra(ax, bgr):
    ax.imshow(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    cv_ = config.KAMERA_TINGGI / 2.0

    for y in Y_GRID:                                   # garis sejajar jalan
        u, v = lanes.ke_piksel(_rentang(X_GRID[0], X_GRID[-1]), y)
        ax.plot(u, v, color=KEDUA, lw=1.3, alpha=.95)
    for x in X_GRID:                                   # garis melintang, jarak tetap
        u, v = lanes.ke_piksel(np.full(40, x), np.linspace(Y_GRID[0], Y_GRID[-1], 40))
        ax.plot(u, v, color='white', lw=2.6, alpha=.75)
        ax.plot(u, v, color=REDUP, lw=1.3)
        ax.text(u[0] - 14, v[0], f'{x} m', color=GARIS, fontsize=9, va='center',
                ha='right', weight='bold',
                bbox=dict(fc='white', ec='none', alpha=.8, pad=1.4))

    ax.axhline(cv_, color=AKSEN, lw=1.5, ls='--')
    ax.text(ZOOM[1] - 12, cv_ - 8, 'horizon:  v = c_v  ->  x = infinity', color=AKSEN,
            fontsize=9.5, va='bottom', ha='right', weight='bold',
            bbox=dict(fc='white', ec='none', alpha=.85, pad=2))

    for (x, y), (dx, dy) in zip(CONTOH, ((-120, -62), (96, -46))):
        u, v = lanes.ke_piksel(x, y)
        ax.plot([u], [v], 'o', ms=9, mfc='none', mec=AKSEN, mew=2.4)
        ax.annotate(f'pixel ({u:.0f}, {v:.0f})\n->  x = {x:.0f} m,  y = {y:+.2f} m',
                    (u, v), textcoords='offset points', xytext=(dx, dy),
                    fontsize=9, color=AKSEN, weight='bold',
                    bbox=dict(fc='white', ec=AKSEN, lw=.8, alpha=.9, pad=3),
                    arrowprops=dict(arrowstyle='-|>', color=AKSEN, lw=1.2))

    ax.plot([], [], color=KEDUA, lw=1.3, label='parallel to the road')
    ax.plot([], [], color=REDUP, lw=1.3, label='constant forward distance')
    ax.set_xlim(ZOOM[0], ZOOM[1]); ax.set_ylim(ZOOM[2], ZOOM[3])
    ax.set_xticks([]); ax.set_yticks([])
    ax.set_title('1. The metric grid as the camera sees it', fontsize=11, color=GARIS,
                 pad=6)


def panel_bev(ax):
    for y in Y_GRID:
        ax.plot([y, y], [X_GRID[0], X_GRID[-1]], color=KEDUA, lw=1.0)
    for x in X_GRID:
        ax.plot([Y_GRID[0], Y_GRID[-1]], [x, x], color=REDUP, lw=1.0)
        ax.text(Y_GRID[0] - 0.4, x, f'{x} m', color=GARIS, fontsize=8, va='center',
                ha='left', weight='bold')
    for x, y in CONTOH:
        ax.plot([y], [x], 'o', ms=7, mfc='none', mec=AKSEN, mew=2)
    ax.plot([0], [0], marker='s', ms=9, color=GARIS)
    ax.text(0, -1.6, 'ego', fontsize=8, color=GARIS, ha='center')
    ax.set_xlim(8.6, -6.4); ax.set_ylim(-3, X_GRID[-1] + 3)
    ax.set_xlabel('lateral y (m)'); ax.set_ylabel('forward x (m)')
    ax.set_title('2. The same grid after IPM — regular again', fontsize=10, color=GARIS)


def panel_resolusi(ax):
    cv_ = config.KAMERA_TINGGI / 2.0
    dv = np.arange(4, config.KAMERA_TINGGI / 2)
    x = lanes.F_PIKSEL * config.KAMERA_Z / dv
    meter_per_piksel = np.abs(np.gradient(x, dv))
    pakai = (x >= 5) & (x <= 90)
    ax.plot(x[pakai], meter_per_piksel[pakai], color=GARIS, lw=1.6,
            label='metres covered by one pixel row')
    for xx in (10, 20, 44, 80):
        i = int(np.argmin(np.abs(x - xx)))
        ax.plot([x[i]], [meter_per_piksel[i]], 'o', ms=5, color=AKSEN)
        ax.annotate(f'{x[i]:.0f} m -> {meter_per_piksel[i]:.2f} m/px',
                    (x[i], meter_per_piksel[i]), textcoords='offset points',
                    xytext=(8, 6), fontsize=8, color=AKSEN)
    ax.axvline(44.0, color=REDUP, ls=':', lw=1.2,
               label='first detection, 44 m (check_estimation)')
    ax.set_yscale('log'); ax.set_xlabel('forward distance x (m)')
    ax.set_ylabel('metres per pixel row (log)')
    ax.set_title('3. Why far range is hard: one pixel buys more metres',
                 fontsize=10, color=GARIS)


BEV_X = (6.0, 46.0)          # m di depan yang dipetakan
BEV_Y = (-11.0, 11.0)        # m melintang
BEV_PX = 22                  # piksel per meter


def luruskan(bgr):
    """Regangkan seluruh piksel jalan jadi pandangan dari atas (warp IPM penuh).

    Ini bentuk IPM yang lazim ditampilkan. `lanes.py` TIDAK melakukannya --
    ia membalik-proyeksikan piksel masker lajur saja, yang jumlahnya ribuan,
    bukan sejuta. Transformasinya identik; yang berbeda cuma berapa piksel yang
    dikenai. Warp penuh di sini murni untuk penjelasan.

    Dikerjakan sebagai pemetaan BALIK: untuk tiap sel keluaran (x, y) dalam meter,
    cari piksel sumbernya lewat `lanes.ke_piksel`, lalu ambil warnanya. Arah maju
    menghindari lubang di keluaran.
    """
    tinggi = int((BEV_X[1] - BEV_X[0]) * BEV_PX)
    lebar = int((BEV_Y[1] - BEV_Y[0]) * BEV_PX)
    x = np.linspace(BEV_X[1], BEV_X[0], tinggi)          # baris atas = paling jauh
    y = np.linspace(BEV_Y[1], BEV_Y[0], lebar)           # kolom kiri = y positif
    X, Y = np.meshgrid(x, y, indexing='ij')
    u, v = lanes.ke_piksel(X, Y)
    sah = (u >= 0) & (u < bgr.shape[1]) & (v >= 0) & (v < bgr.shape[0])
    keluar = cv2.remap(bgr, u.astype(np.float32), v.astype(np.float32),
                       cv2.INTER_LINEAR, borderValue=(0, 0, 0))
    keluar[~sah] = 0
    return keluar


def _ke_bev_px(x, y):
    """Titik frame ego (m) -> piksel pada kanvas BEV."""
    return ((BEV_Y[1] - np.asarray(y)) * BEV_PX,
            (BEV_X[1] - np.asarray(x)) * BEV_PX)


def gambar_birdseye(bgr, ll, keluar):
    """Tiga panel: citra asli, hasil pelurusan IPM, dan lajur yang tercocok."""
    bev = luruskan(bgr)
    g = lanes.dari_masker(ll, bgr.shape)
    titik = lanes.titik_lajur(ll, bgr.shape)

    bev2 = bev.copy()
    tx, ty = _ke_bev_px(titik[:, 0], titik[:, 1])
    for a, b in zip(tx.astype(int), ty.astype(int)):
        if 0 <= b < bev2.shape[0] and 0 <= a < bev2.shape[1]:
            cv2.circle(bev2, (a, b), 2, (60, 200, 60), -1)
    if g is not None and g.lebar_lajur:
        for gx, gy in g.garis(jangkauan=BEV_X, n=60):
            px, py = _ke_bev_px(gx, gy)
            p = np.column_stack([px, py]).astype(np.int32)
            for i in range(0, len(p) - 1, 2):            # putus-putus, gaya lazim
                cv2.line(bev2, tuple(p[i]), tuple(p[i + 1]), (40, 40, 235), 2,
                         cv2.LINE_AA)

    fig, ax = plt.subplots(1, 3, figsize=(13.5, 6.4),
                           gridspec_kw=dict(width_ratios=(1.55, 1.0, 1.0)))
    ax[0].imshow(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    ax[0].set_title('Original camera image', fontsize=10, color=GARIS)
    for a, img, judul in ((ax[1], bev, 'After IPM — bird\'s eye'),
                          (ax[2], bev2, 'Lane pixels + fitted lattice')):
        a.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB),
                 extent=(BEV_Y[1], BEV_Y[0], BEV_X[0], BEV_X[1]), aspect='equal')
        a.set_title(judul, fontsize=10, color=GARIS)
        a.set_xlabel('lateral y (m)')
    ax[1].set_ylabel('forward x (m)')
    ax[0].set_xticks([]); ax[0].set_yticks([])
    for a in ax[1:]:
        a.tick_params(colors=GARIS, labelsize=8)
        a.xaxis.label.set_color(GARIS); a.yaxis.label.set_color(GARIS)
    for a in ax:
        for sp in a.spines.values():
            sp.set_color(REDUP)
    fig.suptitle('The full IPM warp — every road pixel straightened onto the ground '
                 'plane', fontsize=12, color=GARIS)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(keluar, dpi=150)


def main_():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frame', default=f'{config.OUT_DIR}/sensor_rgb.png')
    ap.add_argument('--weight', default=None, help='checkpoint YOLOPX lain')
    args = ap.parse_args()
    bgr = cv2.imread(args.frame)

    fig = plt.figure(figsize=(13.2, 8.6))
    gs = fig.add_gridspec(2, 2, height_ratios=(1.0, 0.92), hspace=.26, wspace=.22)
    ax = [fig.add_subplot(gs[0, :]), fig.add_subplot(gs[1, 0]),
          fig.add_subplot(gs[1, 1])]
    panel_citra(ax[0], bgr)
    panel_bev(ax[1])
    panel_resolusi(ax[2])
    for a in ax:
        a.legend(loc='upper right', frameon=False, fontsize=8.5, labelcolor=GARIS)
        a.tick_params(colors=GARIS, labelsize=8)
        for sp in a.spines.values():
            sp.set_color(REDUP)
        a.xaxis.label.set_color(GARIS); a.yaxis.label.set_color(GARIS)
    ax[1].set_aspect('equal')
    ax[1].grid(alpha=.15); ax[2].grid(alpha=.15, which='both')
    fig.suptitle(f'Inverse perspective mapping — camera {config.KAMERA_Z:.2f} m above '
                 f'a flat road, fov {config.KAMERA_FOV:.0f} deg, f = {lanes.F_PIKSEL:.0f} px',
                 fontsize=12, color=GARIS)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    keluar = f'{config.OUT_DIR}/ipm_explained.png'
    fig.savefig(keluar, dpi=150)
    print(keluar)

    import yolopx
    net = yolopx.YOLOPX(args.weight)
    _, _, ll = net.infer(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    keluar2 = f'{config.OUT_DIR}/ipm_birdseye.png'
    gambar_birdseye(bgr, ll, keluar2)
    print(keluar2)


if __name__ == '__main__':
    main_()
