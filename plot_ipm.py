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

INK, ACCENT, MUTED, SECONDARY = '#22303f', '#b5462f', '#94a3ad', '#2f6f9f'
X_GRID = (8, 12, 16, 22, 30, 45)                       # m di depan
Y_GRID = (-5.25, -1.75, 1.75, 5.25)                    # batas lajur, m
SAMPLE = ((12.0, 1.75), (30.0, -1.75))                 # titik yang dianotasi


def _span(x0, x1, n=80):
    return np.linspace(x0, x1, n)


ZOOM = (160, 1130, 700, 346)        # xlim lalu ylim: dizoom, BUKAN dipotong --
                                    # koordinat piksel tetap apa adanya


def panel_image(ax, bgr):
    ax.imshow(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    cv_ = config.CAMERA_HEIGHT / 2.0

    for y in Y_GRID:                                   # garis sejajar jalan
        u, v = lanes.to_pixel(_span(X_GRID[0], X_GRID[-1]), y)
        ax.plot(u, v, color=SECONDARY, lw=1.3, alpha=.95)
    for x in X_GRID:                                   # garis melintang, jarak tetap
        u, v = lanes.to_pixel(np.full(40, x), np.linspace(Y_GRID[0], Y_GRID[-1], 40))
        ax.plot(u, v, color='white', lw=2.6, alpha=.75)
        ax.plot(u, v, color=MUTED, lw=1.3)
        ax.text(u[0] - 14, v[0], f'{x} m', color=INK, fontsize=9, va='center',
                ha='right', weight='bold',
                bbox=dict(fc='white', ec='none', alpha=.8, pad=1.4))

    ax.axhline(cv_, color=ACCENT, lw=1.5, ls='--')
    ax.text(ZOOM[1] - 12, cv_ - 8, 'horizon:  v = c_v  ->  x = infinity', color=ACCENT,
            fontsize=9.5, va='bottom', ha='right', weight='bold',
            bbox=dict(fc='white', ec='none', alpha=.85, pad=2))

    for (x, y), (dx, dy) in zip(SAMPLE, ((-120, -62), (96, -46))):
        u, v = lanes.to_pixel(x, y)
        ax.plot([u], [v], 'o', ms=9, mfc='none', mec=ACCENT, mew=2.4)
        ax.annotate(f'pixel ({u:.0f}, {v:.0f})\n->  x = {x:.0f} m,  y = {y:+.2f} m',
                    (u, v), textcoords='offset points', xytext=(dx, dy),
                    fontsize=9, color=ACCENT, weight='bold',
                    bbox=dict(fc='white', ec=ACCENT, lw=.8, alpha=.9, pad=3),
                    arrowprops=dict(arrowstyle='-|>', color=ACCENT, lw=1.2))

    ax.plot([], [], color=SECONDARY, lw=1.3, label='parallel to the road')
    ax.plot([], [], color=MUTED, lw=1.3, label='constant forward distance')
    ax.set_xlim(ZOOM[0], ZOOM[1]); ax.set_ylim(ZOOM[2], ZOOM[3])
    ax.set_xticks([]); ax.set_yticks([])
    ax.set_title('1. The metric grid as the camera sees it', fontsize=11, color=INK,
                 pad=6)


def panel_bev(ax):
    for y in Y_GRID:
        ax.plot([y, y], [X_GRID[0], X_GRID[-1]], color=SECONDARY, lw=1.0)
    for x in X_GRID:
        ax.plot([Y_GRID[0], Y_GRID[-1]], [x, x], color=MUTED, lw=1.0)
        ax.text(Y_GRID[0] - 0.4, x, f'{x} m', color=INK, fontsize=8, va='center',
                ha='left', weight='bold')
    for x, y in SAMPLE:
        ax.plot([y], [x], 'o', ms=7, mfc='none', mec=ACCENT, mew=2)
    ax.plot([0], [0], marker='s', ms=9, color=INK)
    ax.text(0, -1.6, 'ego', fontsize=8, color=INK, ha='center')
    ax.set_xlim(8.6, -6.4); ax.set_ylim(-3, X_GRID[-1] + 3)
    ax.set_xlabel('lateral y (m)'); ax.set_ylabel('forward x (m)')
    ax.set_title('2. The same grid after IPM — regular again', fontsize=10, color=INK)


def panel_resolution(ax):
    cv_ = config.CAMERA_HEIGHT / 2.0
    dv = np.arange(4, config.CAMERA_HEIGHT / 2)
    x = lanes.F_PIXEL * config.CAMERA_Z / dv
    meters_per_pixel = np.abs(np.gradient(x, dv))
    use = (x >= 5) & (x <= 90)
    ax.plot(x[use], meters_per_pixel[use], color=INK, lw=1.6,
            label='metres covered by one pixel row')
    for xx in (10, 20, 44, 80):
        i = int(np.argmin(np.abs(x - xx)))
        ax.plot([x[i]], [meters_per_pixel[i]], 'o', ms=5, color=ACCENT)
        ax.annotate(f'{x[i]:.0f} m -> {meters_per_pixel[i]:.2f} m/px',
                    (x[i], meters_per_pixel[i]), textcoords='offset points',
                    xytext=(8, 6), fontsize=8, color=ACCENT)
    ax.axvline(44.0, color=MUTED, ls=':', lw=1.2,
               label='first detection, 44 m (check_estimation)')
    ax.set_yscale('log'); ax.set_xlabel('forward distance x (m)')
    ax.set_ylabel('metres per pixel row (log)')
    ax.set_title('3. Why far range is hard: one pixel buys more metres',
                 fontsize=10, color=INK)


BEV_X = (6.0, 46.0)          # m di depan yang dipetakan
BEV_Y = (-11.0, 11.0)        # m melintang
BEV_PX = 22                  # piksel per meter


def straighten(bgr):
    """Regangkan seluruh piksel jalan jadi pandangan dari atas (warp IPM penuh).

    Ini bentuk IPM yang lazim ditampilkan. `lanes.py` TIDAK melakukannya --
    ia membalik-proyeksikan piksel masker lajur saja, yang jumlahnya ribuan,
    bukan sejuta. Transformasinya identik; yang berbeda cuma berapa piksel yang
    dikenai. Warp penuh di sini murni untuk penjelasan.

    Dikerjakan sebagai pemetaan BALIK: untuk tiap sel keluaran (x, y) dalam meter,
    cari piksel sumbernya lewat `lanes.to_pixel`, lalu ambil warnanya. Arah maju
    menghindari lubang di keluaran.
    """
    height = int((BEV_X[1] - BEV_X[0]) * BEV_PX)
    width = int((BEV_Y[1] - BEV_Y[0]) * BEV_PX)
    x = np.linspace(BEV_X[1], BEV_X[0], height)          # baris atas = paling jauh
    y = np.linspace(BEV_Y[1], BEV_Y[0], width)           # kolom kiri = y positif
    X, Y = np.meshgrid(x, y, indexing='ij')
    u, v = lanes.to_pixel(X, Y)
    valid = (u >= 0) & (u < bgr.shape[1]) & (v >= 0) & (v < bgr.shape[0])
    out = cv2.remap(bgr, u.astype(np.float32), v.astype(np.float32),
                       cv2.INTER_LINEAR, borderValue=(0, 0, 0))
    out[~valid] = 0
    return out


def _to_bev_px(x, y):
    """Titik frame ego (m) -> piksel pada kanvas BEV."""
    return ((BEV_Y[1] - np.asarray(y)) * BEV_PX,
            (BEV_X[1] - np.asarray(x)) * BEV_PX)


def draw_birdseye(bgr, ll, out):
    """Tiga panel: citra asli, hasil pelurusan IPM, dan lajur yang tercocok."""
    bev = straighten(bgr)
    g = lanes.from_mask(ll, bgr.shape)
    points = lanes.lane_points(ll, bgr.shape)

    bev2 = bev.copy()
    tx, ty = _to_bev_px(points[:, 0], points[:, 1])
    for a, b in zip(tx.astype(int), ty.astype(int)):
        if 0 <= b < bev2.shape[0] and 0 <= a < bev2.shape[1]:
            cv2.circle(bev2, (a, b), 2, (60, 200, 60), -1)
    if g is not None and g.lane_width:
        for gx, gy in g.lines(reach=BEV_X, n=60):
            px, py = _to_bev_px(gx, gy)
            p = np.column_stack([px, py]).astype(np.int32)
            for i in range(0, len(p) - 1, 2):            # putus-putus, gaya lazim
                cv2.line(bev2, tuple(p[i]), tuple(p[i + 1]), (40, 40, 235), 2,
                         cv2.LINE_AA)

    fig, ax = plt.subplots(1, 3, figsize=(13.5, 6.4),
                           gridspec_kw=dict(width_ratios=(1.55, 1.0, 1.0)))
    ax[0].imshow(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    ax[0].set_title('Original camera image', fontsize=10, color=INK)
    for a, img, title in ((ax[1], bev, 'After IPM — bird\'s eye'),
                          (ax[2], bev2, 'Lane pixels + fitted lattice')):
        a.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB),
                 extent=(BEV_Y[1], BEV_Y[0], BEV_X[0], BEV_X[1]), aspect='equal')
        a.set_title(title, fontsize=10, color=INK)
        a.set_xlabel('lateral y (m)')
    ax[1].set_ylabel('forward x (m)')
    ax[0].set_xticks([]); ax[0].set_yticks([])
    for a in ax[1:]:
        a.tick_params(colors=INK, labelsize=8)
        a.xaxis.label.set_color(INK); a.yaxis.label.set_color(INK)
    for a in ax:
        for sp in a.spines.values():
            sp.set_color(MUTED)
    fig.suptitle('The full IPM warp — every road pixel straightened onto the ground '
                 'plane', fontsize=12, color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out, dpi=150)


def main_():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frame', default=f'{config.OUT_DIR}/sensor_rgb.png')
    ap.add_argument('--weight', default=None, help='alternate YOLOPX checkpoint')
    args = ap.parse_args()
    bgr = cv2.imread(args.frame)

    fig = plt.figure(figsize=(13.2, 8.6))
    gs = fig.add_gridspec(2, 2, height_ratios=(1.0, 0.92), hspace=.26, wspace=.22)
    ax = [fig.add_subplot(gs[0, :]), fig.add_subplot(gs[1, 0]),
          fig.add_subplot(gs[1, 1])]
    panel_image(ax[0], bgr)
    panel_bev(ax[1])
    panel_resolution(ax[2])
    for a in ax:
        a.legend(loc='upper right', frameon=False, fontsize=8.5, labelcolor=INK)
        a.tick_params(colors=INK, labelsize=8)
        for sp in a.spines.values():
            sp.set_color(MUTED)
        a.xaxis.label.set_color(INK); a.yaxis.label.set_color(INK)
    ax[1].set_aspect('equal')
    ax[1].grid(alpha=.15); ax[2].grid(alpha=.15, which='both')
    fig.suptitle(f'Inverse perspective mapping — camera {config.CAMERA_Z:.2f} m above '
                 f'a flat road, fov {config.CAMERA_FOV:.0f} deg, f = {lanes.F_PIXEL:.0f} px',
                 fontsize=12, color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    out = f'{config.OUT_DIR}/ipm_explained.png'
    fig.savefig(out, dpi=150)
    print(out)

    import yolopx
    net = yolopx.YOLOPX(args.weight)
    _, _, ll = net.infer(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    out2 = f'{config.OUT_DIR}/ipm_birdseye.png'
    draw_birdseye(bgr, ll, out2)
    print(out2)


if __name__ == '__main__':
    main_()
