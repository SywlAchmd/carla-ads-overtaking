"""Galat lokalisasi odometri terhadap ground truth, seluruh ulangan eksperimen
(WRITING_SUMMARY bagian 36). Dibaca dari log mentah; tanpa simulator.

    SKRIPSI_OUT=out/localization_branch python plot_localization.py
        -> localization_error_s1.png

Panel terakhir menjelaskan SUMBER galat: slip lateral sumbu belakang, yang
dianggap nol oleh dead reckoning (dan oleh bicycle model MPC). Integralnya
digambar di atas galat lateral -- kalau keduanya berimpit, galatnya galat model,
bukan galat sensor.
"""
import argparse
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import config
from plot_run import COLORS, shade_states

ODOM = 'tab:orange'          # sama dengan plot_compare: vision + odometri
INK = '0.25'                 # garis acuan & teks: tinta netral, bukan warna seri
# Reid dkk. (2019), batas galat 95%: lateral jalan bebas hambatan / jalan lokal,
# dan arah jalan lokal (WRITING_SUMMARY 36.2).
LAT_95 = ((0.20, 'Reid 95%, freeway'), (0.10, 'Reid 95%, local street'))
YAW_95 = 0.17


def errors(L, k):
    t = L[:, k['t']]
    lat = L[:, k['y']] - L[:, k['y_gt']]
    lon = L[:, k['x']] - L[:, k['x_gt']]
    yaw = np.degrees(np.angle(np.exp(1j * (L[:, k['yaw']] - L[:, k['yaw_gt']]))))
    # slip sumbu belakang (GT): arah gerak titik sumbu belakang vs arah hadap
    dx, dy = np.gradient(L[:, k['x_gt']], t), np.gradient(L[:, k['y_gt']], t)
    slip = np.arctan2(dy, dx) - L[:, k['yaw_gt']]
    # simpangan yang tidak terlihat dead reckoning, mulai dari galat tick pertama
    # (integrasi sudah berjalan selama pemanasan, sebelum log dicatat)
    lost = lat[0] - np.cumsum(np.hypot(dx, dy) * np.sin(slip) * np.gradient(t))
    return t, lat, lon, yaw, np.degrees(slip), lost


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--file', default='experiment_s1_vision_odom.npz')
    args = ap.parse_args()
    f = np.load(os.path.join(config.OUT_DIR, args.file), allow_pickle=True)
    k = {n: i for i, n in enumerate(f['columns'])}
    runs = [errors(L, k) for L in f['log']]
    t, st = runs[0][0], f['fsm_state'][0]

    fig, ax = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    for a in ax:
        shade_states(a, t, st)
        a.grid(alpha=.3)
        a.axhline(0, color='0.6', lw=.8)

    for i, (t_, lat, lon, yaw, slip, lost) in enumerate(runs):
        kw = dict(color=ODOM, lw=1.0, alpha=.55,
                  label=f'{len(runs)} runs' if i == 0 else None)
        ax[0].plot(t_, lat, **kw)
        ax[1].plot(t_, lon, **kw)
        ax[2].plot(t_, yaw, **kw)
        ax[3].plot(t_, slip, **kw)
    ax[0].plot(runs[0][0], runs[0][5], color=INK, lw=1.4, ls='--',
               label='integrated rear-axle slip (run 1)')

    for v, name in LAT_95:
        for s in (1, -1):
            ax[0].axhline(s * v, color=INK, ls=':', lw=.9)
        ax[0].annotate(name, (t[-1], v), textcoords='offset points', xytext=(-4, 3),
                       ha='right', fontsize=8, color=INK)
    for s in (1, -1):
        ax[2].axhline(s * YAW_95, color=INK, ls=':', lw=.9)
    ax[2].annotate('Reid 95%, local street', (t[-1], YAW_95), textcoords='offset points',
                   xytext=(-4, 3), ha='right', fontsize=8, color=INK)

    # ruang di atas garis acuan teratas, supaya labelnya tidak menabrak tepi panel
    for a, ref, col in ((ax[0], LAT_95[0][0], 1), (ax[2], YAW_95, 3)):
        lim = 1.3 * max(ref, max(np.abs(r[col]).max() for r in runs))
        a.set_ylim(-lim, lim)

    ax[0].set_ylabel('lateral error (m)')
    ax[1].set_ylabel('longitudinal error (m)')
    ax[2].set_ylabel('heading error (deg)')
    ax[3].set_ylabel('rear-axle slip, GT (deg)')
    ax[3].set_xlabel('t (s)')
    ax[0].legend(loc='lower left', fontsize=8)

    fig.legend(handles=[plt.Rectangle((0, 0), 1, 1, color=w) for w in COLORS.values()],
               labels=list(COLORS), loc='upper center', ncol=5, fontsize=8, frameon=False)
    fig.suptitle('Odometry localization error against ground truth', y=0.975)
    fig.tight_layout(rect=(0, 0, 1, 0.945))
    out = os.path.join(config.OUT_DIR, 'localization_error_s1.png')
    fig.savefig(out, dpi=150)
    print(f'Plot: {out}')

    lat = np.abs(np.concatenate([r[1] for r in runs]))
    yaw = np.abs(np.concatenate([r[3] for r in runs]))
    lon = np.abs(np.concatenate([r[2] for r in runs]))
    corr = [np.corrcoef(r[1], r[5])[0, 1] for r in runs]
    print(f'{len(runs)} runs, all ticks pooled:')
    print(f'  lateral       95% {np.percentile(lat, 95):.3f} m   max {lat.max():.3f} m')
    print(f'  longitudinal  95% {np.percentile(lon, 95):.3f} m   max {lon.max():.3f} m')
    print(f'  heading       95% {np.percentile(yaw, 95):.3f} deg max {yaw.max():.3f} deg')
    print(f'  corr(lateral error, integrated slip) per run: '
          f'min {min(corr):.3f}, mean {np.mean(corr):.3f}')


if __name__ == '__main__':
    main()
