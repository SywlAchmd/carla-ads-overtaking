"""Grafik pembanding MPC + ground truth versus MPC + vision, satu skenario.

Dibaca dari log mentah; tidak perlu menjalankan ulang simulasi.

    python plot_banding.py                  # S1 -> out/banding_s1.png
"""
import argparse
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import config
import evaluation

GAYA = {'gt': dict(color='tab:blue', lw=2.0, label='MPC + ground truth'),
        'vision': dict(color='tab:red', lw=1.8, ls='--', label='MPC + vision')}


def muat(sk, mode):
    f = np.load(os.path.join(config.OUT_DIR, f'run_{sk}_mpc_{mode}.npz'))
    k = {n: i for i, n in enumerate(f['kolom'])}
    L, pos = f['log'], f['posisi_kendaraan']
    yaw = L[:, k['yaw']]
    xc = L[:, k['x']] + config.SUMBU_KE_PUSAT * np.cos(yaw)
    yc = L[:, k['y']] + config.SUMBU_KE_PUSAT * np.sin(yaw)
    jarak = evaluation.jarak_kotak(pos[:, 0, 0] - xc, pos[:, 0, 1] - yc,
                                   f['dim_ego'], f['dim_kendaraan'][0], yaw, pos[:, 0, 2])
    return dict(t=L[:, k['t']], y=L[:, k['y']], v=L[:, k['v']] * 3.6, jarak=jarak,
                n_layak=L[:, k['n_layak']], st=f['fsm_state'])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--skenario', default='S1', choices=list(config.SKENARIO))
    args = ap.parse_args()
    sk = args.skenario.lower()
    d = {m: muat(sk, m) for m in ('gt', 'vision')}

    fig, ax = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    for a in ax:
        a.grid(alpha=.3)

    # Tepi lajur digambar eksplisit: "keluar lajur" adalah temuan yang perlu
    # terbaca dari gambar, bukan cuma dari angka.
    ax[0].axhline(0, color='0.5', lw=.8)
    ax[0].axhline(config.SIDE_SIGN * config.LANE_WIDTH, color='0.5', lw=.8)
    for tepi in (0.5, -0.5, -1.5):
        ax[0].axhline(tepi * config.LANE_WIDTH, color='0.75', ls=':', lw=.8)
    ax[0].set_ylabel('simpangan lateral (m)')

    ax[1].axhline(config.V_MAX * 3.6, color='crimson', ls=':', lw=1, label='batas 50 km/jam')
    ax[1].set_ylabel('kecepatan (km/jam)')

    ax[2].axhline(config.JARAK_AMAN, color='crimson', ls=':', lw=1.2,
                  label=f'syarat lulus {config.JARAK_AMAN:.0f} m')
    ax[2].set_ylabel('jarak antar bodi (m)')
    # Dipotong ke 12 m: yang perlu terbaca adalah daerah kritis dekat syarat
    # 1,0 m, bukan 68 m saat kedua kendaraan masih berjauhan.
    ax[2].set_ylim(0, 12)

    ax[3].set_ylabel('kandidat planner lolos')
    ax[3].set_xlabel('t (detik)')
    ax[3].set_ylim(-0.4, 9.6)

    for m, g in GAYA.items():
        x = d[m]
        ax[0].plot(x['t'], x['y'], **g)
        ax[1].plot(x['t'], x['v'], **g)
        ax[2].plot(x['t'], x['jarak'], **g)
        ax[3].plot(x['t'], x['n_layak'], **g)

    for m, warna, xy in (('gt', 'tab:blue', (-64, 26)), ('vision', 'tab:red', (16, -6))):
        x = d[m]
        i = int(np.argmin(x['jarak']))
        ax[2].annotate(f"minimum {x['jarak'][i]:.2f} m", (x['t'][i], x['jarak'][i]),
                       textcoords='offset points', xytext=xy, color=warna,
                       fontsize=9, fontweight='bold',
                       arrowprops=dict(arrowstyle='->', color=warna, lw=1))
        nol = int((x['n_layak'] == 0).sum())
        ax[3].annotate(f'{nol} tick tanpa kandidat', (0.99, 0.30 + 0.16 * (m == 'gt')),
                       xycoords='axes fraction', ha='right', color=warna, fontsize=9)

    ax[0].legend(loc='lower right', fontsize=9)
    for a in ax[1:3]:
        a.legend(loc='lower right', fontsize=8)
    fig.suptitle(f'Skenario {args.skenario} — pengaruh sumber perception terhadap hasil kendali',
                 y=0.985)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    keluar = os.path.join(config.OUT_DIR, f'banding_{sk}.png')
    fig.savefig(keluar, dpi=150)
    print(f'Grafik: {keluar}')


if __name__ == '__main__':
    main()
