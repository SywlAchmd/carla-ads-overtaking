"""Grafik hasil satu run tertutup, untuk bab 4 (rencana kerja bagian 11.6).

Dibaca dari log mentah supaya tidak perlu menjalankan ulang simulasi.

    python plot_run.py                     # out/run_s1_mpc_gt.npz -> out/run_s1_mpc.png
    python plot_run.py --skenario S3
"""
import argparse
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import config
import evaluation

# Latar tiap panel diwarnai menurut state FSM: satu gambar cukup untuk membaca
# kapan tiap fase terjadi, tanpa menaruh garis vertikal di semua panel.
WARNA = {'LANE_KEEPING': '#ffffff', 'CHECK_OVERTAKE': '#fdf3d0',
         'LANE_CHANGE_OVERTAKE': '#dbe9fb', 'OVERTAKING': '#dcf2d7',
         'LANE_CHANGE_RETURN': '#fbdfe4'}


def latar_state(ax, t, st):
    awal = 0
    for i in range(1, len(st) + 1):
        if i == len(st) or st[i] != st[awal]:
            ax.axvspan(t[awal], t[i - 1], color=WARNA.get(st[awal], '#eeeeee'), lw=0, zorder=0)
            awal = i


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--skenario', default='S1', choices=list(config.SKENARIO))
    args = ap.parse_args()
    sk = args.skenario.lower()

    f = np.load(os.path.join(config.OUT_DIR, f'run_{sk}_mpc_gt.npz'))
    L, st, pos = f['log'], f['fsm_state'], f['posisi_kendaraan']
    k = {nama: i for i, nama in enumerate(f['kolom'])}
    t, y, yaw = L[:, 0], L[:, k['y']], L[:, k['yaw']]
    xc = L[:, k['x']] + config.SUMBU_KE_PUSAT * np.cos(yaw)
    yc = y + config.SUMBU_KE_PUSAT * np.sin(yaw)

    fig, ax = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    for a in ax:
        latar_state(a, t, st)
        a.grid(alpha=.3)

    ax[0].axhline(0, color='0.5', lw=.8)
    ax[0].axhline(config.SIDE_SIGN * config.LANE_WIDTH, color='0.5', lw=.8)
    for tepi in (0.5, -0.5, -1.5):
        ax[0].axhline(tepi * config.LANE_WIDTH, color='0.75', ls=':', lw=.8)
    ax[0].plot(t, y, lw=2, label='ego (sumbu belakang)')
    ax[0].plot(t, L[:, k['y_goal']], '--', lw=1.2, label='tengah lajur tujuan FSM')
    ax[0].set_ylabel('simpangan lateral (m)'); ax[0].legend(loc='lower right', fontsize=8)

    ax[1].plot(t, L[:, k['v']] * 3.6, lw=2, label='kecepatan ego')
    ax[1].plot(t, L[:, k['v_goal']] * 3.6, '--', lw=1.2, label='acuan planner')
    ax[1].axhline(config.V_MAX * 3.6, color='crimson', ls=':', lw=1, label='batas 50 km/jam')
    ax[1].set_ylabel('kecepatan (km/jam)'); ax[1].legend(loc='lower right', fontsize=8)

    for i in range(pos.shape[1]):
        d = evaluation.jarak_kotak(pos[:, i, 0] - xc, pos[:, i, 1] - yc,
                                   f['dim_ego'], f['dim_kendaraan'][i], yaw, pos[:, i, 2])
        ax[2].plot(t, d, lw=2, label='target' if i == 0 else f'kendaraan {i + 1}')
    ax[2].axhline(config.JARAK_AMAN, color='crimson', ls=':', lw=1,
                  label=f'syarat {config.JARAK_AMAN:.0f} m')
    ax[2].set_ylabel('jarak antar bodi (m)'); ax[2].legend(loc='upper right', fontsize=8)

    ax[3].plot(t, L[:, k['delta_cmd']], lw=1.5, label='delta MPC (rad)')
    ax3b = ax[3].twinx()
    ax3b.plot(t, L[:, k['solve_ms']], color='tab:orange', lw=.9, label='waktu solve (ms)')
    ax3b.axhline(config.FIXED_DELTA_SECONDS * 1e3, color='crimson', ls=':', lw=1)
    ax3b.set_ylabel('waktu solve (ms), anggaran 50 ms')
    ax[3].set_ylabel('sudut kemudi (rad)'); ax[3].set_xlabel('t (detik)')
    ax[3].legend(loc='upper left', fontsize=8); ax3b.legend(loc='upper right', fontsize=8)

    fig.legend(handles=[plt.Rectangle((0, 0), 1, 1, color=w) for w in WARNA.values()],
               labels=list(WARNA), loc='upper center', ncol=5, fontsize=8, frameon=False)
    fig.suptitle(f'Skenario {args.skenario} — MPC + ground truth perception', y=0.975)
    fig.tight_layout(rect=(0, 0, 1, 0.945))
    keluar = os.path.join(config.OUT_DIR, f'run_{sk}_mpc.png')
    fig.savefig(keluar, dpi=150)
    print(f'Grafik: {keluar}')


if __name__ == '__main__':
    main()
