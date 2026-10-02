"""Grafik hasil satu run tertutup, untuk bab 4 (rencana kerja bagian 11.6).

Dibaca dari log mentah supaya tidak perlu menjalankan ulang simulasi.

    python plot_run.py                        # GT  -> out/run_s1_mpc_gt.png
    python plot_run.py --perception vision    # -> out/run_s1_mpc_vision.png
    python plot_run.py --scenario S3
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
COLORS = {'LANE_KEEPING': '#ffffff', 'CHECK_OVERTAKE': '#fdf3d0',
         'LANE_CHANGE_OVERTAKE': '#dbe9fb', 'OVERTAKING': '#dcf2d7',
         'LANE_CHANGE_RETURN': '#fbdfe4'}


def shade_states(ax, t, st):
    start = 0
    for i in range(1, len(st) + 1):
        if i == len(st) or st[i] != st[start]:
            ax.axvspan(t[start], t[i - 1], color=COLORS.get(st[start], '#eeeeee'), lw=0, zorder=0)
            start = i


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', default='S1', choices=list(config.SCENARIOS))
    ap.add_argument('--perception', default='gt', choices=('gt', 'vision'))
    args = ap.parse_args()
    sk = args.scenario.lower()

    f = np.load(os.path.join(config.OUT_DIR, f'run_{sk}_mpc_{args.perception}.npz'))
    L, st, pos = f['log'], f['fsm_state'], f['vehicle_positions']
    k = {name: i for i, name in enumerate(f['columns'])}
    t, y, yaw = L[:, 0], L[:, k['y']], L[:, k['yaw']]
    xc = L[:, k['x']] + config.AXLE_TO_CENTER * np.cos(yaw)
    yc = y + config.AXLE_TO_CENTER * np.sin(yaw)

    fig, ax = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    for a in ax:
        shade_states(a, t, st)
        a.grid(alpha=.3)

    ax[0].axhline(0, color='0.5', lw=.8)
    ax[0].axhline(config.SIDE_SIGN * config.LANE_WIDTH, color='0.5', lw=.8)
    for edges in (0.5, -0.5, -1.5):
        ax[0].axhline(edges * config.LANE_WIDTH, color='0.75', ls=':', lw=.8)
    # Menurut PETA, bukan frame kendali: frame jalur vision dijangkarkan kamera dan
    # ikut berputar oleh bias arah, jadi `y`-nya bisa tampak 0,3 m dari tengah
    # padahal ego tepat di tengah. Sasaran FSM digeser offset yang sama supaya
    # keduanya tetap sebanding. Untuk GT kedua frame identik.
    y_map = L[:, k['y_map']] if 'y_map' in k else y
    ax[0].plot(t, y_map, lw=2, label='ego (rear axle)')
    ax[0].plot(t, L[:, k['y_goal']] + (y_map - y), '--', lw=1.2, label='FSM target lane centre')
    ax[0].set_ylabel('lateral deviation (m)'); ax[0].legend(loc='upper right', fontsize=8)

    ax[1].plot(t, L[:, k['v']] * 3.6, lw=2, label='ego speed')
    ax[1].plot(t, L[:, k['v_goal']] * 3.6, '--', lw=1.2, label='planner reference')
    ax[1].axhline(config.V_MAX * 3.6, color='crimson', ls=':', lw=1, label='limit 50 km/h')
    ax[1].set_ylabel('speed (km/h)'); ax[1].legend(loc='upper right', fontsize=8)

    for i in range(pos.shape[1]):
        d = evaluation.box_distance(pos[:, i, 0] - xc, pos[:, i, 1] - yc,
                                   f['dim_ego'], f['vehicle_dims'][i], yaw, pos[:, i, 2])
        ax[2].plot(t, d, lw=2, label='target' if i == 0 else f'vehicle {i + 1}')
    ax[2].axhline(config.SAFE_DISTANCE, color='crimson', ls=':', lw=1,
                  label=f'requirement {config.SAFE_DISTANCE:.0f} m')
    ax[2].set_ylabel('body-to-body distance (m)'); ax[2].legend(loc='upper right', fontsize=8)

    ax[3].plot(t, L[:, k['delta_cmd']], lw=1.5, label='MPC steering delta (rad)')
    ax3b = ax[3].twinx()
    ax3b.plot(t, L[:, k['solve_ms']], color='tab:orange', lw=.9, label='solve time (ms)')
    ax3b.axhline(config.FIXED_DELTA_SECONDS * 1e3, color='crimson', ls=':', lw=1)
    ax3b.set_ylabel('solve time (ms), budget 50 ms')
    ax[3].set_ylabel('steering angle (rad)'); ax[3].set_xlabel('t (s)')
    # Dua sumbu, SATU legend: keduanya di kanan atas akan saling menimpa.
    g1, l1 = ax[3].get_legend_handles_labels()
    g2, l2 = ax3b.get_legend_handles_labels()
    ax3b.legend(g1 + g2, l1 + l2, loc='upper right', fontsize=8)

    fig.legend(handles=[plt.Rectangle((0, 0), 1, 1, color=w) for w in COLORS.values()],
               labels=list(COLORS), loc='upper center', ncol=5, fontsize=8, frameon=False)
    name_p = 'ground truth perception' if args.perception == 'gt' else 'vision perception (YOLOPX + depth)'
    fig.suptitle(f'End-to-end overtaking result — Scenario {args.scenario}, MPC + {name_p}',
                 y=0.975)
    fig.tight_layout(rect=(0, 0, 1, 0.945))
    out = os.path.join(config.OUT_DIR, f'run_{sk}_mpc_{args.perception}.png')
    fig.savefig(out, dpi=150)
    print(f'Plot: {out}')


if __name__ == '__main__':
    main()
