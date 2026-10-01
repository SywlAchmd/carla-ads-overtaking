"""Conceptual figures for the slides: sensor rig, frames, planner, MPC, pipeline.

Read from config only -- no simulator, no logs. One consistent style across all
of them so a deck built from these does not look assembled from five sources.

    python plot_concepts.py            # all figures
    python plot_concepts.py --only rig
"""
import argparse
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, Rectangle

import config

# Satu warna aksen saja. Palet lebar dan kotak pastel membuat gambar teknik
# terlihat seperti poster; yang dibutuhkan slide adalah kontras, bukan variasi.
INK, ACCENT, MUTED = '#22303f', '#b5462f', '#94a3ad'
plt.rcParams.update({'font.size': 9, 'axes.edgecolor': INK, 'axes.linewidth': .8,
                     'axes.titlesize': 10, 'figure.titlesize': 11,
                     'xtick.color': INK, 'ytick.color': INK,
                     'axes.labelcolor': INK, 'text.color': INK,
                     'legend.framealpha': 1.0, 'legend.edgecolor': MUTED})

P = json.load(open(config.VEHICLE_PARAMS_JSON))
AXLE_OFFSET = -P['rear_axle_offset_x']          # sumbu belakang -> titik asal aktor
CAM_X = config.CAMERA_AHEAD_OF_AXLE         # kamera di depan sumbu belakang
BODY_HEIGHT = 1.45


def save(fig, name):
    path = os.path.join(config.OUT_DIR, name)
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Figure: {path}')


def rig():
    """Penempatan kamera: tampak samping dan tampak atas."""
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))

    # --- tampak samping: sumbu x diukur dari SUMBU BELAKANG ---
    a = ax[0]
    a.axhline(0, color=INK, lw=1.2)
    a.add_patch(Rectangle((-1.1, 0.05), config.EGO_LENGTH, BODY_HEIGHT,
                          fc='none', ec=MUTED, lw=1.2))
    for xr in (0.0, P['L']):                                     # roda
        a.add_patch(plt.Circle((xr, 0.34), 0.34, fc='none', ec=MUTED, lw=1.2))
    a.plot([0], [0], marker='|', ms=14, color=INK)
    a.annotate('rear axle\n(MPC state origin)', (0, 0), xytext=(0, -0.75),
               ha='center', fontsize=8, arrowprops=dict(arrowstyle='-', color=INK, lw=.8))
    a.plot([CAM_X], [config.CAMERA_Z], marker='s', ms=7, color=ACCENT)
    a.annotate(f'RGB + depth camera\n{config.CAMERA_Z} m above road',
               (CAM_X, config.CAMERA_Z), xytext=(CAM_X + 1.5, config.CAMERA_Z + 0.75),
               fontsize=8, color=ACCENT,
               arrowprops=dict(arrowstyle='->', color=ACCENT, lw=.9))
    a.annotate('', (0, -0.35), (CAM_X, -0.35), arrowprops=dict(arrowstyle='<->', color=INK, lw=.8))
    a.text(CAM_X / 2, -0.28, f'{CAM_X} m', ha='center', fontsize=8)
    a.annotate('', (CAM_X + 0.28, 0), (CAM_X + 0.28, config.CAMERA_Z),
               arrowprops=dict(arrowstyle='<->', color=INK, lw=.8))
    a.text(CAM_X + 0.38, config.CAMERA_Z / 2, f'{config.CAMERA_Z} m', fontsize=8)
    a.set_xlim(-1.6, 5.2); a.set_ylim(-1.1, 3.0); a.set_aspect('equal')
    a.set_title('Side view — placement follows the KITTI rig')
    a.axis('off')

    # --- tampak atas: aspek 1:1 supaya sudut 90 derajat terbaca benar ---
    # Jangkauan dibatasi 20 m: menggambar sampai 50 m memaksa sumbu terdistorsi
    # dan wedge-nya membanjiri bingkai. Jarak yang lebih jauh ditulis sebagai
    # keterangan, bukan digambar.
    b = ax[1]
    for sgn, ls in ((0.5, '-'), (-0.5, ':'), (-1.5, '-')):
        b.axhline(sgn * config.LANE_WIDTH, color=MUTED, lw=.8, ls=ls)
    b.add_patch(Rectangle((-1.1, -config.EGO_WIDTH / 2), config.EGO_LENGTH,
                          config.EGO_WIDTH, fc='none', ec=MUTED, lw=1.2))
    half = np.radians(config.CAMERA_FOV / 2)
    far = 12.0
    b.add_patch(Polygon([(CAM_X, 0), (CAM_X + far, far * np.tan(half)),
                         (CAM_X + far, -far * np.tan(half))],
                        fc=ACCENT, alpha=.06, ec=ACCENT, lw=.9, ls='--'))
    b.plot([CAM_X], [0], marker='s', ms=7, color=ACCENT)
    b.text(CAM_X + 4.2, 1.1, '%.0f deg FOV' % config.CAMERA_FOV, color=ACCENT, fontsize=8)

    # Titik ini yang menjelaskan kenapa S3 tidak bisa dijalankan dengan rig ini.
    x_enter = CAM_X + config.LANE_WIDTH
    b.plot([x_enter], [-config.LANE_WIDTH], marker='o', ms=6, mfc='white',
           mec=ACCENT, mew=1.4)
    b.annotate('a vehicle in the next lane enters\nthe frame only past this point',
               (x_enter, -config.LANE_WIDTH), xytext=(x_enter + 1.2, -6.4),
               fontsize=8, color=ACCENT,
               arrowprops=dict(arrowstyle='->', color=ACCENT, lw=.9))
    b.text(-1.1, config.LANE_WIDTH * 0.72, 'ego lane', fontsize=8, color=MUTED)
    b.text(-1.1, -config.LANE_WIDTH * 1.28, 'overtaking lane', fontsize=8, color=MUTED)
    b.set_xlim(-2.5, 13.5); b.set_ylim(-8.2, 6.4); b.set_aspect('equal')
    b.set_title('Top view (to scale) — first 12 m of the field of view')
    b.axis('off')

    fig.suptitle('Sensor configuration — one forward RGB camera with co-located depth', y=1.04)
    fig.text(0.5, -0.02,
             'RGB %dx%d @ 20 Hz, %.0f deg FOV   |   depth co-located, ideal sensor   |   '
             'stereo baseline %.2f m available but unused   |   '
             'measured range 45.6 m, overtake trigger at ~32 m'
             % (config.CAMERA_WIDTH, config.CAMERA_HEIGHT, config.CAMERA_FOV,
                config.CAMERA_BASELINE),
             ha='center', fontsize=8, color=MUTED)
    save(fig, 'sensor_rig.png')


def frames():
    """Tiga titik acuan pada satu kendaraan, dan rantai konversi frame."""
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.4))

    a = ax[0]
    a.add_patch(Rectangle((-AXLE_OFFSET, -config.EGO_WIDTH / 2), config.EGO_LENGTH,
                          config.EGO_WIDTH, fc='none', ec=MUTED, lw=1.2))
    # Pusat bodi (1,433) dan kamera (1,68) hanya berjarak 0,25 m, jadi labelnya
    # disebar ke arah berbeda -- ditumpuk vertikal, keduanya saling menutupi.
    points = [(0.0, 'rear axle', 'MPC state,\nplanner reference', (-2.6, 2.4), 'right'),
             (AXLE_OFFSET, 'body centre', 'safety zone,\nobstacle anchor', (0.1, 3.1), 'center'),
             (CAM_X, 'camera', 'perception origin', (3.4, 2.0), 'left')]
    for x, name, role, (tx, ty), ha in points:
        a.plot([x], [0], marker='o', ms=7, mfc='white', mec=ACCENT, mew=1.5, zorder=3)
        a.annotate('%s\n%s' % (name, role), (x, 0.35), xytext=(tx, ty), ha=ha, fontsize=8,
                   arrowprops=dict(arrowstyle='->', color=ACCENT, lw=.9,
                                   connectionstyle='arc3,rad=0.15'))
    a.annotate('', (0, -1.55), (AXLE_OFFSET, -1.55),
               arrowprops=dict(arrowstyle='<->', color=ACCENT, lw=1.2))
    a.text(AXLE_OFFSET / 2, -2.25, '%.3f m' % AXLE_OFFSET, ha='center', fontsize=9, color=ACCENT)
    a.text(AXLE_OFFSET / 2, -3.1, 'mixing these two shifted every obstacle\nby this much - '
                            'three times in this project', ha='center', fontsize=8, color=ACCENT)
    a.set_xlim(-3.4, 6.2); a.set_ylim(-4.2, 4.6); a.set_aspect('equal'); a.axis('off')
    a.set_title('Three reference points on one vehicle')

    b = ax[1]
    b.axis('off'); b.set_xlim(0, 1); b.set_ylim(0, 1)
    stage = [('CARLA world', 'left-handed, y points right\nactor origin = body centre'),
             ('Right-handed', 'y -> -y,  yaw -> -yaw\norigin moved to rear axle'),
             ('Road frame', 'rotated by road heading\nx = along road, y = lateral')]
    for i, (name, body) in enumerate(stage):
        top = 0.92 - i * 0.30
        b.add_patch(Rectangle((0.04, top - 0.20), 0.92, 0.20, fc='none', ec=INK, lw=1.0,
                              transform=b.transAxes))
        b.text(0.08, top - 0.06, name, fontsize=9.5, fontweight='semibold',
               transform=b.transAxes)
        b.text(0.08, top - 0.165, body, fontsize=8, color=MUTED, transform=b.transAxes)
        if i < 2:                                    # panah MENURUN, searah bacaan
            b.annotate('', (0.5, top - 0.30), (0.5, top - 0.21),
                       xycoords=b.transAxes, textcoords=b.transAxes,
                       arrowprops=dict(arrowstyle='->', color=INK, lw=1.1))
    b.text(0.5, 0.04, 'localization.py is the only module allowed to convert frames',
           ha='center', fontsize=8, color=MUTED, style='italic', transform=b.transAxes)
    b.set_title('Frame conversion chain')

    fig.suptitle('Localization - ground truth transform, converted once and only here', y=1.0)
    save(fig, 'localization_frames.png')


def pipeline():
    """Rantai end-to-end berikut lajunya."""
    fig, ax = plt.subplots(figsize=(11, 3.6))
    ax.set_xlim(0, 100); ax.set_ylim(0, 34); ax.axis('off')

    blocks = [('Camera\nRGB + depth', 20, 'sensors.py'),
            ('YOLOPX\n+ tracking', 20, 'perception.py\ntracking.py'),
            ('Localization', 20, 'localization.py'),
            ('Behavior FSM', 10, 'planning.py'),
            ('Local planner\n9 candidates', 10, 'planning.py'),
            ('MPC\nCasADi + IPOPT', 20, 'control.py'),
            ('Throttle PI\n+ steer map', 20, 'control.py')]
    width, spacing = 11.6, 2.0
    for i, (name, hz, module) in enumerate(blocks):
        x = i * (width + spacing)
        color = ACCENT if hz == 10 else INK
        ax.add_patch(Rectangle((x, 14), width, 11, fc='none', ec=color, lw=1.1))
        ax.text(x + width / 2, 21.0, name, ha='center', va='center', fontsize=8.5)
        ax.text(x + width / 2, 16.2, f'{hz} Hz', ha='center', fontsize=8, color=color)
        ax.text(x + width / 2, 11.6, module, ha='center', va='top', fontsize=7, color=MUTED)
        if i < len(blocks) - 1:
            ax.annotate('', (x + width + spacing, 19.5), (x + width, 19.5),
                        arrowprops=dict(arrowstyle='->', color=INK, lw=1.0))

    ax.annotate('', (0.5, 29.5), (95.0, 29.5), arrowprops=dict(arrowstyle='<-', color=MUTED, lw=.9))
    ax.text(48, 30.4, 'CARLA synchronous tick — every command goes through simulation.tick',
            ha='center', fontsize=8, color=MUTED)
    # Batas frame ada SETELAH localization, bukan di tengah FSM.
    limit = 2 * (width + spacing) + width + spacing / 2
    ax.annotate('', (limit, 7.8), (limit, 13.6),
                arrowprops=dict(arrowstyle='-', color=MUTED, lw=.8, ls=':'))
    ax.text(limit - 2, 6.6, 'ego frame, relative to ego', ha='right', fontsize=8, color=MUTED)
    ax.text(limit + 2, 6.6, 'road frame, absolute', ha='left', fontsize=8, color=MUTED)
    ax.text(limit, 3.0, 'converted once, in localization.obstacles_ego_to_road  '
                        '(the FSM then reads gaps back relative to the ego)',
            ha='center', fontsize=8, color=MUTED, style='italic')
    ax.set_title('End-to-end pipeline — one tick is 50 ms', pad=14)
    save(fig, 'pipeline.png')


def mpc():
    """Receding horizon. Ramalan yang ditumpuk di satu sumbu saling menutupi,
    jadi horizonnya digambar sebagai batang waktu di panel bawah."""
    fig, ax = plt.subplots(2, 1, figsize=(11, 5.0), sharex=True,
                           gridspec_kw=dict(height_ratios=[2.1, 1]))
    T, dt, N = 4.0, config.MPC_DT, config.MPC_N
    pause = 0.6                       # jarak antar tick YANG DIGAMBAR, bukan yang nyata

    def quintic(tt):
        u = np.clip(tt / T, 0, 1)
        return -config.LANE_WIDTH * (10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5)

    a = ax[0]
    t = np.linspace(0, T, 200)
    a.plot(t, quintic(t), '--', color=INK, lw=1.7, label='planner reference', zorder=3)
    tick_colors = [ACCENT, '#4e7488', MUTED]
    for i in range(3):
        t0 = i * pause
        th = np.linspace(t0, t0 + N * dt, 60)
        bump = 0.28 * np.exp(-2.6 * (th - t0))
        a.plot(th, quintic(th) + bump, color=tick_colors[i], lw=1.3, zorder=2,
               label='2 s prediction, recomputed every tick' if i == 0 else None)
        a.plot([t0], [quintic(t0) + bump[0]], marker='o', ms=5,
               color=tick_colors[i], zorder=4)
    te = np.linspace(0, config.FIXED_DELTA_SECONDS, 10)
    a.plot(te, quintic(te) + 0.28 * np.exp(-2.6 * te), color=ACCENT, lw=5,
           solid_capstyle='butt', zorder=5, label='actually executed: the first 50 ms')
    a.set_ylabel('lateral position (m)')
    a.set_ylim(-4.1, 0.9)
    a.legend(loc='upper right', fontsize=8)
    a.grid(alpha=.22)
    a.set_title('Model predictive control - predict 2 s, execute 50 ms, discard the rest, repeat')

    b = ax[1]
    for i in range(3):
        t0 = i * pause
        y = 2 - i
        b.barh(y, N * dt, left=t0, height=.5, color=tick_colors[i], alpha=.2,
               edgecolor=tick_colors[i], lw=1.0)
        b.barh(y, config.FIXED_DELTA_SECONDS, left=t0, height=.5, color=tick_colors[i])
        b.text(t0 - 0.07, y, 'tick %d' % (i + 1), ha='right', va='center',
               fontsize=8, color=tick_colors[i])
    b.text(pause * 2 + N * dt + 0.08, 1, 'solid = executed\nshaded = predicted, then discarded',
           va='center', fontsize=8, color=MUTED)
    b.set_ylim(-0.4, 2.9); b.set_yticks([])
    b.set_xlabel('time (s)')
    b.spines[['left', 'right', 'top']].set_visible(False)
    b.grid(axis='x', alpha=.22)
    fig.text(0.5, -0.02, 'Ticks are drawn %.1f s apart so the horizons stay legible; '
                         'the real spacing is %.0f ms.' % (pause, config.FIXED_DELTA_SECONDS * 1e3),
             ha='center', fontsize=8, color=MUTED, style='italic')
    fig.tight_layout()
    save(fig, 'mpc_concept.png')


def scenario():
    """Susunan skenario S1 berikut parameter kedua kendaraan."""
    fig, ax = plt.subplots(2, 1, figsize=(11, 5.2),
                           gridspec_kw=dict(height_ratios=[1.35, 1]))

    a = ax[0]
    for sgn in (0.5, -0.5, -1.5):
        a.axhline(sgn * config.LANE_WIDTH, color=MUTED, lw=.8,
                  ls=':' if sgn == -0.5 else '-')
    a.text(-12.5, 0, 'ego lane', fontsize=8, color=MUTED, va='center')
    a.text(-12.5, -config.LANE_WIDTH, 'overtaking\nlane', fontsize=8, color=MUTED, va='center')

    gap, v_ego, v_tgt = config.SCENARIOS['S1'][0][0], config.EGO_V0, config.SCENARIOS['S1'][0][2]
    for x, pj, lb, color, name, ve in (
            (0.0, P['length'], P['width'], ACCENT, 'ego', v_ego),
            (gap, config.OTHER_LENGTH, config.OTHER_WIDTH, INK, 'target', v_tgt)):
        a.add_patch(Rectangle((x - pj / 2, -lb / 2), pj, lb, fc='none', ec=color, lw=1.3))
        a.annotate('', (x + pj / 2 + 7, 0), (x + pj / 2 + 1, 0),
                   arrowprops=dict(arrowstyle='->', color=color, lw=1.2))
        # Nama dan kecepatan DI ATAS kendaraannya: ditaruh di bawah, keduanya
        # jatuh ke lajur menyalip dan terbaca seolah milik lajur itu.
        a.text(x, lb / 2 + 0.35, '%s  -  %.1f m/s (%.1f km/h)' % (name, ve, ve * 3.6),
               ha='center', va='bottom', fontsize=8.5, color=color)

    a.annotate('', (P['length'] / 2, -config.LANE_WIDTH),
               (gap - config.OTHER_LENGTH / 2, -config.LANE_WIDTH),
               arrowprops=dict(arrowstyle='<->', color=INK, lw=1.0))
    a.text(gap / 2, -config.LANE_WIDTH + 0.35, 'initial gap %.0f m (body to body)' % gap,
           ha='center', fontsize=8)
    a.text(gap / 2, -config.LANE_WIDTH - 1.5,
           'closing at %.1f m/s  ->  overtaking lane must be cleared before the gap runs out'
           % (v_ego - v_tgt), ha='center', fontsize=8, color=MUTED)
    a.set_xlim(-13, 74); a.set_ylim(-7.4, 3.6); a.axis('off')
    a.set_title('Scenario S1 - flying overtaking, target lane empty')

    b = ax[1]
    b.axis('off'); b.set_xlim(0, 1); b.set_ylim(0, 1)
    body = [['Model', 'Dodge Charger 2020', 'Lincoln MKZ 2020'],
           ['Length x width', '%.3f x %.3f m' % (P['length'], P['width']),
            '%.3f x %.3f m' % (config.OTHER_LENGTH, config.OTHER_WIDTH)],
           ['Wheelbase', '%.3f m' % P['L'], 'not modelled'],
           ['Mass', '%.0f kg' % P['mass'], 'not modelled'],
           ['Speed', '%.1f m/s  (%.1f km/h)' % (v_ego, v_ego * 3.6),
            '%.1f m/s  (%.1f km/h)' % (v_tgt, v_tgt * 3.6)],
           ['Control', 'MPC, closed loop', 'speed forced along the road'],
           ['Start', 'spawn point 75, Town04', '%.0f m ahead, same lane' % gap]]
    x = [0.01, 0.22, 0.60]
    for j, k in enumerate(['', 'Ego', 'Target']):
        b.text(x[j], 0.95, k, fontsize=9, fontweight='semibold', transform=b.transAxes)
    b.plot([0.01, 0.99], [0.90, 0.90], color=INK, lw=.9, transform=b.transAxes)
    for i, rows in enumerate(body):
        y = 0.79 - i * 0.118
        for j, cell in enumerate(rows):
            b.text(x[j], y, cell, fontsize=8, color=MUTED if j == 0 else INK,
                   transform=b.transAxes)
    b.text(0.01, -0.09, 'Road: straight 400 m section, lane width %.2f m. Target dimensions are '
                        'assumed fixed - perception never measures them.' % config.LANE_WIDTH,
           fontsize=8, color=MUTED, style='italic', transform=b.transAxes)
    fig.tight_layout()
    save(fig, 'scenario_s1.png')


ALL = dict(rig=rig, frames=frames, scenario=scenario, pipeline=pipeline, mpc=mpc)

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', choices=list(ALL))
    a = ap.parse_args()
    for name, fn in ([(a.only, ALL[a.only])] if a.only else ALL.items()):
        fn()
