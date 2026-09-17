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
GARIS, AKSEN, REDUP = '#22303f', '#b5462f', '#94a3ad'
plt.rcParams.update({'font.size': 9, 'axes.edgecolor': GARIS, 'axes.linewidth': .8,
                     'axes.titlesize': 10, 'figure.titlesize': 11,
                     'xtick.color': GARIS, 'ytick.color': GARIS,
                     'axes.labelcolor': GARIS, 'text.color': GARIS,
                     'legend.framealpha': 1.0, 'legend.edgecolor': REDUP})

P = json.load(open(config.VEHICLE_PARAMS_JSON))
SUMBU = -P['rear_axle_offset_x']          # sumbu belakang -> titik asal aktor
KAM_X = config.KAMERA_DEPAN_SUMBU         # kamera di depan sumbu belakang
TINGGI_BODI = 1.45


def simpan(fig, nama):
    jalur = os.path.join(config.OUT_DIR, nama)
    fig.savefig(jalur, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Figure: {jalur}')


def rig():
    """Penempatan kamera: tampak samping dan tampak atas."""
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))

    # --- tampak samping: sumbu x diukur dari SUMBU BELAKANG ---
    a = ax[0]
    a.axhline(0, color=GARIS, lw=1.2)
    a.add_patch(Rectangle((-1.1, 0.05), config.EGO_PANJANG, TINGGI_BODI,
                          fc='none', ec=REDUP, lw=1.2))
    for xr in (0.0, P['L']):                                     # roda
        a.add_patch(plt.Circle((xr, 0.34), 0.34, fc='none', ec=REDUP, lw=1.2))
    a.plot([0], [0], marker='|', ms=14, color=GARIS)
    a.annotate('rear axle\n(MPC state origin)', (0, 0), xytext=(0, -0.75),
               ha='center', fontsize=8, arrowprops=dict(arrowstyle='-', color=GARIS, lw=.8))
    a.plot([KAM_X], [config.KAMERA_Z], marker='s', ms=7, color=AKSEN)
    a.annotate(f'RGB + depth camera\n{config.KAMERA_Z} m above road',
               (KAM_X, config.KAMERA_Z), xytext=(KAM_X + 1.5, config.KAMERA_Z + 0.75),
               fontsize=8, color=AKSEN,
               arrowprops=dict(arrowstyle='->', color=AKSEN, lw=.9))
    a.annotate('', (0, -0.35), (KAM_X, -0.35), arrowprops=dict(arrowstyle='<->', color=GARIS, lw=.8))
    a.text(KAM_X / 2, -0.28, f'{KAM_X} m', ha='center', fontsize=8)
    a.annotate('', (KAM_X + 0.28, 0), (KAM_X + 0.28, config.KAMERA_Z),
               arrowprops=dict(arrowstyle='<->', color=GARIS, lw=.8))
    a.text(KAM_X + 0.38, config.KAMERA_Z / 2, f'{config.KAMERA_Z} m', fontsize=8)
    a.set_xlim(-1.6, 5.2); a.set_ylim(-1.1, 3.0); a.set_aspect('equal')
    a.set_title('Side view — placement follows the KITTI rig')
    a.axis('off')

    # --- tampak atas: aspek 1:1 supaya sudut 90 derajat terbaca benar ---
    # Jangkauan dibatasi 20 m: menggambar sampai 50 m memaksa sumbu terdistorsi
    # dan wedge-nya membanjiri bingkai. Jarak yang lebih jauh ditulis sebagai
    # keterangan, bukan digambar.
    b = ax[1]
    for sgn, ls in ((0.5, '-'), (-0.5, ':'), (-1.5, '-')):
        b.axhline(sgn * config.LANE_WIDTH, color=REDUP, lw=.8, ls=ls)
    b.add_patch(Rectangle((-1.1, -config.EGO_LEBAR / 2), config.EGO_PANJANG,
                          config.EGO_LEBAR, fc='none', ec=REDUP, lw=1.2))
    setengah = np.radians(config.KAMERA_FOV / 2)
    jauh = 12.0
    b.add_patch(Polygon([(KAM_X, 0), (KAM_X + jauh, jauh * np.tan(setengah)),
                         (KAM_X + jauh, -jauh * np.tan(setengah))],
                        fc=AKSEN, alpha=.06, ec=AKSEN, lw=.9, ls='--'))
    b.plot([KAM_X], [0], marker='s', ms=7, color=AKSEN)
    b.text(KAM_X + 4.2, 1.1, '%.0f deg FOV' % config.KAMERA_FOV, color=AKSEN, fontsize=8)

    # Titik ini yang menjelaskan kenapa S3 tidak bisa dijalankan dengan rig ini.
    x_masuk = KAM_X + config.LANE_WIDTH
    b.plot([x_masuk], [-config.LANE_WIDTH], marker='o', ms=6, mfc='white',
           mec=AKSEN, mew=1.4)
    b.annotate('a vehicle in the next lane enters\nthe frame only past this point',
               (x_masuk, -config.LANE_WIDTH), xytext=(x_masuk + 1.2, -6.4),
               fontsize=8, color=AKSEN,
               arrowprops=dict(arrowstyle='->', color=AKSEN, lw=.9))
    b.text(-1.1, config.LANE_WIDTH * 0.72, 'ego lane', fontsize=8, color=REDUP)
    b.text(-1.1, -config.LANE_WIDTH * 1.28, 'overtaking lane', fontsize=8, color=REDUP)
    b.set_xlim(-2.5, 13.5); b.set_ylim(-8.2, 6.4); b.set_aspect('equal')
    b.set_title('Top view (to scale) — first 12 m of the field of view')
    b.axis('off')

    fig.suptitle('Sensor configuration — one forward RGB camera with co-located depth', y=1.04)
    fig.text(0.5, -0.02,
             'RGB %dx%d @ 20 Hz, %.0f deg FOV   |   depth co-located, ideal sensor   |   '
             'stereo baseline %.2f m available but unused   |   '
             'measured range 45.6 m, overtake trigger at ~32 m'
             % (config.KAMERA_LEBAR, config.KAMERA_TINGGI, config.KAMERA_FOV,
                config.KAMERA_BASELINE),
             ha='center', fontsize=8, color=REDUP)
    simpan(fig, 'sensor_rig.png')


def frames():
    """Tiga titik acuan pada satu kendaraan, dan rantai konversi frame."""
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.4))

    a = ax[0]
    a.add_patch(Rectangle((-SUMBU, -config.EGO_LEBAR / 2), config.EGO_PANJANG,
                          config.EGO_LEBAR, fc='none', ec=REDUP, lw=1.2))
    # Pusat bodi (1,433) dan kamera (1,68) hanya berjarak 0,25 m, jadi labelnya
    # disebar ke arah berbeda -- ditumpuk vertikal, keduanya saling menutupi.
    titik = [(0.0, 'rear axle', 'MPC state,\nplanner reference', (-2.6, 2.4), 'right'),
             (SUMBU, 'body centre', 'safety zone,\nobstacle anchor', (0.1, 3.1), 'center'),
             (KAM_X, 'camera', 'perception origin', (3.4, 2.0), 'left')]
    for x, nama, guna, (tx, ty), ha in titik:
        a.plot([x], [0], marker='o', ms=7, mfc='white', mec=AKSEN, mew=1.5, zorder=3)
        a.annotate('%s\n%s' % (nama, guna), (x, 0.35), xytext=(tx, ty), ha=ha, fontsize=8,
                   arrowprops=dict(arrowstyle='->', color=AKSEN, lw=.9,
                                   connectionstyle='arc3,rad=0.15'))
    a.annotate('', (0, -1.55), (SUMBU, -1.55),
               arrowprops=dict(arrowstyle='<->', color=AKSEN, lw=1.2))
    a.text(SUMBU / 2, -2.25, '%.3f m' % SUMBU, ha='center', fontsize=9, color=AKSEN)
    a.text(SUMBU / 2, -3.1, 'mixing these two shifted every obstacle\nby this much - '
                            'three times in this project', ha='center', fontsize=8, color=AKSEN)
    a.set_xlim(-3.4, 6.2); a.set_ylim(-4.2, 4.6); a.set_aspect('equal'); a.axis('off')
    a.set_title('Three reference points on one vehicle')

    b = ax[1]
    b.axis('off'); b.set_xlim(0, 1); b.set_ylim(0, 1)
    tahap = [('CARLA world', 'left-handed, y points right\nactor origin = body centre'),
             ('Right-handed', 'y -> -y,  yaw -> -yaw\norigin moved to rear axle'),
             ('Road frame', 'rotated by road heading\nx = along road, y = lateral')]
    for i, (nama, isi) in enumerate(tahap):
        atas = 0.92 - i * 0.30
        b.add_patch(Rectangle((0.04, atas - 0.20), 0.92, 0.20, fc='none', ec=GARIS, lw=1.0,
                              transform=b.transAxes))
        b.text(0.08, atas - 0.06, nama, fontsize=9.5, fontweight='semibold',
               transform=b.transAxes)
        b.text(0.08, atas - 0.165, isi, fontsize=8, color=REDUP, transform=b.transAxes)
        if i < 2:                                    # panah MENURUN, searah bacaan
            b.annotate('', (0.5, atas - 0.30), (0.5, atas - 0.21),
                       xycoords=b.transAxes, textcoords=b.transAxes,
                       arrowprops=dict(arrowstyle='->', color=GARIS, lw=1.1))
    b.text(0.5, 0.04, 'localization.py is the only module allowed to convert frames',
           ha='center', fontsize=8, color=REDUP, style='italic', transform=b.transAxes)
    b.set_title('Frame conversion chain')

    fig.suptitle('Localization - ground truth transform, converted once and only here', y=1.0)
    simpan(fig, 'localization_frames.png')


def pipeline():
    """Rantai end-to-end berikut lajunya."""
    fig, ax = plt.subplots(figsize=(11, 3.6))
    ax.set_xlim(0, 100); ax.set_ylim(0, 34); ax.axis('off')

    blok = [('Camera\nRGB + depth', 20, 'sensors.py'),
            ('YOLOPX\n+ tracking', 20, 'perception.py\ntracking.py'),
            ('Localization', 20, 'localization.py'),
            ('Behavior FSM', 10, 'planning.py'),
            ('Local planner\n9 candidates', 10, 'planning.py'),
            ('MPC\nCasADi + IPOPT', 20, 'control.py'),
            ('Throttle PI\n+ steer map', 20, 'control.py')]
    lebar, celah = 11.6, 2.0
    for i, (nama, hz, modul) in enumerate(blok):
        x = i * (lebar + celah)
        warna = AKSEN if hz == 10 else GARIS
        ax.add_patch(Rectangle((x, 14), lebar, 11, fc='none', ec=warna, lw=1.1))
        ax.text(x + lebar / 2, 21.0, nama, ha='center', va='center', fontsize=8.5)
        ax.text(x + lebar / 2, 16.2, f'{hz} Hz', ha='center', fontsize=8, color=warna)
        ax.text(x + lebar / 2, 11.6, modul, ha='center', va='top', fontsize=7, color=REDUP)
        if i < len(blok) - 1:
            ax.annotate('', (x + lebar + celah, 19.5), (x + lebar, 19.5),
                        arrowprops=dict(arrowstyle='->', color=GARIS, lw=1.0))

    ax.annotate('', (0.5, 29.5), (95.0, 29.5), arrowprops=dict(arrowstyle='<-', color=REDUP, lw=.9))
    ax.text(48, 30.4, 'CARLA synchronous tick — every command goes through simulation.tick',
            ha='center', fontsize=8, color=REDUP)
    # Batas frame ada SETELAH localization, bukan di tengah FSM.
    batas = 2 * (lebar + celah) + lebar + celah / 2
    ax.annotate('', (batas, 7.8), (batas, 13.6),
                arrowprops=dict(arrowstyle='-', color=REDUP, lw=.8, ls=':'))
    ax.text(batas - 2, 6.6, 'ego frame, relative to ego', ha='right', fontsize=8, color=REDUP)
    ax.text(batas + 2, 6.6, 'road frame, absolute', ha='left', fontsize=8, color=REDUP)
    ax.text(batas, 3.0, 'converted once, in localization.halangan_ego_ke_jalan  '
                        '(the FSM then reads gaps back relative to the ego)',
            ha='center', fontsize=8, color=REDUP, style='italic')
    ax.set_title('End-to-end pipeline — one tick is 50 ms', pad=14)
    simpan(fig, 'pipeline.png')


def mpc():
    """Receding horizon. Ramalan yang ditumpuk di satu sumbu saling menutupi,
    jadi horizonnya digambar sebagai batang waktu di panel bawah."""
    fig, ax = plt.subplots(2, 1, figsize=(11, 5.0), sharex=True,
                           gridspec_kw=dict(height_ratios=[2.1, 1]))
    T, dt, N = 4.0, config.MPC_DT, config.MPC_N
    jeda = 0.6                       # jarak antar tick YANG DIGAMBAR, bukan yang nyata

    def quintic(tt):
        u = np.clip(tt / T, 0, 1)
        return -config.LANE_WIDTH * (10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5)

    a = ax[0]
    t = np.linspace(0, T, 200)
    a.plot(t, quintic(t), '--', color=GARIS, lw=1.7, label='planner reference', zorder=3)
    warna_tick = [AKSEN, '#4e7488', REDUP]
    for i in range(3):
        t0 = i * jeda
        th = np.linspace(t0, t0 + N * dt, 60)
        simpang = 0.28 * np.exp(-2.6 * (th - t0))
        a.plot(th, quintic(th) + simpang, color=warna_tick[i], lw=1.3, zorder=2,
               label='2 s prediction, recomputed every tick' if i == 0 else None)
        a.plot([t0], [quintic(t0) + simpang[0]], marker='o', ms=5,
               color=warna_tick[i], zorder=4)
    te = np.linspace(0, config.FIXED_DELTA_SECONDS, 10)
    a.plot(te, quintic(te) + 0.28 * np.exp(-2.6 * te), color=AKSEN, lw=5,
           solid_capstyle='butt', zorder=5, label='actually executed: the first 50 ms')
    a.set_ylabel('lateral position (m)')
    a.set_ylim(-4.1, 0.9)
    a.legend(loc='upper right', fontsize=8)
    a.grid(alpha=.22)
    a.set_title('Model predictive control - predict 2 s, execute 50 ms, discard the rest, repeat')

    b = ax[1]
    for i in range(3):
        t0 = i * jeda
        y = 2 - i
        b.barh(y, N * dt, left=t0, height=.5, color=warna_tick[i], alpha=.2,
               edgecolor=warna_tick[i], lw=1.0)
        b.barh(y, config.FIXED_DELTA_SECONDS, left=t0, height=.5, color=warna_tick[i])
        b.text(t0 - 0.07, y, 'tick %d' % (i + 1), ha='right', va='center',
               fontsize=8, color=warna_tick[i])
    b.text(jeda * 2 + N * dt + 0.08, 1, 'solid = executed\nshaded = predicted, then discarded',
           va='center', fontsize=8, color=REDUP)
    b.set_ylim(-0.4, 2.9); b.set_yticks([])
    b.set_xlabel('time (s)')
    b.spines[['left', 'right', 'top']].set_visible(False)
    b.grid(axis='x', alpha=.22)
    fig.text(0.5, -0.02, 'Ticks are drawn %.1f s apart so the horizons stay legible; '
                         'the real spacing is %.0f ms.' % (jeda, config.FIXED_DELTA_SECONDS * 1e3),
             ha='center', fontsize=8, color=REDUP, style='italic')
    fig.tight_layout()
    simpan(fig, 'mpc_concept.png')


def scenario():
    """Susunan skenario S1 berikut parameter kedua kendaraan."""
    fig, ax = plt.subplots(2, 1, figsize=(11, 5.2),
                           gridspec_kw=dict(height_ratios=[1.35, 1]))

    a = ax[0]
    for sgn in (0.5, -0.5, -1.5):
        a.axhline(sgn * config.LANE_WIDTH, color=REDUP, lw=.8,
                  ls=':' if sgn == -0.5 else '-')
    a.text(-12.5, 0, 'ego lane', fontsize=8, color=REDUP, va='center')
    a.text(-12.5, -config.LANE_WIDTH, 'overtaking\nlane', fontsize=8, color=REDUP, va='center')

    gap, v_ego, v_tgt = config.SKENARIO['S1'][0][0], config.EGO_V0, config.SKENARIO['S1'][0][2]
    for x, pj, lb, warna, nama, ve in (
            (0.0, P['length'], P['width'], AKSEN, 'ego', v_ego),
            (gap, config.LAIN_PANJANG, config.LAIN_LEBAR, GARIS, 'target', v_tgt)):
        a.add_patch(Rectangle((x - pj / 2, -lb / 2), pj, lb, fc='none', ec=warna, lw=1.3))
        a.annotate('', (x + pj / 2 + 7, 0), (x + pj / 2 + 1, 0),
                   arrowprops=dict(arrowstyle='->', color=warna, lw=1.2))
        # Nama dan kecepatan DI ATAS kendaraannya: ditaruh di bawah, keduanya
        # jatuh ke lajur menyalip dan terbaca seolah milik lajur itu.
        a.text(x, lb / 2 + 0.35, '%s  -  %.1f m/s (%.1f km/h)' % (nama, ve, ve * 3.6),
               ha='center', va='bottom', fontsize=8.5, color=warna)

    a.annotate('', (P['length'] / 2, -config.LANE_WIDTH),
               (gap - config.LAIN_PANJANG / 2, -config.LANE_WIDTH),
               arrowprops=dict(arrowstyle='<->', color=GARIS, lw=1.0))
    a.text(gap / 2, -config.LANE_WIDTH + 0.35, 'initial gap %.0f m (body to body)' % gap,
           ha='center', fontsize=8)
    a.text(gap / 2, -config.LANE_WIDTH - 1.5,
           'closing at %.1f m/s  ->  overtaking lane must be cleared before the gap runs out'
           % (v_ego - v_tgt), ha='center', fontsize=8, color=REDUP)
    a.set_xlim(-13, 74); a.set_ylim(-7.4, 3.6); a.axis('off')
    a.set_title('Scenario S1 - flying overtaking, target lane empty')

    b = ax[1]
    b.axis('off'); b.set_xlim(0, 1); b.set_ylim(0, 1)
    isi = [['Model', 'Dodge Charger 2020', 'Nissan Patrol'],
           ['Length x width', '%.3f x %.3f m' % (P['length'], P['width']),
            '%.3f x %.3f m' % (config.LAIN_PANJANG, config.LAIN_LEBAR)],
           ['Wheelbase', '%.3f m' % P['L'], 'not modelled'],
           ['Mass', '%.0f kg' % P['mass'], 'not modelled'],
           ['Speed', '%.1f m/s  (%.1f km/h)' % (v_ego, v_ego * 3.6),
            '%.1f m/s  (%.1f km/h)' % (v_tgt, v_tgt * 3.6)],
           ['Control', 'MPC, closed loop', 'speed forced along the road'],
           ['Start', 'spawn point 75, Town04', '%.0f m ahead, same lane' % gap]]
    x = [0.01, 0.22, 0.60]
    for j, k in enumerate(['', 'Ego', 'Target']):
        b.text(x[j], 0.95, k, fontsize=9, fontweight='semibold', transform=b.transAxes)
    b.plot([0.01, 0.99], [0.90, 0.90], color=GARIS, lw=.9, transform=b.transAxes)
    for i, baris in enumerate(isi):
        y = 0.79 - i * 0.118
        for j, sel in enumerate(baris):
            b.text(x[j], y, sel, fontsize=8, color=REDUP if j == 0 else GARIS,
                   transform=b.transAxes)
    b.text(0.01, -0.09, 'Road: straight 400 m section, lane width %.2f m. Target dimensions are '
                        'assumed fixed - perception never measures them.' % config.LANE_WIDTH,
           fontsize=8, color=REDUP, style='italic', transform=b.transAxes)
    fig.tight_layout()
    simpan(fig, 'scenario_s1.png')


SEMUA = dict(rig=rig, frames=frames, scenario=scenario, pipeline=pipeline, mpc=mpc)

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', choices=list(SEMUA))
    a = ap.parse_args()
    for nama, fn in ([(a.only, SEMUA[a.only])] if a.only else SEMUA.items()):
        fn()
