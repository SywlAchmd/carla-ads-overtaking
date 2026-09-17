"""Gambar konfigurasi sensor ala KITTI: foto perspektif + skema tampak atas.

Dua gambar, meniru cara KITTI menampilkan rig-nya (Geiger dkk., 2013, Gambar 2):

    out/sensor_rig_photo.png    foto ego di CARLA, sensor dan sumbunya ditimpakan
    out/sensor_rig_topdown.png  skema berdimensi, semua tinggi thd permukaan jalan

Fotonya diambil dari simulator, bukan digambar: kendaraannya memang Dodge
Charger yang dipakai eksperimen, jadi pembaca melihat rig yang sesungguhnya.

    python show_rig.py            (butuh server CARLA)
"""
import math
import queue

import carla
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Rectangle

import config
import simulation

FOTO = (1600, 1100)
GARIS, AKSEN, HIJAU, BIRU = '#22303f', '#c0392b', '#1e7a46', '#1f4e9c'

# Diukur dari physics control CARLA, bukan diasumsikan (lihat NOTES.md).
RODA_DEPAN_X, RODA_BELAKANG_X = 1.611, -1.433
TRACK_DEPAN, TRACK_BELAKANG = 1.624, 1.654
RODA_R, RODA_W = 0.350, 0.25
TINGGI_BODI = 1.535
KAM_X = config.KAMERA_DEPAN_SUMBU + RODA_BELAKANG_X      # thd titik asal aktor


def _K():
    f = FOTO[0] / (2.0 * math.tan(math.radians(90.0) / 2.0))
    return np.array([[f, 0, FOTO[0] / 2], [0, f, FOTO[1] / 2], [0, 0, 1.0]])


def _piksel(kamera, K, titik_dunia):
    """Titik dunia CARLA -> piksel. `titik_dunia` iterable of (x, y, z)."""
    w2c = np.array(kamera.get_transform().get_inverse_matrix())
    keluar = []
    for x, y, z in titik_dunia:
        p = w2c @ np.array([x, y, z, 1.0])
        p = np.array([p[1], -p[2], p[0]])            # sumbu UE -> sumbu kamera baku
        if p[2] < 0.1:
            keluar.append((np.nan, np.nan))
            continue
        uv = K @ p
        keluar.append((uv[0] / uv[2], uv[1] / uv[2]))
    return np.array(keluar)


def _dunia(ego, lokal):
    """Titik di frame kendaraan (x maju, y kanan, z naik) -> dunia CARLA."""
    tf = ego.get_transform()
    yaw = math.radians(tf.rotation.yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    return [(tf.location.x + c * lx - s * ly, tf.location.y + s * lx + c * ly,
             tf.location.z + lz) for lx, ly, lz in lokal]


def foto():
    with simulation.carla_world() as world:
        with simulation.ego_vehicle(world) as ego:
            bp = world.get_blueprint_library().find('sensor.camera.rgb')
            bp.set_attribute('image_size_x', str(FOTO[0]))
            bp.set_attribute('image_size_y', str(FOTO[1]))
            bp.set_attribute('fov', '90')
            # Sudut tiga-perempat depan-kiri, sedikit di atas atap -- sama dengan
            # cara KITTI memotret rig-nya, supaya atap dan sisi sama-sama terbaca.
            d = carla.Location(x=4.9, y=-3.7, z=2.35)
            arah = carla.Rotation(
                yaw=math.degrees(math.atan2(-d.y, -d.x)),
                pitch=math.degrees(math.atan2(0.9 - d.z, math.hypot(d.x, d.y))))
            cam = world.spawn_actor(bp, carla.Transform(d, arah), attach_to=ego)
            q = queue.Queue()
            cam.listen(q.put)
            try:
                for _ in range(12):
                    simulation.tick(world)
                    citra = q.get(timeout=5.0)
                rgb = np.frombuffer(citra.raw_data, dtype=np.uint8).reshape(
                    FOTO[1], FOTO[0], 4)[:, :, :3][:, :, ::-1].copy()
                K = _K()
                sensor = _dunia(ego, [(KAM_X, 0, config.KAMERA_Z)])
                uv_s = _piksel(cam, K, sensor)[0]
                # Sumbu kamera sepanjang 1,2 m: x maju, y kiri, z naik (frame RH)
                panjang = 1.2
                ujung = _piksel(cam, K, _dunia(ego, [
                    (KAM_X + panjang, 0, config.KAMERA_Z),
                    (KAM_X, -panjang, config.KAMERA_Z),
                    (KAM_X, 0, config.KAMERA_Z + panjang)]))
                # State MPC ada di TENGAH sumbu belakang (y = 0), bukan di roda.
                # Versi pertama menaruhnya di pusat roda kiri supaya kelihatan,
                # dan itu menyesatkan: titiknya bergeser 0,83 m ke samping.
                axle = _piksel(cam, K, _dunia(ego, [
                    (RODA_BELAKANG_X, -TRACK_BELAKANG / 2, RODA_R),
                    (RODA_BELAKANG_X, 0.0, RODA_R),
                    (RODA_BELAKANG_X, TRACK_BELAKANG / 2, RODA_R)]))
                sumbu_belakang = axle[1]
            finally:
                cam.stop(); cam.destroy()

    fig, ax = plt.subplots(figsize=(11, 7.6))
    ax.imshow(rgb); ax.axis('off')
    for (ux, uy), warna, nama in zip(ujung, (AKSEN, HIJAU, BIRU), ('x', 'y', 'z')):
        ax.annotate('', (ux, uy), uv_s, arrowprops=dict(arrowstyle='->', color=warna, lw=2.2))
        ax.text(ux, uy - 12, nama, color=warna, fontsize=13, fontweight='bold', ha='center',
                zorder=8,
                bbox=dict(boxstyle='square,pad=0.15', fc='white', ec='none', alpha=.85))
    ax.plot(*uv_s, marker='o', ms=9, mfc='none', mec=AKSEN, mew=2.4)
    ax.annotate('RGB camera + depth camera\nco-located, 1.65 m above the road',
                uv_s, xytext=(uv_s[0] - 500, uv_s[1] - 235), color=AKSEN, fontsize=12,
                arrowprops=dict(arrowstyle='->', color=AKSEN, lw=1.6),
                bbox=dict(boxstyle='round,pad=0.42', fc='white', ec=AKSEN, lw=1.0, alpha=.92))
    # Ditaruh jauh ke kiri-bawah: di dekat markernya, teks ini jatuh di atas bodi.
    ax.plot(axle[:, 0], axle[:, 1], ls='--', lw=1.6, color=GARIS, zorder=4)
    ax.plot(*sumbu_belakang, marker='o', ms=9, mfc='white', mec=GARIS, mew=2.0, zorder=5)
    ax.annotate('centre of the rear axle\nMPC state origin, planner reference', sumbu_belakang,
                xytext=(sumbu_belakang[0] + 150, sumbu_belakang[1] + 210),
                color=GARIS, fontsize=11,
                arrowprops=dict(arrowstyle='->', color=GARIS, lw=1.5),
                bbox=dict(boxstyle='round,pad=0.42', fc='white', ec=GARIS, lw=1.0, alpha=.92))
    ax.text(0.5, -0.035, 'Axes are the right-handed frame used throughout: '
                         'x forward, y left, z up. CARLA\'s own frame is left-handed.',
            transform=ax.transAxes, ha='center', fontsize=9, color='0.4', style='italic')
    ax.set_title('Sensor configuration on the ego vehicle (Dodge Charger 2020, CARLA)',
                 fontsize=12, pad=10)
    fig.tight_layout()
    fig.savefig(f'{config.OUT_DIR}/sensor_rig_photo.png', dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Photo   : {config.OUT_DIR}/sensor_rig_photo.png')


def tampak_atas():
    fig, ax = plt.subplots(figsize=(11, 5.4))
    ax.set_aspect('equal'); ax.axis('off')
    pj, lb = config.EGO_PANJANG, config.EGO_LEBAR

    ax.add_patch(Rectangle((-pj / 2, -lb / 2), pj, lb, fc='0.94', ec='0.75',
                           lw=1.4, zorder=1, joinstyle='round'))
    for wx, tr in ((RODA_DEPAN_X, TRACK_DEPAN), (RODA_BELAKANG_X, TRACK_BELAKANG)):
        for sy in (-1, 1):
            ax.add_patch(Rectangle((wx - RODA_R, sy * tr / 2 - RODA_W / 2),
                                   2 * RODA_R, RODA_W, fc='0.15', ec='none', zorder=3))
    ax.add_patch(Rectangle((KAM_X - 0.16, -0.30), 0.32, 0.60, fc=AKSEN, ec='none',
                           alpha=.85, zorder=4))
    ax.annotate('RGB camera (1280x720, 90 deg FOV)\ndepth camera, co-located',
                (KAM_X + 0.16, 0.28), xytext=(KAM_X + 1.0, 1.55), fontsize=9, color=AKSEN,
                arrowprops=dict(arrowstyle='->', color=AKSEN, lw=1.1))
    ax.text(KAM_X, -0.52, 'height: 1.65 m', ha='center', fontsize=8.5, color=HIJAU)

    def ukur(x0, x1, y, teks, warna=GARIS):
        ax.annotate('', (x0, y), (x1, y), arrowprops=dict(arrowstyle='<->', color=warna, lw=.9))
        ax.text((x0 + x1) / 2, y + 0.07, teks, ha='center', fontsize=8.5, color=warna)

    for x in (RODA_BELAKANG_X, KAM_X, RODA_DEPAN_X):
        ax.plot([x, x], [-lb / 2 - 0.18, -1.62], ls=':', lw=.8, color='0.55', zorder=0)
    ukur(RODA_BELAKANG_X, KAM_X, -1.28, '1.68 m')
    ukur(RODA_BELAKANG_X, RODA_DEPAN_X, -1.58, '3.044 m  (wheelbase)')
    ukur(-pj / 2, pj / 2, -2.02, '5.008 m')
    ax.annotate('', (pj / 2 + 0.34, -lb / 2), (pj / 2 + 0.34, lb / 2),
                arrowprops=dict(arrowstyle='<->', color=GARIS, lw=.9))
    ax.text(pj / 2 + 0.44, -0.55, '1.882 m', va='center', ha='left',
            fontsize=8.5, color=GARIS)
    ax.plot([RODA_BELAKANG_X], [0], marker='o', ms=7, mfc='white', mec=GARIS, mew=1.5, zorder=5)
    ax.text(RODA_BELAKANG_X, 0.30, 'rear axle', ha='center', fontsize=8.5, color=GARIS)
    ax.text(RODA_DEPAN_X, lb / 2 + 0.16, 'wheel radius 0.350 m', ha='center',
            fontsize=8, color='0.45')

    ax.text(0, 2.22, 'All heights are measured from the road surface',
            ha='center', fontsize=9, color=HIJAU,
            bbox=dict(boxstyle='round,pad=0.35', fc='white', ec=HIJAU, lw=.9))
    # Roda depan ada di x = +1,611, jadi arah maju ke KANAN. Panah yang menunjuk
    # ke kiri sempat tergambar dan membuat seluruh skema terbaca terbalik.
    ax.annotate('', (pj / 2 + 1.30, 0), (pj / 2 + 0.80, 0),
                arrowprops=dict(arrowstyle='->', color='0.45', lw=1.2))
    ax.text(pj / 2 + 1.05, 0.20, 'forward', ha='center', fontsize=8.5, color='0.45')
    ax.text(RODA_BELAKANG_X, lb / 2 + 0.16, 'rear', ha='center', fontsize=8, color='0.45')
    ax.set_xlim(-4.0, 5.4); ax.set_ylim(-2.5, 2.7)
    ax.set_title('Top view, to scale - one forward camera pair, no LiDAR, no IMU/GNSS',
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(f'{config.OUT_DIR}/sensor_rig_topdown.png', dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Top view: {config.OUT_DIR}/sensor_rig_topdown.png')


def tampak_atas_foto():
    """Tampak atas dari render CARLA, bukan skema. FOV sempit dari ketinggian
    supaya mendekati ortografik: 20 derajat dari 22 m memberi parallax ~7 %
    antara atap dan permukaan jalan."""
    lebar, tinggi, fov, h = 1500, 1100, 20.0, 22.0
    with simulation.carla_world() as world:
        with simulation.ego_vehicle(world) as ego:
            bp = world.get_blueprint_library().find('sensor.camera.rgb')
            bp.set_attribute('image_size_x', str(lebar))
            bp.set_attribute('image_size_y', str(tinggi))
            bp.set_attribute('fov', str(fov))
            cam = world.spawn_actor(
                bp, carla.Transform(carla.Location(x=0, y=0, z=h),
                                    carla.Rotation(pitch=-90, yaw=-90)), attach_to=ego)
            q = queue.Queue()
            cam.listen(q.put)
            try:
                for _ in range(12):
                    simulation.tick(world)
                    citra = q.get(timeout=5.0)
                rgb = np.frombuffer(citra.raw_data, dtype=np.uint8).reshape(
                    tinggi, lebar, 4)[:, :, :3][:, :, ::-1].copy()
                f = lebar / (2.0 * math.tan(math.radians(fov) / 2.0))
                K = np.array([[f, 0, lebar / 2], [0, f, tinggi / 2], [0, 0, 1.0]])
                titik = _piksel(cam, K, _dunia(ego, [
                    (RODA_BELAKANG_X, 0, RODA_R),                   # 0 tengah sumbu belakang
                    (KAM_X, 0, config.KAMERA_Z),                    # 1 sensor
                    (RODA_DEPAN_X, 0, RODA_R),                      # 2 tengah sumbu depan
                    (RODA_BELAKANG_X, -TRACK_BELAKANG / 2, RODA_R),  # 3 roda belakang kiri
                    (RODA_BELAKANG_X, TRACK_BELAKANG / 2, RODA_R),   # 4 roda belakang kanan
                    (config.EGO_PANJANG / 2, 0, 0),                 # 5 ujung depan
                    (-config.EGO_PANJANG / 2, 0, 0)]))              # 6 ujung belakang
            finally:
                cam.stop(); cam.destroy()

    fig, ax = plt.subplots(figsize=(11, 8))
    ax.imshow(rgb); ax.axis('off')
    (p_ax, p_sen, p_dep, p_kiri, p_kanan, p_depan, p_blk) = titik

    ax.plot([p_kiri[0], p_kanan[0]], [p_kiri[1], p_kanan[1]], ls='--', lw=1.6, color=GARIS)
    ax.plot(*p_ax, marker='o', ms=10, mfc='white', mec=GARIS, mew=2.2, zorder=5)
    # Di bawah marker, label ini jatuh di atas bodi dan di antara garis dimensi;
    # ditaruh di atas, berseberangan dengan label kamera.
    ax.annotate('centre of the rear axle\nMPC state origin', p_ax,
                xytext=(p_ax[0] - 60, p_ax[1] - 215), ha='center', color=GARIS, fontsize=11,
                arrowprops=dict(arrowstyle='->', color=GARIS, lw=1.5),
                bbox=dict(boxstyle='round,pad=0.42', fc='white', ec=GARIS, lw=1.0, alpha=.92))
    ax.plot(*p_sen, marker='s', ms=11, mfc=AKSEN, mec='white', mew=1.4, zorder=5)
    ax.annotate('RGB + depth camera\n1.65 m above the road', p_sen,
                xytext=(p_sen[0] + 40, p_sen[1] - 215), ha='center', color=AKSEN, fontsize=11,
                arrowprops=dict(arrowstyle='->', color=AKSEN, lw=1.5),
                bbox=dict(boxstyle='round,pad=0.42', fc='white', ec=AKSEN, lw=1.0, alpha=.92))

    def ukur(a, b, dy, teks, warna=GARIS):
        ya = (a[1] + b[1]) / 2 + dy
        ax.annotate('', (a[0], ya), (b[0], ya),
                    arrowprops=dict(arrowstyle='<->', color=warna, lw=1.3))
        for px in (a[0], b[0]):
            ax.plot([px, px], [(a[1] + b[1]) / 2, ya], ls=':', lw=.9, color=warna)
        ax.text((a[0] + b[0]) / 2, ya - 12, teks, ha='center', fontsize=10, color=warna)

    ukur(p_ax, p_sen, 300, '1.68 m')
    ukur(p_ax, p_dep, 370, '3.044 m  (wheelbase)')
    ukur(p_blk, p_depan, 440, '5.008 m')
    ax.annotate('', (p_depan[0] + 95, p_ax[1]), (p_depan[0] + 25, p_ax[1]),
                arrowprops=dict(arrowstyle='->', color='0.35', lw=1.6))
    ax.text(p_depan[0] + 60, p_ax[1] - 18, 'forward', ha='center', fontsize=10, color='0.35')
    ax.set_title('Top view rendered in CARLA - one forward camera pair, '
                 'no LiDAR, no IMU/GNSS', fontsize=12, pad=10)
    fig.tight_layout()
    fig.savefig(f'{config.OUT_DIR}/sensor_rig_topdown_render.png', dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Top render: {config.OUT_DIR}/sensor_rig_topdown_render.png')


if __name__ == '__main__':
    tampak_atas()
    foto()
    tampak_atas_foto()
