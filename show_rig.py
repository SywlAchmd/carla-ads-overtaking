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

PHOTO = (1600, 1100)
INK, ACCENT, GREEN, BLUE = '#22303f', '#c0392b', '#1e7a46', '#1f4e9c'

# Diukur dari physics control CARLA, bukan diasumsikan (lihat NOTES.md).
FRONT_WHEEL_X, REAR_WHEEL_X = 1.611, -1.433
TRACK_FRONT, TRACK_REAR = 1.624, 1.654
WHEEL_R, WHEEL_W = 0.350, 0.25
BODY_HEIGHT = 1.535
CAM_X = config.CAMERA_AHEAD_OF_AXLE + REAR_WHEEL_X      # thd titik asal aktor


def _K():
    f = PHOTO[0] / (2.0 * math.tan(math.radians(90.0) / 2.0))
    return np.array([[f, 0, PHOTO[0] / 2], [0, f, PHOTO[1] / 2], [0, 0, 1.0]])


def _pixels(camera, K, world_points):
    """Titik dunia CARLA -> piksel. `world_points` iterable of (x, y, z)."""
    w2c = np.array(camera.get_transform().get_inverse_matrix())
    out = []
    for x, y, z in world_points:
        p = w2c @ np.array([x, y, z, 1.0])
        p = np.array([p[1], -p[2], p[0]])            # sumbu UE -> sumbu kamera baku
        if p[2] < 0.1:
            out.append((np.nan, np.nan))
            continue
        uv = K @ p
        out.append((uv[0] / uv[2], uv[1] / uv[2]))
    return np.array(out)


def _world(ego, local):
    """Titik di frame kendaraan (x maju, y kanan, z naik) -> dunia CARLA."""
    tf = ego.get_transform()
    yaw = math.radians(tf.rotation.yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    return [(tf.location.x + c * lx - s * ly, tf.location.y + s * lx + c * ly,
             tf.location.z + lz) for lx, ly, lz in local]


def photo():
    with simulation.carla_world() as world:
        with simulation.ego_vehicle(world) as ego:
            bp = world.get_blueprint_library().find('sensor.camera.rgb')
            bp.set_attribute('image_size_x', str(PHOTO[0]))
            bp.set_attribute('image_size_y', str(PHOTO[1]))
            bp.set_attribute('fov', '90')
            # Sudut tiga-perempat depan-kiri, sedikit di atas atap -- sama dengan
            # cara KITTI memotret rig-nya, supaya atap dan sisi sama-sama terbaca.
            d = carla.Location(x=4.9, y=-3.7, z=2.35)
            direction = carla.Rotation(
                yaw=math.degrees(math.atan2(-d.y, -d.x)),
                pitch=math.degrees(math.atan2(0.9 - d.z, math.hypot(d.x, d.y))))
            cam = world.spawn_actor(bp, carla.Transform(d, direction), attach_to=ego)
            q = queue.Queue()
            cam.listen(q.put)
            try:
                for _ in range(12):
                    simulation.tick(world)
                    image = q.get(timeout=5.0)
                rgb = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
                    PHOTO[1], PHOTO[0], 4)[:, :, :3][:, :, ::-1].copy()
                K = _K()
                sensor = _world(ego, [(CAM_X, 0, config.CAMERA_Z)])
                uv_s = _pixels(cam, K, sensor)[0]
                # Sumbu kamera sepanjang 1,2 m: x maju, y kiri, z naik (frame RH)
                length = 1.2
                tip = _pixels(cam, K, _world(ego, [
                    (CAM_X + length, 0, config.CAMERA_Z),
                    (CAM_X, -length, config.CAMERA_Z),
                    (CAM_X, 0, config.CAMERA_Z + length)]))
                # State MPC ada di TENGAH sumbu belakang (y = 0), bukan di roda.
                # Versi pertama menaruhnya di pusat roda kiri supaya kelihatan,
                # dan itu menyesatkan: titiknya bergeser 0,83 m ke samping.
                axle = _pixels(cam, K, _world(ego, [
                    (REAR_WHEEL_X, -TRACK_REAR / 2, WHEEL_R),
                    (REAR_WHEEL_X, 0.0, WHEEL_R),
                    (REAR_WHEEL_X, TRACK_REAR / 2, WHEEL_R)]))
                rear_axle = axle[1]
            finally:
                cam.stop(); cam.destroy()

    fig, ax = plt.subplots(figsize=(11, 7.6))
    ax.imshow(rgb); ax.axis('off')
    for (ux, uy), color, name in zip(tip, (ACCENT, GREEN, BLUE), ('x', 'y', 'z')):
        ax.annotate('', (ux, uy), uv_s, arrowprops=dict(arrowstyle='->', color=color, lw=2.2))
        ax.text(ux, uy - 12, name, color=color, fontsize=13, fontweight='bold', ha='center',
                zorder=8,
                bbox=dict(boxstyle='square,pad=0.15', fc='white', ec='none', alpha=.85))
    ax.plot(*uv_s, marker='o', ms=9, mfc='none', mec=ACCENT, mew=2.4)
    ax.annotate('RGB camera + depth camera\nco-located, 1.65 m above the road',
                uv_s, xytext=(uv_s[0] - 500, uv_s[1] - 235), color=ACCENT, fontsize=12,
                arrowprops=dict(arrowstyle='->', color=ACCENT, lw=1.6),
                bbox=dict(boxstyle='round,pad=0.42', fc='white', ec=ACCENT, lw=1.0, alpha=.92))
    # Ditaruh jauh ke kiri-bawah: di dekat markernya, teks ini jatuh di atas bodi.
    ax.plot(axle[:, 0], axle[:, 1], ls='--', lw=1.6, color=INK, zorder=4)
    ax.plot(*rear_axle, marker='o', ms=9, mfc='white', mec=INK, mew=2.0, zorder=5)
    ax.annotate('centre of the rear axle\nMPC state origin, planner reference', rear_axle,
                xytext=(rear_axle[0] + 150, rear_axle[1] + 210),
                color=INK, fontsize=11,
                arrowprops=dict(arrowstyle='->', color=INK, lw=1.5),
                bbox=dict(boxstyle='round,pad=0.42', fc='white', ec=INK, lw=1.0, alpha=.92))
    ax.text(0.5, -0.035, 'Axes are the right-handed frame used throughout: '
                         'x forward, y left, z up. CARLA\'s own frame is left-handed.',
            transform=ax.transAxes, ha='center', fontsize=9, color='0.4', style='italic')
    ax.set_title('Sensor configuration on the ego vehicle (Dodge Charger 2020, CARLA)',
                 fontsize=12, pad=10)
    fig.tight_layout()
    fig.savefig(f'{config.OUT_DIR}/sensor_rig_photo.png', dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Photo   : {config.OUT_DIR}/sensor_rig_photo.png')


def top_view():
    fig, ax = plt.subplots(figsize=(11, 5.4))
    ax.set_aspect('equal'); ax.axis('off')
    pj, lb = config.EGO_LENGTH, config.EGO_WIDTH

    ax.add_patch(Rectangle((-pj / 2, -lb / 2), pj, lb, fc='0.94', ec='0.75',
                           lw=1.4, zorder=1, joinstyle='round'))
    for wx, tr in ((FRONT_WHEEL_X, TRACK_FRONT), (REAR_WHEEL_X, TRACK_REAR)):
        for sy in (-1, 1):
            ax.add_patch(Rectangle((wx - WHEEL_R, sy * tr / 2 - WHEEL_W / 2),
                                   2 * WHEEL_R, WHEEL_W, fc='0.15', ec='none', zorder=3))
    ax.add_patch(Rectangle((CAM_X - 0.16, -0.30), 0.32, 0.60, fc=ACCENT, ec='none',
                           alpha=.85, zorder=4))
    ax.annotate('RGB camera (1280x720, 90 deg FOV)\ndepth camera, co-located',
                (CAM_X + 0.16, 0.28), xytext=(CAM_X + 1.0, 1.55), fontsize=9, color=ACCENT,
                arrowprops=dict(arrowstyle='->', color=ACCENT, lw=1.1))
    ax.text(CAM_X, -0.52, 'height: 1.65 m', ha='center', fontsize=8.5, color=GREEN)

    def measure(x0, x1, y, text, color=INK):
        ax.annotate('', (x0, y), (x1, y), arrowprops=dict(arrowstyle='<->', color=color, lw=.9))
        ax.text((x0 + x1) / 2, y + 0.07, text, ha='center', fontsize=8.5, color=color)

    for x in (REAR_WHEEL_X, CAM_X, FRONT_WHEEL_X):
        ax.plot([x, x], [-lb / 2 - 0.18, -1.62], ls=':', lw=.8, color='0.55', zorder=0)
    measure(REAR_WHEEL_X, CAM_X, -1.28, '1.68 m')
    measure(REAR_WHEEL_X, FRONT_WHEEL_X, -1.58, '3.044 m  (wheelbase)')
    measure(-pj / 2, pj / 2, -2.02, '5.008 m')
    ax.annotate('', (pj / 2 + 0.34, -lb / 2), (pj / 2 + 0.34, lb / 2),
                arrowprops=dict(arrowstyle='<->', color=INK, lw=.9))
    ax.text(pj / 2 + 0.44, -0.55, '1.882 m', va='center', ha='left',
            fontsize=8.5, color=INK)
    ax.plot([REAR_WHEEL_X], [0], marker='o', ms=7, mfc='white', mec=INK, mew=1.5, zorder=5)
    ax.text(REAR_WHEEL_X, 0.30, 'rear axle', ha='center', fontsize=8.5, color=INK)
    ax.text(FRONT_WHEEL_X, lb / 2 + 0.16, 'wheel radius 0.350 m', ha='center',
            fontsize=8, color='0.45')

    ax.text(0, 2.22, 'All heights are measured from the road surface',
            ha='center', fontsize=9, color=GREEN,
            bbox=dict(boxstyle='round,pad=0.35', fc='white', ec=GREEN, lw=.9))
    # Roda depan ada di x = +1,611, jadi arah maju ke KANAN. Panah yang menunjuk
    # ke kiri sempat tergambar dan membuat seluruh skema terbaca terbalik.
    ax.annotate('', (pj / 2 + 1.30, 0), (pj / 2 + 0.80, 0),
                arrowprops=dict(arrowstyle='->', color='0.45', lw=1.2))
    ax.text(pj / 2 + 1.05, 0.20, 'forward', ha='center', fontsize=8.5, color='0.45')
    ax.text(REAR_WHEEL_X, lb / 2 + 0.16, 'rear', ha='center', fontsize=8, color='0.45')
    ax.set_xlim(-4.0, 5.4); ax.set_ylim(-2.5, 2.7)
    ax.set_title('Top view, to scale - one forward camera pair, no LiDAR, no IMU/GNSS',
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(f'{config.OUT_DIR}/sensor_rig_topdown.png', dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Top view: {config.OUT_DIR}/sensor_rig_topdown.png')


def top_view_photo():
    """Tampak atas dari render CARLA, bukan skema. FOV sempit dari ketinggian
    supaya mendekati ortografik: 20 derajat dari 22 m memberi parallax ~7 %
    antara atap dan permukaan jalan."""
    width, height, fov, h = 1500, 1100, 20.0, 22.0
    with simulation.carla_world() as world:
        with simulation.ego_vehicle(world) as ego:
            bp = world.get_blueprint_library().find('sensor.camera.rgb')
            bp.set_attribute('image_size_x', str(width))
            bp.set_attribute('image_size_y', str(height))
            bp.set_attribute('fov', str(fov))
            cam = world.spawn_actor(
                bp, carla.Transform(carla.Location(x=0, y=0, z=h),
                                    carla.Rotation(pitch=-90, yaw=-90)), attach_to=ego)
            q = queue.Queue()
            cam.listen(q.put)
            try:
                for _ in range(12):
                    simulation.tick(world)
                    image = q.get(timeout=5.0)
                rgb = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
                    height, width, 4)[:, :, :3][:, :, ::-1].copy()
                f = width / (2.0 * math.tan(math.radians(fov) / 2.0))
                K = np.array([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1.0]])
                points = _pixels(cam, K, _world(ego, [
                    (REAR_WHEEL_X, 0, WHEEL_R),                   # 0 tengah sumbu belakang
                    (CAM_X, 0, config.CAMERA_Z),                    # 1 sensor
                    (FRONT_WHEEL_X, 0, WHEEL_R),                      # 2 tengah sumbu depan
                    (REAR_WHEEL_X, -TRACK_REAR / 2, WHEEL_R),  # 3 roda belakang kiri
                    (REAR_WHEEL_X, TRACK_REAR / 2, WHEEL_R),   # 4 roda belakang kanan
                    (config.EGO_LENGTH / 2, 0, 0),                 # 5 ujung depan
                    (-config.EGO_LENGTH / 2, 0, 0)]))              # 6 ujung belakang
            finally:
                cam.stop(); cam.destroy()

    fig, ax = plt.subplots(figsize=(11, 8))
    ax.imshow(rgb); ax.axis('off')
    (p_ax, p_sen, p_front_axle, p_left, p_right, p_front, p_rear) = points

    ax.plot([p_left[0], p_right[0]], [p_left[1], p_right[1]], ls='--', lw=1.6, color=INK)
    ax.plot(*p_ax, marker='o', ms=10, mfc='white', mec=INK, mew=2.2, zorder=5)
    # Di bawah marker, label ini jatuh di atas bodi dan di antara garis dimensi;
    # ditaruh di atas, berseberangan dengan label kamera.
    ax.annotate('centre of the rear axle\nMPC state origin', p_ax,
                xytext=(p_ax[0] - 60, p_ax[1] - 215), ha='center', color=INK, fontsize=11,
                arrowprops=dict(arrowstyle='->', color=INK, lw=1.5),
                bbox=dict(boxstyle='round,pad=0.42', fc='white', ec=INK, lw=1.0, alpha=.92))
    ax.plot(*p_sen, marker='s', ms=11, mfc=ACCENT, mec='white', mew=1.4, zorder=5)
    ax.annotate('RGB + depth camera\n1.65 m above the road', p_sen,
                xytext=(p_sen[0] + 40, p_sen[1] - 215), ha='center', color=ACCENT, fontsize=11,
                arrowprops=dict(arrowstyle='->', color=ACCENT, lw=1.5),
                bbox=dict(boxstyle='round,pad=0.42', fc='white', ec=ACCENT, lw=1.0, alpha=.92))

    def measure(a, b, dy, text, color=INK):
        ya = (a[1] + b[1]) / 2 + dy
        ax.annotate('', (a[0], ya), (b[0], ya),
                    arrowprops=dict(arrowstyle='<->', color=color, lw=1.3))
        for px in (a[0], b[0]):
            ax.plot([px, px], [(a[1] + b[1]) / 2, ya], ls=':', lw=.9, color=color)
        ax.text((a[0] + b[0]) / 2, ya - 12, text, ha='center', fontsize=10, color=color)

    measure(p_ax, p_sen, 300, '1.68 m')
    measure(p_ax, p_front_axle, 370, '3.044 m  (wheelbase)')
    measure(p_rear, p_front, 440, '5.008 m')
    ax.annotate('', (p_front[0] + 95, p_ax[1]), (p_front[0] + 25, p_ax[1]),
                arrowprops=dict(arrowstyle='->', color='0.35', lw=1.6))
    ax.text(p_front[0] + 60, p_ax[1] - 18, 'forward', ha='center', fontsize=10, color='0.35')
    ax.set_title('Top view rendered in CARLA - one forward camera pair, '
                 'no LiDAR, no IMU/GNSS', fontsize=12, pad=10)
    fig.tight_layout()
    fig.savefig(f'{config.OUT_DIR}/sensor_rig_topdown_render.png', dpi=140, bbox_inches='tight')
    plt.close(fig)
    print(f'Top render: {config.OUT_DIR}/sensor_rig_topdown_render.png')


if __name__ == '__main__':
    top_view()
    photo()
    top_view_photo()
