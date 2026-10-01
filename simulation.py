"""Setup dunia CARLA: koneksi, mode sinkron, spawn ego, reference path."""
import contextlib
import math

import carla
import numpy as np

import config
import localization

_client = None           # diisi carla_world(), dipakai tick()


@contextlib.contextmanager
def carla_world():
    """Mode sinkron dipaksa aktif; setting dunia dikembalikan saat keluar."""
    global _client
    client = carla.Client(config.CARLA_HOST, config.CARLA_PORT)
    client.set_timeout(config.CARLA_TIMEOUT)
    _client = client

    world = client.get_world()
    if not world.get_map().name.endswith(config.TOWN):
        world = client.load_world(config.TOWN)

    original = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = config.FIXED_DELTA_SECONDS
    world.apply_settings(settings)
    world.set_weather(carla.WeatherParameters.ClearNoon)

    try:
        yield world
    finally:
        world.apply_settings(original)


def tick(world, commands=()):
    """Terapkan perintah aktor, BARU tick. Semua perintah aktor wajib lewat sini.

    `apply_control`, `set_target_velocity`, dan `set_transform` dikirim tanpa
    menunggu, sedangkan `world.tick()` menunggu. Server bisa memproses tick
    sebelum perintahnya tiba, sehingga perintah kadang baru berlaku satu frame
    kemudian -- acak, tergantung penjadwalan thread server. Terukur 11 Sep 2026:
    lima run main.py berkonfigurasi identik memberi lima hasil berbeda sejak
    tick pertama, satu gagal lane_departure. Fisika CARLA sendiri deterministik.

    `apply_batch_sync` baru kembali setelah perintah diterapkan, jadi urutannya
    terjamin: tiga run identik bit-per-bit. Ditegakkan tests/test_architecture.py.
    """
    if commands:
        for r in _client.apply_batch_sync(list(commands), False):
            if r.has_error():
                raise RuntimeError(f'command to actor {r.actor_id} failed: {r.error}')
    return world.tick()


def velocity(actor, v, yaw_deg=None):
    """Perintah kecepatan bodi `v` m/s searah `yaw_deg` (derajat, frame CARLA);
    bawaan = arah hadap aktor saat ini."""
    yaw = math.radians(actor.get_transform().rotation.yaw if yaw_deg is None else yaw_deg)
    return carla.command.ApplyTargetVelocity(
        actor.id, carla.Vector3D(v * math.cos(yaw), v * math.sin(yaw), 0.0))


def pose(actor, x, y, z, yaw_deg):
    """Perintah tempatkan aktor di pose tertentu (frame CARLA, derajat).

    Dipakai skrip validasi perception yang perlu pose TERKENDALI, bukan hasil
    fisika: `check_lanes.py` menggeser ego melintang untuk menguji simpangan yang
    diukur. Bukan untuk loop kendali -- di situ pose harus datang dari fisika.
    """
    return carla.command.ApplyTransform(
        actor.id, carla.Transform(carla.Location(x=float(x), y=float(y), z=float(z)),
                                  carla.Rotation(yaw=float(yaw_deg))))


@contextlib.contextmanager
def ego_vehicle(world):
    """Spawn point tetap, bukan acak (bagian 11.1)."""
    bp = world.get_blueprint_library().find(config.EGO_BP)
    bp.set_attribute('role_name', 'ego')
    ego = world.spawn_actor(bp, world.get_map().get_spawn_points()[config.SPAWN_IDX])

    world.tick()
    try:
        yield ego
    finally:
        ego.destroy()
        world.tick()


def walk_lane(wp, length_m, step=1.0):
    """Telusuri lajur maju tiap `step` meter. Berhenti di percabangan/persimpangan."""
    yield wp
    travelled = 0.0
    while travelled < length_m:
        nxt = wp.next(step)
        if len(nxt) != 1:
            return
        wp = nxt[0]
        travelled += step
        yield wp


def reference_path(world, location, length_m=300.0, step=1.0, max_drift_deg=5.0):
    """Centerline lajur mulai dari titik terdekat, frame right-handed (bagian 5.1).

    Path ini TIDAK berubah saat menyalip -- manuver dinyatakan sebagai deviasi
    lateral terhadapnya. Kembali: ndarray (4, N) = [x, y, psi, z].

    Baris z tetap dalam frame CARLA (ketinggian tidak dikonversi) dan hanya
    dipakai untuk menempatkan aktor; planner hanya membaca tiga baris pertama.
    """
    wp = world.get_map().get_waypoint(location, project_to_road=True,
                                      lane_type=carla.LaneType.Driving)
    road = list(walk_lane(wp, length_m, step))
    pts = [localization.carla_xy_yaw_to_rh(p.transform.location, p.transform.rotation.yaw)
           for p in road]

    covered = (len(pts) - 1) * step
    if covered < length_m * 0.99:
        raise RuntimeError(f'Lane ended after {covered:.0f} m of the {length_m:.0f} m '
                           f'requested (fork or end of road). Pick another SPAWN_IDX.')

    x, y, psi = np.array(pts).T
    psi = np.unwrap(psi)        # tanpa ini psi_ref melompat 2pi dan error MPC meledak

    drift = np.degrees(np.abs(psi - psi[0]).max())
    if drift > max_drift_deg:
        raise RuntimeError(f'Path deviates {drift:.1f}° from straight over {length_m:.0f} m '
                           f'(limit {max_drift_deg:.0f}°). The whole thesis assumes a straight '
                           f'section -- shorten length_m or pick another SPAWN_IDX.')
    return np.vstack([x, y, psi, [p.transform.location.z for p in road]])
