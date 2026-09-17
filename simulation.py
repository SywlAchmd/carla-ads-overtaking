"""Setup dunia CARLA: koneksi, mode sinkron, spawn ego, reference path."""
import contextlib
import math

import carla
import numpy as np

import config
import localization

_klien = None           # diisi carla_world(), dipakai tick()


@contextlib.contextmanager
def carla_world():
    """Mode sinkron dipaksa aktif; setting dunia dikembalikan saat keluar."""
    global _klien
    client = carla.Client(config.CARLA_HOST, config.CARLA_PORT)
    client.set_timeout(config.CARLA_TIMEOUT)
    _klien = client

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


def tick(world, perintah=()):
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
    if perintah:
        for r in _klien.apply_batch_sync(list(perintah), False):
            if r.has_error():
                raise RuntimeError(f'perintah ke aktor {r.actor_id} gagal: {r.error}')
    return world.tick()


def kecepatan(actor, v, yaw_deg=None):
    """Perintah kecepatan bodi `v` m/s searah `yaw_deg` (derajat, frame CARLA);
    bawaan = arah hadap aktor saat ini."""
    yaw = math.radians(actor.get_transform().rotation.yaw if yaw_deg is None else yaw_deg)
    return carla.command.ApplyTargetVelocity(
        actor.id, carla.Vector3D(v * math.cos(yaw), v * math.sin(yaw), 0.0))


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
    jalan = list(walk_lane(wp, length_m, step))
    pts = [localization.carla_xy_yaw_to_rh(p.transform.location, p.transform.rotation.yaw)
           for p in jalan]

    covered = (len(pts) - 1) * step
    if covered < length_m * 0.99:
        raise RuntimeError(f'Lajur habis setelah {covered:.0f} m dari {length_m:.0f} m yang '
                           f'diminta (percabangan atau ujung jalan). Pilih SPAWN_IDX lain.')

    x, y, psi = np.array(pts).T
    psi = np.unwrap(psi)        # tanpa ini psi_ref melompat 2pi dan error MPC meledak

    drift = np.degrees(np.abs(psi - psi[0]).max())
    if drift > max_drift_deg:
        raise RuntimeError(f'Path menyimpang {drift:.1f}° dari lurus sepanjang {length_m:.0f} m '
                           f'(batas {max_drift_deg:.0f}°). Seluruh skripsi mengasumsikan ruas '
                           f'lurus -- perpendek length_m atau pilih SPAWN_IDX lain.')
    return np.vstack([x, y, psi, [p.transform.location.z for p in jalan]])
