"""Rekam video lintasan acuan global planner, dari spawn sampai ujung path.

Mobil digerakkan langsung mengikuti waypoint (physics dimatikan) -- ini
visualisasi keluaran global planner, BUKAN hasil kendali. Controller baru ada
di Tahap 5.

    python record_path.py   ->  out/reference_path.mp4
"""
import glob
import os
import queue
import shutil
import subprocess
import tempfile

import carla

import config
import simulation

LENGTH = 300.0
SPEED = 13.9                                  # m/s, 50 km/jam
STEP = SPEED * config.FIXED_DELTA_SECONDS     # jarak per tick -> kecepatan nyata


def capture(world, ego, poses, out_name, cam_tf=None, extras=None, annotate=None,
            size=(960, 540)):
    """Gerakkan ego lewat `poses` (physics mati), rekam kamera, encode ke mp4.

    `extras` = list (actor, poses) untuk kendaraan lain yang ikut digerakkan.
    Dipakai bersama oleh record_path dan record_maneuver.
    """
    frames_dir = tempfile.mkdtemp(prefix='rec_')
    out = os.path.join(config.OUT_DIR, out_name)
    ego.set_simulate_physics(False)
    for actor, _ in (extras or []):
        actor.set_simulate_physics(False)

    bp = world.get_blueprint_library().find('sensor.camera.rgb')
    bp.set_attribute('image_size_x', str(size[0]))
    bp.set_attribute('image_size_y', str(size[1]))
    bp.set_attribute('bloom_intensity', '0.0')     # garis debug jadi membanjir kalau bloom aktif
    cam = world.spawn_actor(bp, cam_tf or carla.Transform(
        carla.Location(x=-9, z=4.5), carla.Rotation(pitch=-12)), attach_to=ego)
    images = queue.Queue()
    cam.listen(images.put)
    try:
        for i, tf in enumerate(poses):
            simulation.tick(world, [carla.command.ApplyTransform(ego.id, tf)] + [
                carla.command.ApplyTransform(actor.id, other[min(i, len(other) - 1)])
                for actor, other in (extras or [])])
            images.get(timeout=5.0).save_to_disk(f'{frames_dir}/{i:05d}.png')
    finally:
        cam.stop(); cam.destroy()

    berkas = sorted(glob.glob(f'{frames_dir}/*.png'))
    if annotate is not None:                    # overlay digambar setelah semua frame ada
        for i, f in enumerate(berkas):
            annotate(i, f)

    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', '20',
                    '-i', f'{frames_dir}/%05d.png', '-c:v', 'libx264',
                    '-pix_fmt', 'yuv420p', '-crf', '23', out], check=True)
    n = len(berkas)
    shutil.rmtree(frames_dir)
    print(f'{n} frame -> {out}  ({os.path.getsize(out)/1e6:.1f} MB, {n/20:.0f} detik)')
    return out


def main():
    with simulation.carla_world() as world:
        sp = world.get_map().get_spawn_points()[config.SPAWN_IDX]
        wps = list(simulation.walk_lane(
            world.get_map().get_waypoint(sp.location, project_to_road=True,
                                         lane_type=carla.LaneType.Driving),
            LENGTH, STEP))
        print(f'{len(wps)} titik, step {STEP:.2f} m, setara {SPEED * 3.6:.0f} km/jam')
        poses = []
        for wp in wps:
            tf = wp.transform
            tf.location.z += 0.05
            poses.append(tf)
        with simulation.ego_vehicle(world) as ego:
            capture(world, ego, poses, 'reference_path.mp4')


if __name__ == '__main__':
    main()
