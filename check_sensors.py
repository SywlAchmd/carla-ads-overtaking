"""Verifikasi penempatan rig kamera terhadap angka KITTI (Tahap 8).

Menyimpan satu frame RGB dan depth, lalu memeriksa tinggi dan posisi memanjang
kamera yang BENAR-BENAR terjadi di simulator, bukan yang diminta.

    python check_sensors.py
"""
import json

import numpy as np

import config
import localization
import sensors
import simulation


def main():
    params = json.load(open(config.VEHICLE_PARAMS_JSON))
    with simulation.carla_world() as world:
        with simulation.ego_vehicle(world) as ego:
            with sensors.CameraRig(world, ego, params) as rig:
                for _ in range(10):                      # tunggu suspensi & render mapan
                    simulation.tick(world)
                    frame = rig.grab()

                tf_ego = ego.get_transform()
                tf_cam = rig.sensor['rgb'].get_transform()
                box = ego.bounding_box
                # titik terbawah bodi = permukaan jalan (roda menyentuh tanah)
                ground = tf_ego.location.z + box.location.z - box.extent.z
                forward = ((tf_cam.location.x - tf_ego.location.x) ** 2
                        + (tf_cam.location.y - tf_ego.location.y) ** 2) ** 0.5

                d = sensors.depth_meter(frame['depth'])
                print(f'camera height above road : {tf_cam.location.z - ground:.3f} m '
                      f'(requested {config.CAMERA_Z:.2f})')
                print(f'distance from rear axle   : {forward - params["rear_axle_offset_x"]:.3f} m '
                      f'(requested {config.CAMERA_AHEAD_OF_AXLE:.2f})')
                print(f'resolution                : {frame["rgb"].width}x{frame["rgb"].height}, '
                      f'fov {config.CAMERA_FOV:.0f} degrees')
                print(f'depth at image center     : {d[d.shape[0] // 2, d.shape[1] // 2]:.1f} m '
                      f'(range {d.min():.1f}-{d.max():.0f} m)')

                for name in ('rgb', 'depth'):
                    path = f'{config.OUT_DIR}/sensor_{name}.png'
                    frame[name].save_to_disk(path)
                    print(f'{name:<6} -> {path}')


if __name__ == '__main__':
    main()
