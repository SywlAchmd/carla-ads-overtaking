"""Verifikasi penempatan rig kamera terhadap angka KITTI (Tahap 8).

Menyimpan satu frame RGB dan depth, lalu memeriksa tinggi dan posisi memanjang
kamera yang BENAR-BENAR terjadi di simulator, bukan yang diminta.

    python cek_sensor.py
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
            with sensors.RigKamera(world, ego, params) as rig:
                for _ in range(10):                      # tunggu suspensi & render mapan
                    simulation.tick(world)
                    frame = rig.ambil()

                tf_ego = ego.get_transform()
                tf_cam = rig.sensor['rgb'].get_transform()
                kotak = ego.bounding_box
                # titik terbawah bodi = permukaan jalan (roda menyentuh tanah)
                tanah = tf_ego.location.z + kotak.location.z - kotak.extent.z
                maju = ((tf_cam.location.x - tf_ego.location.x) ** 2
                        + (tf_cam.location.y - tf_ego.location.y) ** 2) ** 0.5

                d = sensors.depth_meter(frame['depth'])
                print(f'tinggi kamera di atas jalan : {tf_cam.location.z - tanah:.3f} m '
                      f'(diminta {config.KAMERA_Z:.2f})')
                print(f'jarak dari sumbu belakang   : {maju - params["rear_axle_offset_x"]:.3f} m '
                      f'(diminta {config.KAMERA_DEPAN_SUMBU:.2f})')
                print(f'resolusi                    : {frame["rgb"].width}x{frame["rgb"].height}, '
                      f'fov {config.KAMERA_FOV:.0f} derajat')
                print(f'depth di tengah citra       : {d[d.shape[0] // 2, d.shape[1] // 2]:.1f} m '
                      f'(rentang {d.min():.1f}-{d.max():.0f} m)')

                for nama in ('rgb', 'depth'):
                    jalur = f'{config.OUT_DIR}/sensor_{nama}.png'
                    frame[nama].save_to_disk(jalur)
                    print(f'{nama:<6} -> {jalur}')


if __name__ == '__main__':
    main()
