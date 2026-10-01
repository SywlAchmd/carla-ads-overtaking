"""Rig kamera untuk perception berbasis vision (rencana kerja bagian 10).

Penempatan mendekati KITTI (lihat config): kamera warna 1,65 m di atas jalan dan
1,68 m di depan sumbu roda belakang. Depth camera ditaruh SATU TITIK dengan kamera
warna kiri, supaya piksel deteksi bisa langsung dibaca kedalamannya tanpa
kalibrasi antar sensor. Depth di CARLA adalah sensor ideal; nyatakan itu di
batasan masalah (aturan penulisan nomor 3).

Mode sinkron: tiap sensor menghasilkan tepat satu frame per tick, diambil lewat
antrean supaya frame yang dipakai pasti milik tick yang sama.
"""
import queue

import carla
import numpy as np

import config


def _attach(world, ego, bp_id, tf, **attrs):
    bp = world.get_blueprint_library().find(bp_id)
    for name, value in attrs.items():
        bp.set_attribute(name, str(value))
    return world.spawn_actor(bp, tf, attach_to=ego)


class CameraRig:
    """Kamera warna (+ depth) di posisi ala KITTI. Pakai lewat `with`.

    `params` = out/vehicle_params.json; offset sumbu belakang dipakai untuk
    mengubah jarak "di depan sumbu belakang" menjadi koordinat frame kendaraan.
    """

    def __init__(self, world, ego, params, stereo=False):
        x = config.CAMERA_AHEAD_OF_AXLE + params['rear_axle_offset_x']
        size = dict(image_size_x=config.CAMERA_WIDTH, image_size_y=config.CAMERA_HEIGHT,
                      fov=config.CAMERA_FOV)
        dy = config.CAMERA_BASELINE / 2.0 if stereo else 0.0
        points = {'rgb': -dy, 'depth': -dy}
        if stereo:
            points['rgb_right'] = +dy
        self.x = x                      # m, kamera di depan titik asal aktor ego
        self.sensor, self.queues = {}, {}
        for name, y in points.items():
            bp_id = 'sensor.camera.depth' if name == 'depth' else 'sensor.camera.rgb'
            tf = carla.Transform(carla.Location(x=x, y=y, z=config.CAMERA_Z))
            s = _attach(world, ego, bp_id, tf, **size)
            q = queue.Queue()
            s.listen(q.put)
            self.sensor[name], self.queues[name] = s, q

    def grab(self, timeout=2.0):
        """Frame terbaru tiap sensor untuk tick yang baru saja dijalankan."""
        return {name: q.get(timeout=timeout) for name, q in self.queues.items()}

    def destroy(self):
        for s in self.sensor.values():
            s.stop()
            s.destroy()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.destroy()


def depth_meter(image):
    """Citra depth CARLA -> jarak dalam meter, ndarray (H, W).

    CARLA mengkodekan jarak di tiga kanal warna; rumusnya ada di dokumentasi
    sensor: (R + G*256 + B*256^2) / (256^3 - 1) * 1000 meter.

    Nilainya jarak PLANAR (sepanjang sumbu optik), bukan radial -- diukur 16
    September 2026 terhadap permukaan jalan, rasio tepi/tengah 1,000 vs 1,28
    yang diprediksi radial. Lihat CATATAN.
    """
    buf = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
    b, g, r = (buf[:, :, i].astype(np.float64) for i in range(3))
    return (r + g * 256.0 + b * 65536.0) / (16777215.0) * 1000.0


def rgb_array(image):
    """Citra RGB CARLA -> ndarray (H, W, 3), urutan RGB."""
    buf = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
    return buf[:, :, :3][:, :, ::-1].copy()
