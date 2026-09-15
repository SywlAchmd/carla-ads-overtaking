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

import config


def _pasang(world, ego, tipe, tf, **atribut):
    bp = world.get_blueprint_library().find(tipe)
    for nama, nilai in atribut.items():
        bp.set_attribute(nama, str(nilai))
    return world.spawn_actor(bp, tf, attach_to=ego)


class RigKamera:
    """Kamera warna (+ depth) di posisi ala KITTI. Pakai lewat `with`.

    `params` = out/vehicle_params.json; offset sumbu belakang dipakai untuk
    mengubah jarak "di depan sumbu belakang" menjadi koordinat frame kendaraan.
    """

    def __init__(self, world, ego, params, stereo=False):
        x = config.KAMERA_DEPAN_SUMBU + params['rear_axle_offset_x']
        ukuran = dict(image_size_x=config.KAMERA_LEBAR, image_size_y=config.KAMERA_TINGGI,
                      fov=config.KAMERA_FOV)
        dy = config.KAMERA_BASELINE / 2.0 if stereo else 0.0
        titik = {'rgb': -dy, 'depth': -dy}
        if stereo:
            titik['rgb_kanan'] = +dy
        self.sensor, self.antrean = {}, {}
        for nama, y in titik.items():
            tipe = 'sensor.camera.depth' if nama == 'depth' else 'sensor.camera.rgb'
            tf = carla.Transform(carla.Location(x=x, y=y, z=config.KAMERA_Z))
            s = _pasang(world, ego, tipe, tf, **ukuran)
            q = queue.Queue()
            s.listen(q.put)
            self.sensor[nama], self.antrean[nama] = s, q

    def ambil(self, timeout=2.0):
        """Frame terbaru tiap sensor untuk tick yang baru saja dijalankan."""
        return {nama: q.get(timeout=timeout) for nama, q in self.antrean.items()}

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
    """
    import numpy as np
    buf = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
    b, g, r = (buf[:, :, i].astype(np.float64) for i in range(3))
    return (r + g * 256.0 + b * 65536.0) / (16777215.0) * 1000.0
