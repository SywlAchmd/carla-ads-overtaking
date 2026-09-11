"""Perception (rencana kerja bagian 2).

Keluaran: ndarray (M, 4) = [x, y, vx, vy] dalam **frame ego** -- posisi dan
kecepatan RELATIF terhadap ego, satuan meter dan m/s.

Frame ego dipilih karena itu satu-satunya yang bisa dihasilkan kamera: dia
melihat kotak di gambar dan menghitung jaraknya, tanpa tahu dirinya berada di
mana. Konversi ke frame jalan dilakukan `localization.halangan_ego_ke_jalan`.

`GroundTruthPerception` sengaja melaporkan besaran yang SAMA dengan yang nanti
dihasilkan `VisionPerception`, supaya perbandingan keduanya di bagian 11.4
setara -- bukan membandingkan informasi yang berbeda jenis.
"""
import math

import numpy as np

JANGKAUAN = 80.0        # m ke depan; di luar ini tidak relevan untuk horizon 2 detik
BELAKANG = 15.0         # m ke belakang; untuk memeriksa lajur tujuan


class GroundTruthPerception:
    def __init__(self, world, ego, jangkauan=JANGKAUAN):
        self.world, self.ego, self.jangkauan = world, ego, jangkauan

    def update(self):
        tf = self.ego.get_transform()
        v = self.ego.get_velocity()
        yaw = math.radians(tf.rotation.yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        # kecepatan ego di frame CARLA, untuk mengubah absolut -> relatif
        vx_e, vy_e = v.x, v.y

        keluar = []
        for a in self.world.get_actors().filter('vehicle.*'):
            if a.id == self.ego.id:
                continue
            loc, vel = a.get_transform().location, a.get_velocity()
            dx, dy = loc.x - tf.location.x, loc.y - tf.location.y
            # putar ke frame ego, lalu balik tanda y (CARLA left-handed -> RH)
            x = c * dx + s * dy
            y = -(-s * dx + c * dy)
            if not (-BELAKANG < x < self.jangkauan):
                continue
            dvx, dvy = vel.x - vx_e, vel.y - vy_e
            keluar.append([x, y, c * dvx + s * dvy, -(-s * dvx + c * dvy)])
        return np.array(keluar) if keluar else np.empty((0, 4))
