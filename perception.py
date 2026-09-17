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

import config
import sensors
import tracking

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


F_PIKSEL = config.KAMERA_LEBAR / (2.0 * math.tan(math.radians(config.KAMERA_FOV) / 2.0))


def koreksi_muka(kotak):
    """Geseran dari permukaan yang TERLIHAT ke pusat bodi -> (dx memanjang, dy melintang).

    Depth membaca permukaan terdekat yang terlihat (bagian 18.4), dan permukaan
    itu berganti selama manuver: saat target di depan yang terlihat muka
    BELAKANG, saat berdampingan yang terlihat SISI. Menambahkan setengah panjang
    di sumbu memanjang tanpa syarat -- seperti versi pertama -- melaporkan target
    sampai +3,82 m terlalu jauh ke depan begitu ego berdampingan, yaitu seolah
    kendaraan itu tepat di depan ego (bagian 19.6).

    Pembedanya rasio lebar/tinggi kotak: terukur 1,04 tampak belakang dan 2,48
    tampak samping untuk Nissan Patrol -- terpisah 2,4x. Di antaranya dicampur
    linier, karena sudut pandang serong memang memperlihatkan keduanya sekaligus.

    ponytail: satu jenis kendaraan, dimensi dianggap tetap (sudah di batasan
    masalah). Kalau skenario nanti memuat kendaraan beragam, rasio ini harus
    datang dari kelas deteksi, bukan dari konstanta.
    """
    ar = (kotak[2] - kotak[0]) / max(kotak[3] - kotak[1], 1.0)
    f = float(np.clip((ar - config.AR_BELAKANG) / (config.AR_SAMPING - config.AR_BELAKANG),
                      0.0, 1.0))
    return (1.0 - f) * config.LAIN_PANJANG / 2.0, f * config.LAIN_LEBAR / 2.0


class VisionPerception:
    """YOLOPX + depth camera + pelacak. Kontrak keluaran sama dengan GT di atas.

    Depth CARLA terukur **planar** (sepanjang sumbu optik), jadi balik-proyeksinya
    langsung tanpa faktor sinar: Z = depth, y = -Z*du/f. Depth membaca permukaan
    yang terlihat, bukan pusat bodi; koreksinya di `koreksi_muka`.
    Dimensi kendaraan lain dianggap tetap -- masuk batasan masalah.
    """

    def __init__(self, net, rig):
        self.net, self.rig = net, rig
        self.pelacak = tracking.Pelacak()

    def update(self, frame, dt, ego_v=0.0, ego_a=0.0, ego_w=0.0):
        depth = sensors.depth_meter(frame['depth'])
        kotak, _, _ = self.net.infer(sensors.rgb_array(frame['rgb']),
                                     conf=config.TRACK_CONF_RENDAH)
        pakai, z, R = [], [], []
        for b in kotak:
            u, v = int((b[0] + b[2]) / 2), int((b[1] + b[3]) / 2)
            # ponytail: median petak 7x7 di pusat kotak -- terukur tepat di bagian
            # 18.4 sampai 50 m. Ganti kalau kotak pernah lebih kecil dari 7 px.
            d = float(np.median(depth[max(v - 3, 0):v + 4, max(u - 3, 0):u + 4]))
            if not 0.5 < d < JANGKAUAN:
                continue
            pakai.append(b)
            dx, dy = koreksi_muka(b)
            y = -d * (u - config.KAMERA_LEBAR / 2.0) / F_PIKSEL
            # koreksi melintang menjauhi ego: pusat bodi ada di BALIK sisi yang terlihat
            z.append([d + self.rig.x + dx, y + math.copysign(dy, y)])
            # derau melintang tumbuh dengan jarak: kuantisasi kotak x d/f
            R.append(np.diag([config.TRACK_SIGMA_D ** 2,
                              (config.TRACK_SIGMA_PIKSEL * d / F_PIKSEL) ** 2]))
        if not pakai:
            return self.pelacak.update(np.empty((0, 4)), [], np.empty((0, 2)),
                                       np.empty((0, 2, 2)), dt, ego_v, ego_a, ego_w)
        pakai = np.array(pakai)
        return self.pelacak.update(pakai[:, :4], pakai[:, 4], z, R,
                                   dt, ego_v, ego_a, ego_w)
