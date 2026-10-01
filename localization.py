"""Tahap 2 -- localization dari ground truth CARLA (rencana kerja bagian 4).

Satu-satunya tempat konversi koordinat terjadi (aturan bagian 2.4):
  - CARLA left-handed, Y ke bawah  ->  right-handed:  y -> -y, yaw -> -yaw
  - origin actor -> titik sumbu belakang, memakai offset terukur bukan L/2

Modul lain menerima data yang sudah bersih dan tidak boleh mengonversi apa pun.
"""
import math
from dataclasses import dataclass

import numpy as np

import config


@dataclass(frozen=True)
class EgoState:
    x: float          # m, frame world right-handed, titik sumbu belakang
    y: float
    yaw: float        # rad
    v: float          # m/s, longitudinal
    yaw_rate: float   # rad/s
    timestamp: float  # detik simulasi

    def as_vector(self) -> np.ndarray:
        return np.array([self.x, self.y, self.yaw, self.v])


def to_rh_rear_axle(tf, vel, rear_offset_x):
    """(transform, velocity, offset) -> (x, y, yaw, v_long) di frame right-handed."""
    yaw_c = math.radians(tf.rotation.yaw)
    x = tf.location.x + rear_offset_x * math.cos(yaw_c)
    y_c = tf.location.y + rear_offset_x * math.sin(yaw_c)
    v_long = vel.x * math.cos(yaw_c) + vel.y * math.sin(yaw_c)
    return x, -y_c, -yaw_c, v_long


def wrap(angle):
    """Sudut -> (-pi, pi]. Jangan pernah mengurangi dua sudut tanpa ini."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def carla_xy_yaw_to_rh(location, yaw_deg):
    """Titik & arah dari data peta -> right-handed. Bukan untuk ego (tanpa offset sumbu)."""
    return location.x, -location.y, -math.radians(yaw_deg)


class PathFrame:
    """Frame sejajar jalan: x = maju sepanjang jalan, y = simpangan lateral.

    Jalan lurus (bagian 5), jadi cukup satu rotasi + translasi. Seluruh modul
    hilir (planner, FSM, MPC) bekerja di frame ini supaya tidak ada dua konvensi.
    """

    def __init__(self, ref):
        """ref = (3, N) [x, y, psi] dari simulation.reference_path, frame RH."""
        self.origin = np.asarray(ref[:2, 0], dtype=float)
        self.psi0 = float(np.mean(ref[2]))
        self._c, self._s = math.cos(self.psi0), math.sin(self.psi0)

    @classmethod
    def from_perception(cls, ego_rh, lane, x0=0.0):
        """Frame jalan dari APA YANG DILIHAT, bukan dari `world.get_map()`.

        `ego_rh` = EgoState frame dunia right-handed, `lane` = `lanes.LaneGeometry`
        frame terakhir. Dua besaran yang dipakai keduanya hasil ukur kamera:

          * arah jalan  = yaw ego dunia dikurangi yaw ego terhadap lajur;
          * tengah lajur = posisi ego digeser melintang sejauh simpangan terukur.

        `x0` hanya memilih di mana s = 0 diletakkan. Itu konvensi, bukan geometri --
        tidak ada besaran fisik yang bergantung padanya -- dan disamakan dengan
        frame peta supaya log kedua jalur bisa dibandingkan langsung.
        """
        psi0 = wrap(ego_rh.yaw - lane.yaw)
        c, s_ = math.cos(psi0), math.sin(psi0)
        dev = lane.lane_dev
        # titik di sumbu lajur, sejajar ego; lalu mundur x0 supaya ego.x = x0
        ox = ego_rh.x + dev * s_ - x0 * c
        oy = ego_rh.y - dev * c - x0 * s_
        obj = cls.__new__(cls)
        obj.origin = np.array([ox, oy], dtype=float)
        obj.psi0 = float(psi0)
        obj._c, obj._s = c, s_
        return obj

    @classmethod
    def from_pose(cls, ego_rh, psi0, x0, y0):
        """Frame berarah `psi0` yang menempatkan ego tepat di (x0, y0).

        Untuk MENJEJAK arah jalan tanpa memindahkan apa pun: jangkar sekali di
        awal membekukan galat arah, dan galat 0,3 deg saja menjadi simpangan
        0,87 m setelah 250 m -- terukur, dan ego benar-benar keluar dari tengah
        lajur, bukan sekadar frame yang miring. Memutar frame begitu saja tidak
        bisa: ego 250 m dari titik asal, jadi rotasi 0,3 deg melompatkan y-nya
        1,3 m. Titik asalnya karena itu ikut digeser supaya (x0, y0) tetap.
        """
        c, s_ = math.cos(psi0), math.sin(psi0)
        obj = cls.__new__(cls)
        obj.origin = np.array([ego_rh.x - (x0 * c - y0 * s_),
                                ego_rh.y - (x0 * s_ + y0 * c)], dtype=float)
        obj.psi0 = float(psi0)
        obj._c, obj._s = c, s_
        return obj

    def _xy(self, x, y):
        dx, dy = x - self.origin[0], y - self.origin[1]
        return self._c * dx + self._s * dy, -self._s * dx + self._c * dy

    def points(self, x_rh, y_rh):
        """Titik frame right-handed -> frame jalan. Untuk pencatatan ground truth."""
        return self._xy(x_rh, y_rh)

    def to_rh(self, px, py):
        """Kebalikan `points`: frame jalan -> right-handed. Untuk menggambar
        lintasan planner di atas citra kamera."""
        px, py = np.asarray(px, dtype=float), np.asarray(py, dtype=float)
        return (self.origin[0] + self._c * px - self._s * py,
                self.origin[1] + self._s * px + self._c * py)

    def ego(self, state: EgoState) -> EgoState:
        x, y = self._xy(state.x, state.y)
        return EgoState(x, y, wrap(state.yaw - self.psi0), state.v,
                        state.yaw_rate, state.timestamp)

    def obstacle(self, x, y, vx, vy):
        """-> [x, y, vx, vy] di frame jalan. Kecepatan hanya dirotasi."""
        px, py = self._xy(x, y)
        return [px, py, self._c * vx + self._s * vy, -self._s * vx + self._c * vy]


def obstacles_ego_to_road(obs_rel, ego):
    """Halangan frame ego -> frame jalan. obs_rel = (M, 4) [x, y, vx, vy].

    Perception melaporkan apa yang DILIHAT: posisi relatif terhadap ego dan
    kecepatan relatif terhadap ego (bagian 10.4). Kamera memang hanya bisa itu.
    Penjumlahan "+ posisi ego" dan "+ kecepatan ego" dilakukan di sini, bukan di
    dalam perception -- supaya VisionPerception nanti tinggal dipasang tanpa
    perlu tahu-menahu soal frame jalan.
    """
    o = np.asarray(obs_rel, dtype=float)
    if len(o) == 0:
        return np.empty((0, 4))
    c, s = math.cos(ego.yaw), math.sin(ego.yaw)
    # Jangkar = PUSAT BODI ego, bukan sumbu belakang. Perception mengukur dari
    # pusat bodi ego (terukur: bounding_box.location.x = -0,005 m terhadap titik
    # asal aktor), sedangkan `ego.x` adalah sumbu belakang. Menambahkan sumbu
    # belakang menaruh halangan 1,433 m terlalu dekat di frame jalan.
    #
    # Satu perbaikan di sini membenarkan KEDUA pemakainya, karena masing-masing
    # sudah menuliskan acuannya sendiri: zona planner & MPC menggeser ego ke
    # pusat bodi (`+ AXLE_TO_CENTER`) lalu mengurangkan posisi halangan, jadi
    # keduanya kini pusat-ke-pusat; sementara main.py mengurangkan `ego.x` untuk
    # FSM, jadi `front[0]` kini sumbu belakang -> pusat bodi, persis yang
    # didokumentasikan `planning._v_follow`.
    xc = ego.x + config.AXLE_TO_CENTER * c
    yc = ego.y + config.AXLE_TO_CENTER * s
    x, y, vx, vy = o[:, 0], o[:, 1], o[:, 2], o[:, 3]
    return np.column_stack([
        xc + c * x - s * y,
        yc + s * x + c * y,
        c * vx - s * vy + ego.v * c,          # relatif -> absolut
        s * vx + c * vy + ego.v * s])


class CarlaGTLocalization:
    def __init__(self, ego, rear_offset_x):
        self.ego = ego
        self.rear_offset_x = rear_offset_x

    def update(self) -> EgoState:
        """Panggil SETELAH world.tick(), bukan sebelum."""
        x, y, yaw, v = to_rh_rear_axle(self.ego.get_transform(),
                                       self.ego.get_velocity(),
                                       self.rear_offset_x)
        # get_angular_velocity() deg/s di frame dunia; komponen z = laju yaw.
        # Dinegasikan sama seperti yaw karena konversi ke right-handed.
        yaw_rate = -math.radians(self.ego.get_angular_velocity().z)
        t = self.ego.get_world().get_snapshot().timestamp.elapsed_seconds
        return EgoState(x, y, yaw, v, yaw_rate, t)
