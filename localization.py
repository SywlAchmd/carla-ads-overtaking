"""Tahap 2 -- localization dari ground truth CARLA (rencana kerja bagian 4).

Satu-satunya tempat konversi koordinat terjadi (aturan bagian 2.4):
  - CARLA left-handed, Y ke bawah  ->  right-handed:  y -> -y, yaw -> -yaw
  - origin actor -> titik sumbu belakang, memakai offset terukur bukan L/2

Modul lain menerima data yang sudah bersih dan tidak boleh mengonversi apa pun.
"""
import math
from dataclasses import dataclass

import numpy as np


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


def wrap(sudut):
    """Sudut -> (-pi, pi]. Jangan pernah mengurangi dua sudut tanpa ini."""
    return (sudut + math.pi) % (2 * math.pi) - math.pi


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

    def _xy(self, x, y):
        dx, dy = x - self.origin[0], y - self.origin[1]
        return self._c * dx + self._s * dy, -self._s * dx + self._c * dy

    def titik(self, x_rh, y_rh):
        """Titik frame right-handed -> frame jalan. Untuk pencatatan ground truth."""
        return self._xy(x_rh, y_rh)

    def ego(self, state: EgoState) -> EgoState:
        x, y = self._xy(state.x, state.y)
        return EgoState(x, y, wrap(state.yaw - self.psi0), state.v,
                        state.yaw_rate, state.timestamp)

    def obstacle(self, x, y, vx, vy):
        """-> [x, y, vx, vy] di frame jalan. Kecepatan hanya dirotasi."""
        px, py = self._xy(x, y)
        return [px, py, self._c * vx + self._s * vy, -self._s * vx + self._c * vy]


def halangan_ego_ke_jalan(obs_rel, ego):
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
    x, y, vx, vy = o[:, 0], o[:, 1], o[:, 2], o[:, 3]
    return np.column_stack([
        ego.x + c * x - s * y,
        ego.y + s * x + c * y,
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
