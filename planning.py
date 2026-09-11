"""Tahap 3 -- local planner (rencana kerja bagian 5.2-5.5).

MURNI NUMERIK. Tidak boleh mengimpor carla (aturan bagian 2.4) supaya bisa diuji
di terminal tanpa menyalakan simulator.

Quintic lateral: 6 syarat batas -> 6 koefisien -> derajat 5. Ini bukan pilihan
melainkan solusi Euler-Lagrange dari minimisasi integral jerk kuadrat
(Werling dkk. 2010). Quartic longitudinal: posisi akhir bebas, jadi 5 syarat.

Jalan lurus, jadi Cartesian langsung -- s sejajar x, d sejajar y (bagian 5).
"""
import math
from dataclasses import dataclass

import numpy as np

import config


def quintic_coeffs(y0, dy0, ddy0, yT, dyT, ddyT, T):
    """Koefisien naik [a0..a5] untuk y(t) yang memenuhi 6 syarat batas."""
    a0, a1, a2 = y0, dy0, ddy0 / 2.0
    A = np.array([[T**3,      T**4,      T**5],
                  [3 * T**2,  4 * T**3,  5 * T**4],
                  [6 * T,    12 * T**2, 20 * T**3]])
    b = np.array([yT - (a0 + a1 * T + a2 * T**2),
                  dyT - (a1 + 2 * a2 * T),
                  ddyT - 2 * a2])
    return np.concatenate([[a0, a1, a2], np.linalg.solve(A, b)])


def quartic_coeffs(x0, dx0, ddx0, dxT, ddxT, T):
    """Koefisien naik [b0..b4]. Derajat 4: hanya kecepatan akhir yang ditarget."""
    b0, b1, b2 = x0, dx0, ddx0 / 2.0
    A = np.array([[3 * T**2, 4 * T**3],
                  [6 * T,   12 * T**2]])
    b = np.array([dxT - (b1 + 2 * b2 * T),
                  ddxT - 2 * b2])
    return np.concatenate([[b0, b1, b2], np.linalg.solve(A, b)])


def _deriv(c, n=1):
    for _ in range(n):
        c = c[1:] * np.arange(1, len(c))
    return c


def _val(c, t):
    return np.polyval(c[::-1], t)


def peak_lateral_accel(dy, T):
    """|y''|max analitik untuk quintic bersyarat awal nol -- 10/sqrt(3) = 5.7735."""
    return (10.0 / math.sqrt(3.0)) * abs(dy) / T**2


@dataclass(frozen=True)
class Trajectory:
    states: np.ndarray    # (4, N+1) = [x_ref, y_ref, psi_ref, v_ref]
    dt: float
    timestamp: float = 0.0
    cy: np.ndarray = None      # koefisien polinomial lateral & longitudinal
    cx: np.ndarray = None

    def sample_at(self, t):
        """Interpolasi -- MPC 20 Hz membaca lintasan planner 10 Hz."""
        grid = np.arange(self.states.shape[1]) * self.dt
        return np.array([np.interp(t, grid, row) for row in self.states])

    def lateral_at(self, t):
        """(y, y\', y\'\') eksak dari polinomial.

        Dipakai saat replan receding horizon: state lateral harus diteruskan
        persis, bukan hasil diferensiasi numerik dari sampel.
        """
        return (_val(self.cy, t), _val(_deriv(self.cy), t), _val(_deriv(self.cy, 2), t))


def _build(cx, cy, T, dt):
    """Sampel polinomial jadi (4, N+1) plus turunan untuk pemeriksaan kelayakan."""
    t = np.arange(0.0, T + 1e-9, dt)
    dcx, dcy = _deriv(cx), _deriv(cy)
    ddcx, ddcy = _deriv(cx, 2), _deriv(cy, 2)

    x, y = _val(cx, t), _val(cy, t)
    dx, dy = _val(dcx, t), _val(dcy, t)
    ddx, ddy = _val(ddcx, t), _val(ddcy, t)

    states = np.vstack([x, y, np.arctan2(dy, dx), np.hypot(dx, dy)])
    return t, states, dx, dy, ddx, ddy


def _curvature(dx, dy, ddx, ddy):
    speed = np.hypot(dx, dy)
    return np.abs(dx * ddy - dy * ddx) / np.maximum(speed**3, 1e-6)


def _ellipse_g(states, obstacles):
    """g_j untuk tiap obstacle di tiap sampel. g >= 1 berarti aman (bagian 7.2)."""
    if obstacles is None or len(obstacles) == 0:
        return None
    n = states.shape[1]
    k = np.arange(n) * config.PLANNER_DT
    obs = np.asarray(obstacles, dtype=float)          # (M, 4) = [x, y, vx, vy]
    xj = obs[:, 0:1] + obs[:, 2:3] * k                # prediksi kecepatan konstan
    yj = obs[:, 1:2] + obs[:, 3:4] * k
    return (((states[0] - xj) / config.ELLIPSE_A)**2
            + ((states[1] - yj) / config.ELLIPSE_B)**2)


def _cost(t, states, ddy, T, y_lane_center, v_desired, cy, cx, g):
    jerk_lat = np.trapezoid(_val(_deriv(cy, 3), t)**2, t)
    jerk_lon = np.trapezoid(_val(_deriv(cx, 3), t)**2, t)

    j_lat = jerk_lat + config.K_TIME * T + config.K_DEV * (states[1, -1] - y_lane_center)**2
    j_lon = jerk_lon + config.K_VEL * (states[3, -1] - v_desired)**2
    j_col = 0.0 if g is None else float(1.0 / g.min())
    return config.W_LAT * j_lat + config.W_LON * j_lon + config.W_COL * j_col


def plan_lane_change(y0, dy0, ddy0, x0, v0, a0, v_desired, obstacles=None,
                     side_sign=None, dt=None, y_goal=None):
    """Bangkitkan kandidat, saring yang tidak layak, kembalikan (terbaik, semua_layak).

    Mengembalikan (None, []) bila tidak ada kandidat yang lolos -- FSM harus
    memperlakukan itu sebagai abort, bukan error.
    """
    side_sign = config.SIDE_SIGN if side_sign is None else side_sign
    dt = config.PLANNER_DT if dt is None else dt
    # y_goal = tengah lajur tujuan (absolut). Diperlukan untuk manuver kembali,
    # yang targetnya 0 dan tidak bisa dinyatakan sebagai side_sign * LANE_WIDTH.
    y_lane_center = side_sign * config.LANE_WIDTH if y_goal is None else y_goal
    arah = 1.0 if y_lane_center >= y0 else -1.0
    kappa_max = 1.0 / config.MIN_TURN_RADIUS

    feasible = []
    for offset in config.LATERAL_OFFSETS:
        y_target = y_lane_center + arah * (offset - config.LANE_WIDTH)
        for T in config.MANEUVER_TIMES:
            if peak_lateral_accel(y_target - y0, T) > config.MAX_LATERAL_ACCEL:
                continue                                  # saringan analitik, murah

            cy = quintic_coeffs(y0, dy0, ddy0, y_target, 0.0, 0.0, T)
            cx = quartic_coeffs(x0, v0, a0, v_desired, 0.0, T)
            t, states, dx, dy, ddx, ddy = _build(cx, cy, T, dt)

            if np.abs(ddy).max() > config.MAX_LATERAL_ACCEL:
                continue
            if _curvature(dx, dy, ddx, ddy).max() > kappa_max:
                continue
            g = _ellipse_g(states, obstacles)
            if g is not None and g.min() < 1.0:
                continue                                  # bertabrakan

            cost = _cost(t, states, ddy, T, y_lane_center, v_desired, cy, cx, g)
            feasible.append((cost, offset, T, Trajectory(states, dt, cy=cy, cx=cx)))

    if not feasible:
        return None, []
    feasible.sort(key=lambda r: r[0])
    return feasible[0][3], feasible


# --- Behavior FSM (bagian 6) ------------------------------------------------
LANE_KEEPING = 'LANE_KEEPING'
CHECK_OVERTAKE = 'CHECK_OVERTAKE'
LANE_CHANGE_OVERTAKE = 'LANE_CHANGE_OVERTAKE'
OVERTAKING = 'OVERTAKING'
LANE_CHANGE_RETURN = 'LANE_CHANGE_RETURN'


def _di_lajur(obstacles, y_lajur):
    """Halangan yang pusatnya berada di dalam lajur y_lajur."""
    if obstacles is None or len(obstacles) == 0:
        return np.empty((0, 4))
    obs = np.asarray(obstacles, dtype=float)
    return obs[np.abs(obs[:, 1] - y_lajur) < config.LANE_WIDTH / 2.0]


def _ttc(depan, v_ego):
    """Waktu sampai menyusul kendaraan depan. inf bila tidak sedang mendekat."""
    dv = v_ego - depan[2]
    return depan[0] / dv if dv > 1e-3 else float('inf')


def _terdepan(obs):
    """Halangan terdekat di depan (x > 0), atau None."""
    depan = obs[obs[:, 0] > 0.0]
    return depan[np.argmin(depan[:, 0])] if len(depan) else None


class BehaviorFSM:
    """Memutuskan KAPAN menyalip. Local planner memutuskan BAGAIMANA.

    Setiap ambang berpasangan dengan ambang histeresis, dan setiap transisi harus
    bertahan `FSM_DWELL` detik sebelum dieksekusi -- tanpa keduanya, noise
    perception membuat state bolak-balik dan MPC menerima referensi yang berubah
    tiap frame (bagian 6).
    """

    def __init__(self, side_sign=None):
        self.side_sign = config.SIDE_SIGN if side_sign is None else side_sign
        self.state = LANE_KEEPING
        self._calon = None            # (state tujuan, waktu permintaan pertama)
        self.abort_terakhir = None    # untuk logging bagian 11.5

    @property
    def y_goal(self):
        """Tengah lajur yang sedang dituju -- diteruskan ke plan_lane_change."""
        menyalip = self.state in (LANE_CHANGE_OVERTAKE, OVERTAKING)
        return self.side_sign * config.LANE_WIDTH if menyalip else 0.0

    def _minta(self, tujuan, t):
        """Transisi baru dieksekusi setelah diminta terus-menerus selama dwell."""
        if self._calon is None or self._calon[0] != tujuan:
            self._calon = (tujuan, t)
            return False
        if t - self._calon[1] >= config.FSM_DWELL:
            self.state, self._calon = tujuan, None
            return True
        return False

    def _langsung(self, tujuan):
        """Abort tidak menunggu dwell -- menunda 0,3 s justru menambah risiko."""
        self.state, self._calon = tujuan, None

    def update(self, t, d, v_ego, obstacles):
        """Satu langkah FSM. `obstacles` = (M,4) [x, y, vx, vy], x relatif ego.

        `d` = simpangan lateral ego dari lajur asal. Kembalikan nama state.
        """
        y_tujuan = self.side_sign * config.LANE_WIDTH
        depan = _terdepan(_di_lajur(obstacles, 0.0))
        lajur_tujuan = _di_lajur(obstacles, y_tujuan)

        if self.state == LANE_KEEPING:
            if depan is not None and (_ttc(depan, v_ego) < config.TTC_TRIGGER
                                      and v_ego - depan[2] > config.DV_TRIGGER):
                self._minta(CHECK_OVERTAKE, t)
            else:
                self._calon = None

        elif self.state == CHECK_OVERTAKE:
            batal = (depan is None or _ttc(depan, v_ego) > config.TTC_EXIT
                     or v_ego - depan[2] < config.DV_EXIT)
            if batal:
                self._minta(LANE_KEEPING, t)
            elif self._lajur_tujuan_aman(lajur_tujuan) and self._sempat(depan, v_ego):
                self._minta(LANE_CHANGE_OVERTAKE, t)
            else:
                self._calon = None

        elif self.state == LANE_CHANGE_OVERTAKE:
            if not self._lajur_tujuan_aman(lajur_tujuan):
                self.abort_terakhir = t                       # jalur abort, bagian 6
                self._langsung(LANE_KEEPING)
            elif abs(d) >= config.LATERAL_MASUK * config.LANE_WIDTH:
                self._minta(OVERTAKING, t)
            else:
                self._calon = None

        elif self.state == OVERTAKING:
            # _terdepan tidak bisa dipakai di sini: dia hanya melihat x > 0,
            # sehingga kendaraan yang baru terlewati 1 m sudah dianggap hilang.
            asal = _di_lajur(obstacles, 0.0)
            belum_lewat = asal[asal[:, 0] > -config.PASS_MARGIN]
            if len(belum_lewat) == 0:
                self._minta(LANE_CHANGE_RETURN, t)
            else:
                self._calon = None

        elif self.state == LANE_CHANGE_RETURN:
            if abs(d) < config.LATERAL_SELESAI:
                self._minta(LANE_KEEPING, t)
            else:
                self._calon = None

        return self.state

    def _lajur_tujuan_aman(self, lajur_tujuan):
        if len(lajur_tujuan) == 0:
            return True
        x = lajur_tujuan[:, 0]
        return not (((x >= 0) & (x < config.D_SAFE_DEPAN)).any()
                    or ((x < 0) & (x > -config.D_SAFE_BELAKANG)).any())

    def _sempat(self, depan, v_ego):
        """Manuver tercepat harus selesai sebelum ego menyusul kendaraan depan."""
        return min(config.MANEUVER_TIMES) < _ttc(depan, v_ego)
