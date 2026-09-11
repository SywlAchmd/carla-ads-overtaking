"""Tahap 5 -- Model Predictive Control (rencana kerja bagian 7).

MURNI NUMERIK. Tidak boleh mengimpor carla (aturan bagian 2.4).

Graf optimasi CasADi dibangun SEKALI di __init__; tiap tick hanya set_value dan
solve. Membangun ulang tiap tick membuat solve puluhan kali lebih lambat.
"""
import math
import time
from dataclasses import dataclass

import casadi as ca
import numpy as np

import config

PARKIR = 1.0e4          # slot obstacle kosong diparkir sejauh ini, g jadi raksasa


@dataclass(frozen=True)
class ControlCommand:
    throttle: float       # [0, 1]
    brake: float          # [0, 1]
    steer: float          # [-1, 1]
    accel_cmd: float      # m/s², nilai mentah dari MPC
    delta_cmd: float      # rad
    solve_time_ms: float
    solver_ok: bool


def _f(x, u, L):
    """Kinematic bicycle model, versi simbolik."""
    return ca.vertcat(x[3] * ca.cos(x[2]),
                      x[3] * ca.sin(x[2]),
                      x[3] / L * ca.tan(u[1]),
                      u[0])


def _rk4(x, u, L, dt):
    k1 = _f(x, u, L)
    k2 = _f(x + dt / 2 * k1, u, L)
    k3 = _f(x + dt / 2 * k2, u, L)
    k4 = _f(x + dt * k3, u, L)
    return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def _wrap(d):
    """Selisih sudut -> (-pi, pi]. atan2(sin, cos) mulus dan bisa diturunkan;
    operator modulo tidak, jadi tidak bisa dipakai di dalam solver."""
    return ca.atan2(ca.sin(d), ca.cos(d))


def steer_command(delta, v_ms, params):
    """delta (rad) -> perintah steer CARLA [-1, 1].

    Dua hal yang harus benar sekaligus:

    1. TANDA. delta right-handed positif = belok KIRI; steer CARLA positif =
       belok KANAN. Tanpa negasi ini MPC mengoreksi ke arah yang salah dan
       errornya membesar sendiri sampai mobil keluar jalan.
    2. PENYEBUT. delta_max_phys x curve(v), BUKAN delta_max constraint solver.
       Bagian 7.5 menulis delta/delta_max -- itu understeer ~2,4x.

    Sumbu-x steering_curve dalam km/jam (ditentukan empiris di Tahap 1).
    """
    kurva = np.asarray(params['steering_curve'], dtype=float)
    skala = float(np.interp(abs(v_ms) * 3.6, kurva[:, 0], kurva[:, 1]))
    return float(np.clip(-delta / (params['delta_max_phys'] * skala), -1.0, 1.0))


class ThrottlePI:
    """a_ref (m/s²) -> throttle/brake, tanpa tabel kalibrasi.

    Mengukur percepatan aktual dan menyesuaikan sendiri, jadi tidak perlu tahu
    hubungan throttle->percepatan. Kalau terbukti kurang responsif, ganti dengan
    throttle map terkalibrasi (bagian 7.5).
    """

    def __init__(self, kp=None, ki=None):
        self.kp = config.THROTTLE_KP if kp is None else kp
        self.ki = config.THROTTLE_KI if ki is None else ki
        self.i = 0.0

    def update(self, a_ref, a_ukur, dt):
        if a_ref < 0.0:                                  # mengerem: rem murni
            self.i = 0.0
            return 0.0, float(np.clip(-a_ref / abs(config.A_MIN), 0.0, 1.0))

        err = a_ref - a_ukur
        keluaran = self.kp * a_ref + self.i + self.kp * err
        if 0.0 < keluaran < 1.0:                         # anti-windup
            self.i += self.ki * err * dt
            self.i = float(np.clip(self.i, -0.5, 0.8))
        return float(np.clip(keluaran, 0.0, 1.0)), 0.0


class MPCController:
    def __init__(self, params, n_obs=None, bobot=None):
        # `bobot` menimpa nilai config tanpa mengeditnya -- dipakai tuning.py
        # untuk menyapu satu bobot sambil yang lain tetap.
        b = dict(Q=config.MPC_Q, Qf_scale=config.MPC_QF_SCALE, R=config.MPC_R,
                 Rd=config.MPC_RD, rho=config.MPC_RHO,
                 kp=config.THROTTLE_KP, ki=config.THROTTLE_KI)
        b.update(bobot or {})
        self.bobot = b
        self.params = params
        self.L = params['L']
        self.delta_max = params['delta_max']
        self.N, self.dt = config.MPC_N, config.MPC_DT
        self.n_obs = config.MPC_MAX_OBSTACLES if n_obs is None else n_obs
        self.pi = ThrottlePI(b['kp'], b['ki'])
        self._u_buffer = None
        self._x_buffer = None
        self.u_prev = np.zeros(2)
        self.last_status = None      # alasan gagal terakhir dari IPOPT
        self.last_iter = 0           # jumlah iterasi; tidak terpengaruh beban mesin

        N, dt = self.N, self.dt
        Q = np.diag(b['Q'])
        Qf = b['Qf_scale'] * Q
        R = np.diag(b['R'])
        Rd = np.diag(b['Rd'])

        opti = ca.Opti()
        X = opti.variable(4, N + 1)
        U = opti.variable(2, N)
        eps = opti.variable(self.n_obs, N + 1)

        x0 = opti.parameter(4)
        xref = opti.parameter(4, N + 1)
        uprev = opti.parameter(2)
        obs = opti.parameter(4, self.n_obs)          # x, y, vx, vy

        opti.subject_to(X[:, 0] == x0)
        biaya = 0
        for k in range(N):
            opti.subject_to(X[:, k + 1] == _rk4(X[:, k], U[:, k], self.L, dt))

            e = X[:, k] - xref[:, k]
            e = ca.vertcat(e[0], e[1], _wrap(e[2]), e[3])
            du = U[:, k] - (uprev if k == 0 else U[:, k - 1])
            biaya += ca.mtimes([e.T, Q, e]) + ca.mtimes([U[:, k].T, R, U[:, k]]) \
                + ca.mtimes([du.T, Rd, du])

            opti.subject_to(opti.bounded(config.A_MIN, U[0, k], config.A_MAX))
            opti.subject_to(opti.bounded(-self.delta_max, U[1, k], self.delta_max))
            opti.subject_to(opti.bounded(-config.DDELTA_MAX, du[1], config.DDELTA_MAX))

        eN = X[:, N] - xref[:, N]
        eN = ca.vertcat(eN[0], eN[1], _wrap(eN[2]), eN[3])
        biaya += ca.mtimes([eN.T, Qf, eN])

        for k in range(N + 1):
            # Batas kecepatan mulai k=1: X[:,0] sudah dikunci ke hasil ukur, jadi
            # membatasinya lagi berarti melarang masa lalu. Kelebihan 0,0003 m/s
            # saja sudah membuat seluruh masalah Infeasible_Problem_Detected.
            if k > 0:
                opti.subject_to(opti.bounded(0.0, X[3, k], config.V_MAX))
            for j in range(self.n_obs):
                xj = obs[0, j] + obs[2, j] * (k * dt)
                yj = obs[1, j] + obs[3, j] * (k * dt)
                g = ((X[0, k] - xj) / config.ELLIPSE_A) ** 2 \
                    + ((X[1, k] - yj) / config.ELLIPSE_B) ** 2
                # slack per obstacle, TIDAK dijumlahkan: satu kendaraan yang mepet
                # tidak boleh menghapus batas aman terhadap kendaraan lain
                opti.subject_to(g >= 1 - eps[j, k])
                opti.subject_to(eps[j, k] >= 0)
            biaya += b['rho'] * ca.sumsqr(eps[:, k])

        opti.minimize(biaya)
        opti.solver('ipopt',
                    {'print_time': False},
                    {'print_level': 0, 'sb': 'yes', 'max_iter': config.MPC_MAX_ITER,
                     'warm_start_init_point': 'yes', 'tol': config.MPC_TOL,
                     'acceptable_tol': config.MPC_TOL * 100, 'acceptable_iter': 5})

        self.opti, self.X, self.U, self.eps = opti, X, U, eps
        self.p = dict(x0=x0, xref=xref, uprev=uprev, obs=obs)

    def _obstacle_matrix(self, obstacles, x_ego):
        m = np.full((4, self.n_obs), 0.0)
        m[0, :] = PARKIR                              # slot kosong: jauh sekali
        if obstacles is not None and len(obstacles):
            o = np.asarray(obstacles, dtype=float)
            if len(o) > self.n_obs:
                # Jarak dari EGO, bukan dari titik asal path. Memakai x absolut
                # akan mengurutkan dari ujung jalan dan menyimpan kendaraan di
                # belakang sambil membuang yang di depan.
                jarak = np.hypot(o[:, 0] - x_ego[0], o[:, 1] - x_ego[1])
                o = o[np.argsort(jarak)[:self.n_obs]]
            m[:, :len(o)] = o.T
        return m

    def solve(self, x_meas, xref, obstacles=None):
        """-> (a, delta, waktu_ms, ok). xref shape (4, N+1)."""
        opti = self.opti
        opti.set_value(self.p['x0'], x_meas)
        opti.set_value(self.p['xref'], xref)
        opti.set_value(self.p['uprev'], self.u_prev)
        opti.set_value(self.p['obs'], self._obstacle_matrix(obstacles, x_meas))

        if self._x_buffer is not None:                # warm start: geser 1 langkah
            opti.set_initial(self.X, np.hstack([self._x_buffer[:, 1:],
                                                self._x_buffer[:, -1:]]))
            opti.set_initial(self.U, np.hstack([self._u_buffer[:, 1:],
                                               self._u_buffer[:, -1:]]))

        t0 = time.perf_counter()
        try:
            sol = opti.solve()
            u = np.array(sol.value(self.U))
            self._u_buffer, self._x_buffer = u, np.array(sol.value(self.X))
            a, delta, ok = float(u[0, 0]), float(u[1, 0]), True
            self.last_status = 'Solve_Succeeded'
            self.last_iter = int(opti.stats().get('iter_count', 0))
        except RuntimeError:
            try:
                self.last_status = opti.stats().get('return_status', '?')
            except Exception:
                self.last_status = 'unknown'
            # Fallback: langkah berikutnya dari solusi tersimpan, kalau tidak ada
            # rem ringan dengan setir terakhir. BUKAN input nol -- setir nol di
            # tengah pindah lajur meluruskan roda saat mobil sedang menyerong.
            if self._u_buffer is not None and self._u_buffer.shape[1] > 1:
                a, delta = float(self._u_buffer[0, 1]), float(self._u_buffer[1, 1])
                self._u_buffer = self._u_buffer[:, 1:]
            else:
                a, delta = -2.0, float(self.u_prev[1])
            self._x_buffer, ok = None, False

        ms = (time.perf_counter() - t0) * 1e3
        self.u_prev = np.array([a, delta])
        return a, delta, ms, ok

    def compute(self, x_meas, xref, a_ukur=0.0, obstacles=None, dt=None):
        """Satu tick penuh -> ControlCommand siap dikirim ke CARLA."""
        a, delta, ms, ok = self.solve(x_meas, xref, obstacles)
        throttle, brake = self.pi.update(a, a_ukur,
                                         config.FIXED_DELTA_SECONDS if dt is None else dt)
        return ControlCommand(throttle, brake,
                              steer_command(delta, x_meas[3], self.params),
                              a, delta, ms, ok)
