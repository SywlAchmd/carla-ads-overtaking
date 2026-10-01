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

    def duration(self):
        """Lama rencana, detik."""
        return (self.states.shape[1] - 1) * self.dt

    def lateral_at(self, t):
        """(y, y\', y\'\') eksak dari polinomial.

        Dipakai saat replan receding horizon: state lateral harus diteruskan
        persis, bukan hasil diferensiasi numerik dari sampel.
        """
        # Dijepit ke durasi rencana: polinomial derajat lima MELEDAK di luar
        # selangnya, dan sejak rencana dipertahankan saat replan gagal
        # (bagian 19.14) `t` memang bisa melewatinya. `sample_at` sudah aman
        # sendiri karena memakai np.interp.
        t = min(max(float(t), 0.0), self.duration())
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


def safety_zone(dx, dy, A=None, B=None):
    """g >= 1 berarti aman. dx, dy = pusat bodi ego ke pusat kendaraan lain.

    Elips-super ((dx/A)^p + (dy/B)^p)^(1/p); dipakai bersama planner (numpy) dan
    MPC (CasADi). Akar ke-p membuat g berskala seperti jarak: tanpa itu slot kosong
    MPC (diparkir 1e4 m) memberi gradien ~1e9. p genap, jadi tanda dx/dy tak penting.

    `A`, `B` bisa berupa simbol CasADi supaya jalur vision memasok zona dari
    dimensi yang diukurnya sendiri (bagian 28.2). Bawaannya konstanta dimensi
    bounding box simulator, sehingga jalur ground truth tidak berubah sama sekali.
    """
    p = config.ELLIPSE_P
    A = config.ELLIPSE_A if A is None else A
    B = config.ELLIPSE_B if B is None else B
    return ((dx / A) ** p + (dy / B) ** p + 1e-12) ** (1.0 / p)


def _ellipse_g(states, obstacles, zone=None):
    """g_j untuk tiap obstacle di tiap sampel. g >= 1 berarti aman (bagian 7.2)."""
    if obstacles is None or len(obstacles) == 0:
        return None
    n = states.shape[1]
    k = np.arange(n) * config.PLANNER_DT
    obs = np.asarray(obstacles, dtype=float)          # (M, 4) = [x, y, vx, vy]
    xj = obs[:, 0:1] + obs[:, 2:3] * k                # prediksi kecepatan konstan
    yj = obs[:, 1:2] + obs[:, 3:4] * k
    # states = sumbu belakang; zona diukur dari pusat bodi
    xc = states[0] + config.AXLE_TO_CENTER * np.cos(states[2])
    yc = states[1] + config.AXLE_TO_CENTER * np.sin(states[2])
    return safety_zone(xc - xj, yc - yj, *(zone or (None, None)))


def _cost(t, states, ddy, T, y_lane_center, v_desired, cy, cx, g):
    jerk_lat = np.trapezoid(_val(_deriv(cy, 3), t)**2, t)
    jerk_lon = np.trapezoid(_val(_deriv(cx, 3), t)**2, t)

    j_lat = jerk_lat + config.K_TIME * T + config.K_DEV * (states[1, -1] - y_lane_center)**2
    j_lon = jerk_lon + config.K_VEL * (states[3, -1] - v_desired)**2
    j_col = 0.0 if g is None else float(1.0 / g.min())
    return config.W_LAT * j_lat + config.W_LON * j_lon + config.W_COL * j_col


def plan_lane_change(y0, dy0, ddy0, x0, v0, a0, v_desired, obstacles=None,
                     side_sign=None, dt=None, y_goal=None, zone=None, lane_width=None):
    """Bangkitkan kandidat, saring yang tidak layak, kembalikan (terbaik, semua_layak).

    Mengembalikan (None, []) bila tidak ada kandidat yang lolos -- FSM harus
    memperlakukan itu sebagai abort, bukan error.
    """
    side_sign = config.SIDE_SIGN if side_sign is None else side_sign
    dt = config.PLANNER_DT if dt is None else dt
    # Lebar lajur hasil ukur bila jalur vision memasoknya (bagian 28.3); kalau
    # tidak, konstanta peta seperti sebelumnya.
    lw = config.LANE_WIDTH if lane_width is None else lane_width
    # y_goal = tengah lajur tujuan (absolut). Diperlukan untuk manuver kembali,
    # yang targetnya 0 dan tidak bisa dinyatakan sebagai side_sign * LANE_WIDTH.
    y_lane_center = side_sign * lw if y_goal is None else y_goal
    direction = 1.0 if y_lane_center >= y0 else -1.0
    kappa_max = 1.0 / config.MIN_TURN_RADIUS

    feasible = []
    for offset in config.LATERAL_OFFSETS:
        # Dikurangi LANE_WIDTH nominal, BUKAN lebar lajur hasil ukur. `LATERAL_OFFSETS`
        # adalah magnitudo terhadap lebar lajur nominal, jadi selisihnya = {-0,5; 0;
        # +0,5} m dari tengah lajur tujuan -- dan kandidat tengahnya tepat di tengah.
        # Memakai `lw` hasil ukur menggeser seluruh kisi kandidat sebesar galat ukur
        # lebar lajur, sehingga TIDAK ADA kandidat yang jatuh di tengah lajur:
        # terukur planner membidik 0,125 m dari tengah, dan MPC mengikutinya dengan
        # tepat (galat lacak 0,0001 m). Regresi yang masuk bersama bagian 28, saat
        # lebar lajur berubah dari konstanta menjadi hasil ukur.
        y_target = y_lane_center + direction * (offset - config.LANE_WIDTH)
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
            g = _ellipse_g(states, obstacles, zone)
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


def _in_lane(obstacles, y_lane, width=None):
    """Halangan yang pusatnya berada di dalam lajur y_lane."""
    if obstacles is None or len(obstacles) == 0:
        return np.empty((0, 4))
    obs = np.asarray(obstacles, dtype=float)
    w = config.LANE_WIDTH if width is None else width
    return obs[np.abs(obs[:, 1] - y_lane) < w / 2.0]


def _ttc(front, v_ego):
    """Waktu sampai menyusul kendaraan depan. inf bila tidak sedang mendekat."""
    dv = v_ego - front[2]
    return front[0] / dv if dv > 1e-3 else float('inf')


def _leading(obs):
    """Halangan terdekat di depan (x > 0), atau None."""
    front = obs[obs[:, 0] > 0.0]
    return front[np.argmin(front[:, 0])] if len(front) else None


def _v_follow(front):
    """Kecepatan acuan untuk mengikuti kendaraan depan pada jarak ikut d*.

    d* = ELLIPSE_A + AXLE_TO_CENTER + FOLLOW_TIME·v_front (kebijakan jarak
    waktu-tetap; celah diukur dari sumbu belakang ego). Menjamin jarak pusat ke
    pusat >= ELLIPSE_A: posisi mengikuti tidak melanggar zona aman MPC sendiri.

    v = v_front + 2e/T, e = celah - d*, T = max(MANEUVER_TIMES). Diturunkan dari
    planner: quartic longitudinal mengubah kecepatan secara halus selama T, jadi
    menutup selisih dv menempuh jarak relatif dv·T/2; agar tidak melampaui e,
    dv <= 2e/T. Hukum akar dv = sqrt(2ae) sempat dipakai dan kebablasan di S3
    karena mengabaikan jeda itu: celah 14,3 m (d* 17,5), ego mundur ke 4,7 m/s.
    e < 0 (terlalu dekat) memberi kecepatan di bawah v_front: mundur ke d*.
    """
    v_front = max(float(front[2]), 0.0)
    e = float(front[0]) - (config.ELLIPSE_A + config.AXLE_TO_CENTER
                           + config.FOLLOW_TIME * v_front)
    return float(np.clip(v_front + 2.0 * e / max(config.MANEUVER_TIMES), 0.0, config.V_REF))


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
        self._pending = None            # (state tujuan, waktu permintaan pertama)
        self.last_abort = None    # untuk logging bagian 11.5
        self._passed = None            # (t, x, v_other) lajur asal terakhir TERLIHAT
        # (y tengah lajur ego di frame jalan, lebar lajur) hasil UKUR, atau None
        # untuk memakai konstanta peta seperti sebelum bagian 28.
        self._lane = None
        self._v_target = None         # laju target saat KEPUTUSAN menyalip diambil
        self.v_goal = config.V_REF    # kecepatan acuan untuk planner, lihat _v_follow

    @property
    def lane_width(self):
        return config.LANE_WIDTH if self._lane is None else self._lane[1]

    @property
    def y_origin(self):
        """Tengah lajur asal di frame jalan. 0 bila memakai konstanta peta."""
        return 0.0 if self._lane is None else self._lane[0]

    @property
    def y_goal(self):
        """Tengah lajur yang sedang dituju -- diteruskan ke plan_lane_change."""
        overtaking = self.state in (LANE_CHANGE_OVERTAKE, OVERTAKING)
        return self.y_origin + (self.side_sign * self.lane_width if overtaking else 0.0)

    def _request(self, target, t):
        """Transisi baru dieksekusi setelah diminta terus-menerus selama dwell."""
        if self._pending is None or self._pending[0] != target:
            self._pending = (target, t)
            return False
        if t - self._pending[1] >= config.FSM_DWELL:
            self.state, self._pending = target, None
            return True
        return False

    def _immediate(self, target):
        """Abort tidak menunggu dwell -- menunda 0,3 s justru menambah risiko."""
        self.state, self._pending = target, None

    def update(self, t, d, v_ego, obstacles, dd=0.0, lane=None):
        """Satu langkah FSM. `obstacles` = (M,4) [x, y, vx, vy], x relatif ego.

        `d` = y ego di frame jalan; simpangan terhadap lajur asal dihitung DI SINI
        terhadap tengah lajur hasil ukur, bukan terhadap centerline peta. Sebelum
        18 Sep 2026 `d` dipakai apa adanya, jadi ambang `LATERAL_ENTER` dan
        `LATERAL_DONE` masih diukur dari peta HD sementara `y_goal` sudah dari
        hasil ukur -- dua acuan berbeda di satu mesin keputusan. `dd` = laju
        lateral (m/s).
        `lane` = (y tengah lajur ego di frame jalan, lebar lajur) hasil UKUR dari
        kepala segmentasi YOLOPX; None berarti memakai konstanta peta seperti
        sebelum bagian 28. Kembalikan nama state.
        """
        # `lane` = (y tengah lajur ego di frame jalan, lebar) hasil ukur, atau None.
        #
        # HANYA disegarkan saat LANE_KEEPING, lalu DIKUNCI sepanjang manuver.
        # Alasannya keras: `lane_dev` mengukur simpangan dari lajur TERDEKAT,
        # jadi begitu ego menyeberang, lajur terdekat berubah menjadi lajur salip
        # dan "tengah lajur asal" ikut melompat satu lajur. `y_goal` lalu
        # menunjuk satu lajur lebih jauh lagi, dan ego mengejar sasaran yang terus
        # lari. Terukur: ego melayang sampai -14,48 m -- empat lajur -- lalu
        # menabrak tiang.
        #
        # Pelajarannya umum: ukuran RELATIF terhadap yang terdekat tidak bisa
        # mendefinisikan sasaran ABSOLUT selama manuver yang mengubah mana yang
        # terdekat. Ia harus dikunci sebelum manuver dimulai.
        if lane is not None and self.state == LANE_KEEPING:
            # DITAPIS, bukan disalin: tengah lajur adalah sifat jalan, jadi ia tidak
            # boleh melompat. Mentah, `y_goal` berkedut sampai 0,147 m antar replan
            # dan MPC mengejar acuan yang bergerigi. Tapisnya harus di SINI, dengan
            # gerbang state yang sama dengan latch-nya: ditapis di pemanggil, ia
            # ikut berjalan selama manuver -- ketika `lane_dev` mengacu ke lajur
            # SALIP -- dan lompatannya justru naik ke 0,693 m.
            if self._lane is None:
                self._lane = lane
            else:
                a = config.ALPHA_LANE_CENTER
                self._lane = tuple(old + a * (fresh - old)
                                    for old, fresh in zip(self._lane, lane))
        lw, y_origin = self.lane_width, self.y_origin
        d = d - y_origin                 # -> simpangan dari tengah lajur asal TERUKUR
        y_dest = y_origin + self.side_sign * lw
        front = _leading(_in_lane(obstacles, y_origin, lw))
        target_lane = _in_lane(obstacles, y_dest, lw)
        # Pemicu & batal memakai kecepatan yang INGIN dipakai, bukan v_ego saja.
        # Saat mengikuti, v_ego ~ v_front: TTC terhadap v_ego tak hingga dan FSM
        # tidak akan pernah menyalip ulang. Alasan menyalip adalah kendaraan depan
        # lebih lambat daripada V_REF. Saat mendekat (v_ego >= V_REF) tidak berubah.
        v_want = max(v_ego, config.V_REF)

        if self.state == LANE_KEEPING:
            if front is not None and (_ttc(front, v_want) < config.TTC_TRIGGER
                                      and v_want - front[2] > config.DV_TRIGGER):
                self._request(CHECK_OVERTAKE, t)
            else:
                self._pending = None

        elif self.state == CHECK_OVERTAKE:
            cancelled = (front is None or _ttc(front, v_want) > config.TTC_EXIT
                     or v_want - front[2] < config.DV_EXIT)
            if cancelled:
                self._request(LANE_KEEPING, t)
            elif self._target_lane_safe(target_lane) and self._enough_time(front, v_ego):
                if self._request(LANE_CHANGE_OVERTAKE, t):
                    # Target masih jauh di depan dan terlihat utuh: di sinilah
                    # lajunya paling dapat dipercaya sepanjang manuver.
                    self._v_target = float(front[2])
            else:
                self._pending = None

        elif self.state == LANE_CHANGE_OVERTAKE:
            if not self._target_lane_safe(target_lane):
                self.last_abort = t                       # jalur abort, bagian 6
                self._immediate(LANE_KEEPING)
            elif abs(d) >= config.LATERAL_ENTER * lw:
                self._request(OVERTAKING, t)
            else:
                self._pending = None

        elif self.state == OVERTAKING:
            # _leading tidak bisa dipakai di sini: dia hanya melihat x > 0,
            # sehingga kendaraan yang baru terlewati 1 m sudah dianggap hilang.
            origin = _in_lane(obstacles, y_origin, lw)
            # Daftar kosong AMBIGU: bisa "sudah terlewat PASS_MARGIN", bisa "tidak
            # terlihat". Rig satu kamera depan kehilangan target tepat saat ego
            # berdampingan, dan versi lama memperlakukan keduanya sama -- vision
            # memutuskan kembali pada -3,17 m setelah 1,10 s tanpa satu pun
            # pengukuran, sementara GT memutuskan pada -10,40 m (bagian 26.1).
            # Karena itu yang terakhir terlihat diteruskan dengan kecepatan
            # relatifnya sampai jelas terlewat; tebakan yang DINYATAKAN, bukan
            # kekosongan yang disalahartikan sebagai aman.
            if len(origin):
                j = int(np.argmax(origin[:, 0]))        # yang paling depan, paling mengikat
                # Posisi selalu disegarkan, LAJU hanya selagi target masih di depan.
                # Begitu ego sejajar, kotak deteksi terpotong tepi citra dan taksiran
                # laju Kalman memburuk: terukur -3,2 m/s padahal sesungguhnya -6,4,
                # yang menunda kembali 3 detik tanpa alasan. Saat masih di depan
                # target terlihat utuh dan lajunya terukur benar.
                # obs[:, 2] kecepatan ABSOLUT (localization.obstacles_ego_to_road).
                # Laju dari SAAT KEPUTUSAN menyalip (`self._v_target`), bukan dari
                # frame terakhir. Terukur: laju yang dibekukan belakangan meleset
                # 9,09 m/s terhadap 7,0 m/s yang sebenarnya -- 30% terlalu tinggi,
                # karena kotak deteksi sudah terpotong saat ego mendekat. Sekali
                # taksiran itu menyamai laju ego, ekstrapolasi TIDAK PERNAH
                # menyimpulkan lewat dan ego tersangkut di lajur salip sampai run
                # habis. Terjadi sungguhan, bukan kekhawatiran.
                v_other = self._v_target if self._v_target is not None else float(origin[j, 2])
                self._passed = (t, float(origin[j, 0]), v_other)
                passed = origin[j, 0] <= -config.PASS_MARGIN
            elif self._passed is not None:
                t0, x0, v_other = self._passed
                # v_ego sekarang, bukan yang dulu: laju ego diketahui persis tiap tick.
                # Dijepit supaya paling lambat -DV_EXIT: FSM hanya masuk manuver ini
                # karena target lebih lambat dari DV_TRIGGER. Kalau taksiran laju
                # belakangan berkata sebaliknya, yang keliru taksirannya, bukan
                # premisnya -- dan tanpa jepitan ini gerbangnya bisa buntu selamanya.
                dv = min(v_other - v_ego, -config.DV_EXIT)
                passed = x0 + dv * (t - t0) <= -config.PASS_MARGIN
            else:
                passed = True                          # tidak pernah ada yang dilewati
            # ponytail: jepitan -DV_EXIT di atas menjamin gerbang ini SELALU bisa
            # menyimpulkan, tetapi harganya taksiran laju yang buruk membuat
            # kesimpulannya terlambat, bukan salah. Di S1 terukur kembali pada
            # -18 m terhadap -10,4 m milik GT. Kamera belakang yang memperbaikinya,
            # bukan menyetel ulang jepitan ini.
            # Jangan mulai kembali selagi masih bergerak menjauhi lajur asal: quintic
            # kembali berangkat dengan laju itu dan kebablasan keluar. Di S3 kelima
            # run gagal mulai kembali pada 0,91-1,89 m/s menjauh; semua yang lolos
            # sudah bergerak ke arah lajur asal.
            receding = self.side_sign * dd > config.DD_RETURN
            if passed and not receding:
                self._request(LANE_CHANGE_RETURN, t)
            else:
                self._pending = None

        elif self.state == LANE_CHANGE_RETURN:
            if abs(d) < config.LATERAL_DONE:
                self._passed = self._v_target = None   # jangan terpakai di salip berikutnya
                self._request(LANE_KEEPING, t)
            else:
                self._pending = None

        # Selama belum/tidak bisa menyalip: ikuti kendaraan depan. Dulu planner
        # tetap diberi V_REF, ego terus mendekat, _enough_time gugur, FSM terjebak di
        # CHECK_OVERTAKE dan MPC terpaksa membelok menembus elips.
        # Hanya bila menyalip tidak mungkin (depan tidak cukup lambat, waktu tidak
        # cukup, atau lajur tujuan terisi). Melambat saat menyalip masih mungkin
        # membuang selisih kecepatan yang justru dipakai untuk menyalip.
        self.v_goal = config.V_REF
        if self.state in (LANE_KEEPING, CHECK_OVERTAKE) and front is not None:
            can_overtake = (v_want - front[2] > config.DV_TRIGGER
                          and self._enough_time(front, v_ego)
                          and self._target_lane_safe(target_lane))
            if not can_overtake:
                self.v_goal = _v_follow(front)
        return self.state

    def _target_lane_safe(self, target_lane):
        if len(target_lane) == 0:
            return True
        x = target_lane[:, 0]
        return not (((x >= 0) & (x < config.D_SAFE_FRONT)).any()
                    or ((x < 0) & (x > -config.D_SAFE_REAR)).any())

    def _enough_time(self, front, v_ego):
        """Manuver tercepat harus selesai sebelum ego menyusul kendaraan depan."""
        return min(config.MANEUVER_TIMES) < _ttc(front, v_ego)
