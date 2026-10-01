"""Uji MPC. Murni numerik, tidak butuh CARLA."""
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config                                                          # noqa: E402
import control as C                                                  # noqa: E402
from planning import plan_lane_change                                 # noqa: E402
from validate_model import bicycle_rk4                                 # noqa: E402

PARAMS = json.load(open(os.path.join(config.OUT_DIR, 'vehicle_params.json')))
V = 13.9
DT = config.FIXED_DELTA_SECONDS

# IPOPT interior-point tidak pernah mendorong eps tepat ke nol; sisa barrier
# terukur 2,2e-06 saat lapang versus 0,999 saat benar-benar melanggar.
SLACK_ZERO = 1e-3


def straight_ref(x0=0.0, y=0.0, v=V):
    """(4, N+1) garis lurus sepanjang +x pada kecepatan tetap."""
    t = np.arange(config.MPC_N + 1) * config.MPC_DT
    return np.vstack([x0 + v * t, np.full_like(t, y), np.zeros_like(t), np.full_like(t, v)])


def closed_loop(mpc, x0, seconds, obstacles=None):
    """Plant = model yang sama (perfect model): menguji logika kontroler, bukan fisika."""
    x, trace = np.array(x0, dtype=float), []
    for _ in range(int(seconds / DT)):
        a, delta, ms, ok = mpc.solve(x, straight_ref(x0=x[0]), obstacles)
        s = bicycle_rk4(np.array([x[0], x[1], x[2]]), x[3], delta, PARAMS['L'], DT)
        x = np.array([s[0], s[1], s[2], x[3] + a * DT])
        trace.append((x.copy(), a, delta, ms, ok, mpc.last_iter))
    return trace


def test_pulls_back_to_reference():
    """Mobil melenceng 1 m ke kiri -> setir ke kanan, error menyusut."""
    mpc = C.MPCController(PARAMS)
    trace = closed_loop(mpc, [0.0, 1.0, 0.0, V], 2.5)
    assert trace[0][2] < 0, f'steering must go right (negative delta), got {trace[0][2]}'
    y_start, y_end = abs(trace[0][0][1]), abs(trace[-1][0][1])
    assert y_end < 0.25 * y_start, (y_start, y_end)
    assert all(ok for *_, ok in trace), 'solver failed mid-loop'


def test_idle_when_already_on_target():
    """Sudah di acuan -> setir hampir nol dan tidak berosilasi."""
    mpc = C.MPCController(PARAMS)
    trace = closed_loop(mpc, [0.0, 0.0, 0.0, V], 1.5)
    delta = np.array([d for _, _, d, *_ in trace])
    assert np.abs(delta).max() < 5e-3, np.abs(delta).max()
    assert np.abs(np.diff(delta)).max() < 2e-3, 'steering oscillates'


def test_respects_steering_rate_limit():
    mpc = C.MPCController(PARAMS)
    trace = closed_loop(mpc, [0.0, 2.5, 0.0, V], 2.0)
    delta = np.array([d for _, _, d, *_ in trace])
    step_max = np.abs(np.diff(delta)).max()
    # perintah keluar tiap 50 ms, batasnya per langkah MPC 100 ms
    assert step_max <= config.DDELTA_MAX * 1.05, step_max
    assert np.abs(delta).max() <= PARAMS['delta_max'] + 1e-6


def test_brakes_when_obstacle_enters_horizon():
    """MPC BUKAN penghindar halangan -- dia penjejak dengan batas aman.

    Halangan tepat di depan pada y=0 membuat elipsnya simetris: belok kiri dan
    kanan berbiaya identik, dan Q[Y] menghukum keduanya. Solver diam di titik
    stasioner simetris (delta = 0) dan memperlambat. Yang memutuskan lewat sisi
    mana adalah planner, lewat lintasan acuan yang dihasilkannya.
    """
    mpc = C.MPCController(PARAMS)
    trace = closed_loop(mpc, [0.0, 0.0, 0.0, V], 2.0,
                          obstacles=[[35.0, 0.0, 0.0, 0.0]])
    v = np.array([s[3] for s, *_ in trace])
    a = np.array([a for _, a, *_ in trace])
    assert v[-1] < V - 1.0, f'did not slow down: {V:.1f} -> {v[-1]:.2f} m/s'
    assert a.min() < -1.0, a.min()

    without = closed_loop(C.MPCController(PARAMS), [0.0, 0.0, 0.0, V], 2.0)
    assert abs(without[-1][0][3] - V) < 0.05, 'without obstacles it should keep going'


def test_tracks_planner_path_past_obstacle():
    """Integrasi Tahap 3 + 5: planner memilih jalur, MPC mengikutinya."""
    import planning
    traj, _ = planning.plan_lane_change(0, 0, 0, 0, V, 0, V,
                                        obstacles=[[45.0, 0.0, 0.0, 0.0]])
    assert traj is not None

    def xref_at(t0):
        t = t0 + np.arange(config.MPC_N + 1) * config.MPC_DT
        return np.column_stack([traj.sample_at(tt) for tt in t])

    mpc = C.MPCController(PARAMS)
    x, err, g_min = np.array([0.0, 0.0, 0.0, V]), [], []
    for i in range(int(3.0 / DT)):
        t0 = i * DT
        a, delta, _, ok = mpc.solve(x, xref_at(t0), [[45.0, 0.0, 0.0, 0.0]])
        assert ok, f'solver failed at tick {i}'
        ref_value = traj.sample_at(t0)
        err.append(math.hypot(x[0] - ref_value[0], x[1] - ref_value[1]))
        g_min.append(planning.safety_zone(x[0] + config.AXLE_TO_CENTER * math.cos(x[2]) - 45.0,
                                        x[1] + config.AXLE_TO_CENTER * math.sin(x[2])))
        s = bicycle_rk4(x[:3], x[3], delta, PARAMS['L'], DT)
        x = np.array([s[0], s[1], s[2], x[3] + a * DT])

    print(f'      XTE mean {np.mean(err):.3f} m, max {np.max(err):.3f} m; '
          f'g_min {np.min(g_min):.2f}')
    assert np.max(err) < 0.5, np.max(err)
    assert x[1] < -2.5, f'did not reach the target lane, y = {x[1]:.2f}'
    assert np.min(g_min) >= 1.0, np.min(g_min)


def test_still_answers_when_no_safe_solution():
    """Kendaraan tepat di depan: solver tidak boleh mengembalikan kosong."""
    mpc = C.MPCController(PARAMS)
    a, delta, ms, ok = mpc.solve(np.array([0.0, 0.0, 0.0, V]), straight_ref(),
                                 obstacles=[[3.0, 0.0, 0.0, 0.0]])
    assert np.isfinite(a) and np.isfinite(delta), (a, delta)
    slack = np.array(mpc.opti.debug.value(mpc.eps))
    assert slack.max() > SLACK_ZERO, 'slack should be active when the constraint is violated'


def test_speed_slightly_above_v_max_still_solvable():
    """State awal dikunci ke hasil ukur; batas kecepatan tidak boleh berlaku di
    k=0. Kelebihan 0,0003 m/s membuat solver infeasible di run tertutup pertama.
    """
    mpc = C.MPCController(PARAMS)
    for excess in (0.0003, 0.05, 0.5):
        a, delta, _, ok = mpc.solve(np.array([0.0, 0.0, 0.0, config.V_MAX + excess]),
                                    straight_ref(v=config.V_MAX))
        assert ok, f'failed at v = V_MAX + {excess}'
        assert np.isfinite(a) and np.isfinite(delta)


def test_full_slots_keep_obstacles_nearest_to_ego():
    """Slot terbatas -> harus menyimpan yang terdekat dari EGO.

    Mengurutkan pakai x absolut berarti mengurutkan dari titik asal path. Bedanya
    baru terlihat saat halangan mengapit ego: urutan absolut menyimpan yang di
    belakang dan membuang yang di depan, padahal yang di depan lebih dekat.
    """
    mpc = C.MPCController(PARAMS, n_obs=2)
    x_ego = np.array([500.0, 0.0, 0.0, V])
    obs = [[486.0, 0.0, 9.0, 0.0],      # 14 m di belakang
           [505.0, -3.5, 7.0, 0.0],     #  5 m di depan
           [512.0, -3.5, 7.0, 0.0]]     # 12 m di depan
    stored = sorted(mpc._obstacle_matrix(obs, x_ego)[0, :2])
    assert stored == [505.0, 512.0], stored
    order_absolute = sorted(np.asarray(obs)[np.argsort([o[0] for o in obs])][:2, 0])
    assert order_absolute != stored, 'test case does not distinguish the two orderings'


def test_slack_zero_when_clear():
    mpc = C.MPCController(PARAMS)
    mpc.solve(np.array([0.0, 0.0, 0.0, V]), straight_ref(),
              obstacles=[[300.0, 0.0, 0.0, 0.0]])
    assert np.array(mpc.opti.debug.value(mpc.eps)).max() < SLACK_ZERO


def test_solver_iteration_count_reasonable():
    """Yang di-assert jumlah ITERASI, bukan waktu jam dinding.

    Waktu solve bergantung beban mesin: konfigurasi yang sama terukur 26 ms saat
    mesin senggang dan 70 ms saat CARLA memakai 141% CPU. Uji berbasis waktu
    akan gagal karena lingkungan, bukan karena kode, dan itu melatih orang
    mengabaikan kegagalan. Jumlah iterasi murni algoritmik.

    Waktu solve tetap dicetak sebagai informasi -- klaim real-time untuk skripsi
    diukur terpisah di mesin yang senggang.
    """
    mpc = C.MPCController(PARAMS)
    trace = closed_loop(mpc, [0.0, 1.0, 0.0, V], 2.0)
    ms = np.array([j[3] for j in trace])
    it = np.array([j[5] for j in trace])
    print(f'      iterations: mean {it[1:].mean():.1f}, max {it[1:].max()}  |  '
          f'solve {ms[1:].mean():.1f} ms (informational only)')
    assert it[1:].mean() < 40, it[1:].mean()
    assert it.max() < config.MPC_MAX_ITER, it.max()


def test_warm_start_speeds_up():
    cold = []
    for _ in range(6):
        m = C.MPCController(PARAMS)          # instance baru = tanpa buffer
        cold.append(m.solve(np.array([0.0, 1.0, 0.0, V]), straight_ref())[2])
    warm = [t[3] for t in closed_loop(C.MPCController(PARAMS), [0.0, 1.0, 0.0, V], 0.5)][1:]
    print(f'      cold {np.median(cold):.1f} ms  vs  warm {np.median(warm):.1f} ms')
    assert np.median(warm) < np.median(cold)


def test_angle_jump_does_not_disrupt():
    """psi acuan 0 vs 2*pi itu arah yang sama -> perintah harus sama."""
    xref = straight_ref(y=0.5)
    a1, d1, *_ = C.MPCController(PARAMS).solve(np.array([0.0, 0.0, 0.0, V]), xref)
    xref2 = xref.copy(); xref2[2, :] += 2 * math.pi
    a2, d2, *_ = C.MPCController(PARAMS).solve(np.array([0.0, 0.0, 0.0, V]), xref2)
    assert abs(d1 - d2) < 1e-6, (d1, d2)
    assert abs(a1 - a2) < 1e-6, (a1, a2)


def test_steer_flips_sign():
    """delta RH positif = belok kiri; steer CARLA positif = belok kanan.

    Salah tanda di sini membuat MPC mengoreksi ke arah yang salah dan error
    membesar sendiri sampai mobil keluar jalan. Terjadi di run tertutup pertama.
    """
    assert C.steer_command(+0.1, V, PARAMS) < 0
    assert C.steer_command(-0.1, V, PARAMS) > 0
    assert abs(C.steer_command(0.0, V, PARAMS)) < 1e-12


def test_steer_uses_physical_delta_max():
    """Bagian 7.5 menulis delta/delta_max -- itu salah, understeer ~2,4x."""
    delta = 0.1
    s = abs(C.steer_command(delta, V, PARAMS))
    wrong = delta / PARAMS['delta_max']
    assert s < wrong, (s, wrong)
    curve = np.asarray(PARAMS['steering_curve'])
    scale = np.interp(V * 3.6, curve[:, 0], curve[:, 1])
    assert abs(s - delta / (PARAMS['delta_max_phys'] * scale)) < 1e-9
    assert abs(C.steer_command(10.0, V, PARAMS)) <= 1.0        # ter-clip


def test_throttle_pi():
    pi = C.ThrottlePI()
    t, b = pi.update(-3.0, 0.0, DT)                 # minta perlambatan
    assert t == 0.0 and abs(b - 0.5) < 1e-9, (t, b)
    pi = C.ThrottlePI()
    t, b = pi.update(2.0, 0.0, DT)                  # minta percepatan
    assert t > 0 and b == 0.0
    for _ in range(40):                             # integral menutup error
        t, _ = pi.update(2.0, 0.5, DT)
    assert t > 0.2, t
    assert 0.0 <= t <= 1.0
    # regresi: saat jelajah, a_ref -0,0016 (praktis nol) dulu memutus throttle
    # dan me-reset integrator -> kecepatan anjlok. Split-range: tetap throttle.
    pi.i = 0.4
    t, b = pi.update(-0.0016, 0.0, DT)
    assert t > 0.35 and b == 0.0 and pi.i > 0.39, (t, b, pi.i)


def test_command_complete():
    mpc = C.MPCController(PARAMS)
    cmd = mpc.compute(np.array([0.0, 0.8, 0.0, V]), straight_ref(), a_meas=0.0)
    assert isinstance(cmd, C.ControlCommand) and cmd.solver_ok
    assert -1.0 <= cmd.steer <= 1.0 and 0.0 <= cmd.throttle <= 1.0
    assert 0.0 <= cmd.brake <= 1.0 and cmd.solve_time_ms > 0
    assert not (cmd.throttle > 0 and cmd.brake > 0), 'throttle and brake at the same time'


def test_lateral_acceleration_kept_near_planner_limit():
    """Ruang kelayakan MPC harus sedekat mungkin dengan planner (bagian 19.9).

    Dirangsang dengan acuan dari PLANNER SUNGGUHAN, bukan tangga. Sejak acuan
    cadangan menahan `y` berjalan (bagian 19.14), MPC tidak pernah lagi menerima
    tangga; menguji dengan tangga berarti mengukur rangsangan yang tidak terjadi
    -- tangga 3,5 m memberi 6,1 m/s^2, sementara loop tertutup terukur 1,29 (GT)
    dan 3,34 (vision). Ini persis pola bagian 15.5: metrik yang mengukur hal lain.

    Batasnya LUNAK (bagian 19.12): `MAX_LATERAL_ACCEL` itu batas kenyamanan, dan
    sebagai constraint keras ia mengalahkan pelebaran jarak saat berpapasan.
    """
    traj, _ = plan_lane_change(0.0, 0.0, 0.0, 0.0, V, 0.0, V)
    assert traj is not None, 'planner gave no plan for an empty lane'
    mpc = C.MPCController(PARAMS)
    x = np.array([0.0, 0.0, 0.0, V])
    worst = 0.0
    for i in range(int(traj.duration() / config.MPC_DT)):
        t0 = i * config.MPC_DT
        xref = np.column_stack([traj.sample_at(t0 + j * config.MPC_DT)
                                for j in range(config.MPC_N + 1)])
        a, delta, _, ok = mpc.solve(x, xref)
        assert ok, f'solver failed at step {i}'
        worst = max(worst, abs(x[3] ** 2 * np.tan(delta) / PARAMS['L']))
        s = bicycle_rk4(np.array([x[0], x[1], x[2]]), x[3], delta, PARAMS['L'],
                        config.MPC_DT)
        x = np.array([s[0], s[1], s[2], x[3] + a * config.MPC_DT])
    assert worst <= config.MAX_LATERAL_ACCEL * 1.1, f'a_lat {worst:.2f} m/s2'


def test_lateral_slack_actually_binds():
    """Batas lunak harus tetap MENGIKAT, bukan hiasan. Diberi tangga -- justru
    kasus terburuk -- hasilnya wajib jauh di bawah kemampuan kemudi penuh."""
    mpc = C.MPCController(PARAMS)
    x = np.array([0.0, 0.0, 0.0, V])
    worst = 0.0
    for _ in range(20):
        tt = np.arange(config.MPC_N + 1) * config.MPC_DT
        xref = np.vstack([x[0] + V * tt, np.full_like(tt, -3.5),
                          np.zeros_like(tt), np.full_like(tt, V)])
        a, delta, _, ok = mpc.solve(x, xref)
        assert ok
        worst = max(worst, abs(x[3] ** 2 * np.tan(delta) / PARAMS['L']))
        s = bicycle_rk4(np.array([x[0], x[1], x[2]]), x[3], delta, PARAMS['L'], DT)
        x = np.array([s[0], s[1], s[2], x[3] + a * DT])
    free = V ** 2 * np.tan(PARAMS['delta_max']) / PARAMS['L']
    assert worst < free / 4.0, f'slack too loose: {worst:.2f} vs free {free:.1f}'


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            fn()
            print(f'ok  {name}')
    print('all passed')
