"""Uji behavior FSM terhadap tabel transisi bagian 6. Tidak butuh CARLA."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config                                                         # noqa: E402
import planning as P                                                  # noqa: E402

V_EGO, V_SLOW = 13.9, 7.0
Y_OVERTAKE = config.SIDE_SIGN * config.LANE_WIDTH


def front(x, v=V_SLOW):
    return [x, 0.0, v, 0.0]


def in_overtake_lane(x, v=V_EGO):
    return [x, Y_OVERTAKE, v, 0.0]


def road(fsm, duration, d, obs, t0=0.0, dt=0.05, v_ego=V_EGO, **kw):
    """Jalankan FSM selama `duration` detik dengan input tetap."""
    t = t0
    for _ in range(int(round(duration / dt))):
        fsm.update(t, d, v_ego, obs, **kw)
        t += dt
    return fsm.state


def test_stays_lane_keeping_without_obstacles():
    fsm = P.BehaviorFSM()
    assert road(fsm, 3.0, 0.0, []) == P.LANE_KEEPING


def test_not_triggered_when_lead_not_slow_enough():
    fsm = P.BehaviorFSM()
    assert road(fsm, 3.0, 0.0, [front(15.0, V_EGO - 1.0)]) == P.LANE_KEEPING


def test_not_triggered_while_still_far():
    fsm = P.BehaviorFSM()
    assert road(fsm, 3.0, 0.0, [front(60.0)]) == P.LANE_KEEPING


def test_dwell_time_delays_transition():
    """0,25 s belum cukup; 0,35 s sudah (ambang FSM_DWELL = 0,3 s)."""
    obs = [front(24.0)]
    assert road(P.BehaviorFSM(), 0.25, 0.0, obs) == P.LANE_KEEPING
    assert road(P.BehaviorFSM(), 0.35, 0.0, obs) == P.CHECK_OVERTAKE


def test_check_overtake_proceeds_when_target_lane_empty():
    fsm = P.BehaviorFSM()
    assert road(fsm, 1.0, 0.0, [front(24.0)]) == P.LANE_CHANGE_OVERTAKE


def test_origin_lane_tracked_through_maneuver_without_jumping():
    """Frame berputar pelan selama manuver: tengah lajur asal bergeser 0,3 m.

    Ukurannya datang dari lajur TERDEKAT -- lajur salip, atau berganti-ganti saat
    ego tepat di atas garis. Lajur asal harus ikut bergeser 0,3 m, bukan dibekukan
    di 0 dan bukan melompat satu lajur.
    """
    w = config.LANE_WIDTH
    fsm = P.BehaviorFSM()
    road(fsm, 1.0, 0.0, [], lane=(0.0, w))                     # terkunci di 0
    fsm.state = P.OVERTAKING
    shift = 0.3
    for k in range(200):
        nearest = shift + (Y_OVERTAKE if k % 3 else 0.0)        # salip, kadang asal
        fsm.update(1.0 + k * 0.1, Y_OVERTAKE, V_EGO, [front(30.0)], lane=(nearest, w))
        assert abs(fsm.y_origin) < 0.5, fsm.y_origin            # tidak pernah melompat
    assert abs(fsm.y_origin - shift) < 0.01, fsm.y_origin


def test_check_overtake_held_when_target_lane_not_drivable():
    """Tak ada deteksi BUKAN bukti lajurnya ada: area jalan harus menandainya."""
    held, free = P.BehaviorFSM(), P.BehaviorFSM()
    assert road(held, 1.0, 0.0, [front(24.0)], drivable=0.0) == P.CHECK_OVERTAKE
    assert road(free, 1.0, 0.0, [front(24.0)], drivable=1.0) == P.LANE_CHANGE_OVERTAKE


def test_check_overtake_held_when_target_lane_occupied():
    fsm = P.BehaviorFSM()
    obs = [front(24.0), in_overtake_lane(10.0)]
    assert road(fsm, 2.0, 0.0, obs) == P.CHECK_OVERTAKE


def test_check_overtake_held_when_vehicle_approaches_from_behind():
    fsm = P.BehaviorFSM()
    obs = [front(24.0), in_overtake_lane(-8.0)]
    assert road(fsm, 2.0, 0.0, obs) == P.CHECK_OVERTAKE


def test_check_overtake_held_when_time_insufficient():
    """TTC 10/6.9 = 1,45 s, lebih pendek dari manuver tercepat 3,0 s."""
    fsm = P.BehaviorFSM()
    assert road(fsm, 2.0, 0.0, [front(10.0)]) == P.CHECK_OVERTAKE


def test_hysteresis_back_to_lane_keeping():
    """Kendaraan depan mempercepat -> batal, memakai ambang keluar bukan masuk."""
    fsm = P.BehaviorFSM()
    fsm.state = P.CHECK_OVERTAKE
    assert road(fsm, 1.0, 0.0, [front(24.0, V_EGO - 1.0)]) == P.LANE_KEEPING


def test_no_chatter_around_threshold():
    """Tepat di ambang masuk: sekali masuk CHECK, tidak keluar lagi."""
    dv = V_EGO - V_SLOW
    gap_threshold = config.TTC_TRIGGER * dv
    fsm = P.BehaviorFSM()
    road(fsm, 1.0, 0.0, [front(gap_threshold - 1.0)])
    order = []
    for k in range(60):                       # celah bergoyang di sekitar ambang
        gap = gap_threshold + (2.0 if k % 2 else -2.0)
        fsm.update(1.0 + k * 0.05, 0.0, V_EGO, [front(gap)])
        order.append(fsm.state)
    assert P.LANE_KEEPING not in order, set(order)


def test_threshold_adapts_to_speed_difference():
    """Inti pemicu TTC: celah pemicu ikut selisih kecepatan, bukan tetap."""
    result = {}
    for v_slow in (11.0, 7.0, 4.0):                  # dv = 2.9, 6.9, 9.9
        dv = V_EGO - v_slow
        for gap in range(70, 4, -1):          # dari jauh ke dekat: cari celah terbesar
            fsm = P.BehaviorFSM()             # yang masih memicu = ambangnya
            if road(fsm, 0.5, 0.0, [front(float(gap), v_slow)]) != P.LANE_KEEPING:
                result[round(dv, 1)] = gap
                break
    assert 2.9 not in result, 'dv < DV_TRIGGER should never trigger'
    assert result[6.9] < result[9.9], result     # makin cepat mendekat, makin jauh dipicu
    for dv, gap in result.items():
        assert abs(gap / dv - config.TTC_TRIGGER) < 0.6, (dv, gap, gap / dv)


def test_enter_overtaking_at_90_percent_lane_width():
    fsm = P.BehaviorFSM()
    fsm.state = P.LANE_CHANGE_OVERTAKE
    obs = [front(12.0)]
    d_short = 0.8 * config.LANE_WIDTH * config.SIDE_SIGN
    assert road(fsm, 1.0, d_short, obs) == P.LANE_CHANGE_OVERTAKE
    d_enough = 0.95 * config.LANE_WIDTH * config.SIDE_SIGN
    assert road(fsm, 1.0, d_enough, obs) == P.OVERTAKING


def test_abort_is_immediate_without_dwell():
    """Kendaraan muncul di lajur tujuan saat pindah lajur -> batal seketika."""
    fsm = P.BehaviorFSM()
    fsm.state = P.LANE_CHANGE_OVERTAKE
    fsm.update(0.0, -1.0 * -config.SIDE_SIGN, V_EGO,
               [front(12.0), in_overtake_lane(5.0)])
    assert fsm.state == P.LANE_KEEPING          # satu tick saja, bukan 0,3 detik
    assert fsm.last_abort == 0.0


def test_return_after_leading_by_pass_margin():
    fsm = P.BehaviorFSM()
    fsm.state = P.OVERTAKING
    d = Y_OVERTAKE
    pending = -(config.PASS_MARGIN - 2.0)
    assert road(fsm, 1.0, d, [front(pending)]) == P.OVERTAKING
    done = -(config.PASS_MARGIN + 2.0)
    assert road(fsm, 1.0, d, [front(done)]) == P.LANE_CHANGE_RETURN


def test_no_return_while_moving_away_from_origin_lane():
    """Target sudah tertinggal, tapi ego masih bergerak keluar 1 m/s: tunggu.
    Begitu laju lateralnya berhenti, kembali seperti biasa."""
    fsm = P.BehaviorFSM()
    fsm.state = P.OVERTAKING
    done = [front(-(config.PASS_MARGIN + 2.0))]
    for i in range(20):
        fsm.update(i * 0.05, Y_OVERTAKE, V_EGO, done, config.SIDE_SIGN * 1.0)
    assert fsm.state == P.OVERTAKING
    assert road(fsm, 1.0, Y_OVERTAKE, done, t0=1.0) == P.LANE_CHANGE_RETURN


def test_no_return_when_obstacle_leaves_view():
    """REGRESI bagian 26.1. Daftar kosong BUKAN bukti sudah terlewat.

    Rig satu kamera depan kehilangan target tepat saat ego berdampingan. Versi
    lama membaca `len(...) == 0` sebagai "aman" dan memutuskan kembali pada
    -3,17 m, padahal syaratnya 8 m.
    """
    fsm = P.BehaviorFSM()
    fsm.state = P.OVERTAKING
    # terlihat terakhir +4,2 m di depan, laju relatif -6,4 m/s (13,4 vs 7,0)
    fsm.update(0.0, Y_OVERTAKE, V_EGO, [front(4.2, V_EGO - 6.4)])
    assert road(fsm, 1.0, Y_OVERTAKE, [], t0=0.05) == P.OVERTAKING


def test_return_after_extrapolation_passes_pass_margin():
    """Lanjutan: begitu perhitungan mati menyimpulkan sudah unggul 8 m, kembali.

    (4,2 + 8,0) / 6,4 = 1,91 s sejak terakhir terlihat, ditambah FSM_DWELL.
    """
    fsm = P.BehaviorFSM()
    fsm.state = P.OVERTAKING
    fsm.update(0.0, Y_OVERTAKE, V_EGO, [front(4.2, V_EGO - 6.4)])
    assert road(fsm, 1.8, Y_OVERTAKE, [], t0=0.05) == P.OVERTAKING
    assert road(fsm, 0.6, Y_OVERTAKE, [], t0=1.85) == P.LANE_CHANGE_RETURN


def test_empty_list_from_start_still_allows_return():
    """Tidak pernah ada apa pun di lajur asal -> tidak ada yang perlu dilewati.
    Perilaku lama dipertahankan; yang berubah hanya kasus PERNAH terlihat."""
    fsm = P.BehaviorFSM()
    fsm.state = P.OVERTAKING
    assert road(fsm, 1.0, Y_OVERTAKE, []) == P.LANE_CHANGE_RETURN


def test_ground_truth_behaviour_unchanged():
    """Selama halangan selalu terlihat, gerbang baru identik dengan yang lama:
    `max(x) <= -PASS_MARGIN` sama dengan `tidak present x > -PASS_MARGIN`. Itulah
    sebabnya seluruh angka jalur ground truth tidak perlu diukur ulang."""
    for dx, expected in ((-2.0, P.OVERTAKING), (+2.0, P.LANE_CHANGE_RETURN)):
        fsm = P.BehaviorFSM()
        fsm.state = P.OVERTAKING
        obs = [front(-(config.PASS_MARGIN + dx)), front(-(config.PASS_MARGIN + 20.0))]
        assert road(fsm, 1.0, Y_OVERTAKE, obs) == expected, dx


def test_speed_frozen_at_overtake_decision():
    """REGRESI. Laju target dibekukan saat KEPUTUSAN menyalip diambil, di mana ia
    masih jauh di depan dan terlihat utuh -- bukan dari frame terakhir, di mana
    kotaknya sudah terpotong tepi citra.

    Terukur di run sungguhan: laju yang dibekukan belakangan memberi 9,09 m/s
    terhadap 7,0 m/s yang benar.
    """
    fsm = P.BehaviorFSM()
    obs = [front(30.0, V_SLOW), in_overtake_lane(60.0)]
    road(fsm, 3.0, 0.0, obs)
    assert fsm.state == P.LANE_CHANGE_OVERTAKE, fsm.state
    assert fsm._v_target is not None
    assert abs(fsm._v_target - V_SLOW) < 1e-9, fsm._v_target

    # laju buruk yang terukur belakangan tidak boleh menggantikannya
    fsm.state = P.OVERTAKING
    fsm.update(9.0, Y_OVERTAKE, V_EGO, [front(2.0, V_EGO + 2.0)])
    assert abs(fsm._passed[2] - V_SLOW) < 1e-9, fsm._passed


def test_return_gate_cannot_deadlock_forever():
    """REGRESI. Kalau taksiran laju target menyamai atau melampaui laju ego,
    ekstrapolasi tidak akan pernah menyimpulkan "lewat" dan ego tersangkut di
    lajur salip sampai run habis -- terjadi sungguhan pada 18 Sep 2026.

    Laju relatif karena itu dijepit ke paling lambat -DV_EXIT: FSM hanya masuk
    manuver ini karena target lebih lambat dari DV_TRIGGER.
    """
    fsm = P.BehaviorFSM()
    fsm.state = P.OVERTAKING
    # taksiran laju target SAMA DENGAN ego: tanpa jepitan, dv = 0 selamanya
    fsm.update(0.0, Y_OVERTAKE, V_EGO, [front(2.0, V_EGO)])
    assert fsm.state == P.OVERTAKING
    # (2,0 + 8,0) / 1,5 = 6,67 s, lalu FSM_DWELL
    assert road(fsm, 6.0, Y_OVERTAKE, [], t0=0.05) == P.OVERTAKING
    assert road(fsm, 1.5, Y_OVERTAKE, [], t0=6.05) == P.LANE_CHANGE_RETURN


def test_extrapolation_not_carried_to_next_overtake():
    """`_passed` harus bersih setelah kembali ke lajur asal, kalau tidak salip
    kedua memulai dengan tebakan basi dari salip pertama."""
    fsm = P.BehaviorFSM()
    fsm.state = P.OVERTAKING
    fsm.update(0.0, Y_OVERTAKE, V_EGO, [front(4.2, V_EGO - 6.4)])
    fsm.state = P.LANE_CHANGE_RETURN
    road(fsm, 0.5, 0.0, [])
    assert fsm.state == P.LANE_KEEPING
    assert fsm._passed is None and fsm._v_target is None


def test_done_when_back_at_lane_center():
    fsm = P.BehaviorFSM()
    fsm.state = P.LANE_CHANGE_RETURN
    assert road(fsm, 1.0, -1.0, []) == P.LANE_CHANGE_RETURN
    assert road(fsm, 1.0, -0.1, []) == P.LANE_KEEPING


def test_y_goal_follows_state():
    fsm = P.BehaviorFSM()
    for state, goal in [(P.LANE_KEEPING, 0.0), (P.CHECK_OVERTAKE, 0.0),
                        (P.LANE_CHANGE_OVERTAKE, Y_OVERTAKE), (P.OVERTAKING, Y_OVERTAKE),
                        (P.LANE_CHANGE_RETURN, 0.0)]:
        fsm.state = state
        assert fsm.y_goal == goal, (state, fsm.y_goal)


def test_no_slowdown_when_lead_is_far():
    fsm = P.BehaviorFSM()
    fsm.update(0.0, 0.0, V_EGO, [front(60.0)])
    assert fsm.v_goal == config.V_REF


def test_no_slowdown_when_able_to_overtake():
    """Depan dekat & lambat, lajur tujuan kosong, waktu cukup: jangan melambat --
    selisih kecepatan itu yang dipakai untuk menyalip (S1 tidak boleh berubah)."""
    fsm = P.BehaviorFSM()
    fsm.update(0.0, 0.0, V_EGO, [front(25.0)])
    assert fsm.state == P.LANE_KEEPING and fsm.v_goal == config.V_REF


def test_follows_when_cannot_overtake_yet():
    """Lajur tujuan terisi: tertahan di CHECK, v_goal turun; tepat di jarak ikut
    = kecepatan depan; lebih dekat = di bawahnya (mundur ke jarak ikut)."""
    d_follow = config.ELLIPSE_A + config.AXLE_TO_CENTER + config.FOLLOW_TIME * V_SLOW
    fsm = P.BehaviorFSM()
    assert road(fsm, 2.0, 0.0, [front(20.0), in_overtake_lane(10.0)]) == P.CHECK_OVERTAKE
    assert fsm.v_goal < config.V_REF
    fsm.update(2.0, 0.0, V_EGO, [front(d_follow), in_overtake_lane(10.0)])
    assert abs(fsm.v_goal - V_SLOW) < 1e-9
    fsm.update(2.05, 0.0, V_EGO, [front(d_follow - 3.0), in_overtake_lane(10.0)])
    assert fsm.v_goal < V_SLOW


def test_overtakes_again_after_following():
    """Accelerative overtaking: ego sudah melambat (v_ego = v_front) sehingga TTC
    terhadap v_ego tak hingga. FSM lama tidak pernah terpicu lagi di sini."""
    d_follow = config.ELLIPSE_A + config.AXLE_TO_CENTER + config.FOLLOW_TIME * V_SLOW
    fsm = P.BehaviorFSM()
    assert road(fsm, 1.0, 0.0, [front(d_follow), in_overtake_lane(5.0)],
                 v_ego=V_SLOW) == P.CHECK_OVERTAKE
    assert road(fsm, 1.0, 0.0, [front(d_follow)], t0=1.0,
                 v_ego=V_SLOW) == P.LANE_CHANGE_OVERTAKE


def test_full_sequence_matches_section_6_table():
    """Skenario S1 sintetis: satu siklus penuh menyalip."""
    fsm, order, t, gap, d = P.BehaviorFSM(), [], 0.0, 40.0, 0.0
    for _ in range(400):
        obs = [front(gap)]
        s = fsm.update(t, d, V_EGO, obs)
        if not order or order[-1] != s:
            order.append(s)
        gap -= (V_EGO - V_SLOW) * 0.05                 # ego mendekat
        if fsm.state in (P.LANE_CHANGE_OVERTAKE, P.OVERTAKING):
            d = max(d - 1.2 * 0.05, Y_OVERTAKE) if config.SIDE_SIGN < 0 else \
                min(d + 1.2 * 0.05, Y_OVERTAKE)
        elif fsm.state == P.LANE_CHANGE_RETURN:
            d = min(d + 1.2 * 0.05, 0.0) if config.SIDE_SIGN < 0 else \
                max(d - 1.2 * 0.05, 0.0)
        t += 0.05
    assert order == [P.LANE_KEEPING, P.CHECK_OVERTAKE, P.LANE_CHANGE_OVERTAKE,
                    P.OVERTAKING, P.LANE_CHANGE_RETURN, P.LANE_KEEPING], order


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            fn()
            print(f'ok  {name}')
    print('all passed')
