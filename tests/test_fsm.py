"""Uji behavior FSM terhadap tabel transisi bagian 6. Tidak butuh CARLA."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config                                                         # noqa: E402
import planning as P                                                  # noqa: E402

V_EGO, V_LAMBAT = 13.9, 7.0
Y_SALIP = config.SIDE_SIGN * config.LANE_WIDTH


def depan(x, v=V_LAMBAT):
    return [x, 0.0, v, 0.0]


def di_lajur_salip(x, v=V_EGO):
    return [x, Y_SALIP, v, 0.0]


def jalan(fsm, durasi, d, obs, t0=0.0, dt=0.05, v_ego=V_EGO):
    """Jalankan FSM selama `durasi` detik dengan input tetap."""
    t = t0
    for _ in range(int(round(durasi / dt))):
        fsm.update(t, d, v_ego, obs)
        t += dt
    return fsm.state


def test_diam_di_lane_keeping_tanpa_halangan():
    fsm = P.BehaviorFSM()
    assert jalan(fsm, 3.0, 0.0, []) == P.LANE_KEEPING


def test_tidak_terpicu_bila_depan_tidak_cukup_lambat():
    fsm = P.BehaviorFSM()
    assert jalan(fsm, 3.0, 0.0, [depan(15.0, V_EGO - 1.0)]) == P.LANE_KEEPING


def test_tidak_terpicu_bila_masih_jauh():
    fsm = P.BehaviorFSM()
    assert jalan(fsm, 3.0, 0.0, [depan(60.0)]) == P.LANE_KEEPING


def test_dwell_time_menunda_transisi():
    """0,25 s belum cukup; 0,35 s sudah (ambang FSM_DWELL = 0,3 s)."""
    obs = [depan(24.0)]
    assert jalan(P.BehaviorFSM(), 0.25, 0.0, obs) == P.LANE_KEEPING
    assert jalan(P.BehaviorFSM(), 0.35, 0.0, obs) == P.CHECK_OVERTAKE


def test_check_overtake_lanjut_bila_lajur_tujuan_kosong():
    fsm = P.BehaviorFSM()
    assert jalan(fsm, 1.0, 0.0, [depan(24.0)]) == P.LANE_CHANGE_OVERTAKE


def test_check_overtake_tertahan_bila_lajur_tujuan_terisi():
    fsm = P.BehaviorFSM()
    obs = [depan(24.0), di_lajur_salip(10.0)]
    assert jalan(fsm, 2.0, 0.0, obs) == P.CHECK_OVERTAKE


def test_check_overtake_tertahan_bila_ada_kendaraan_dari_belakang():
    fsm = P.BehaviorFSM()
    obs = [depan(24.0), di_lajur_salip(-8.0)]
    assert jalan(fsm, 2.0, 0.0, obs) == P.CHECK_OVERTAKE


def test_check_overtake_tertahan_bila_waktu_tidak_cukup():
    """TTC 10/6.9 = 1,45 s, lebih pendek dari manuver tercepat 3,0 s."""
    fsm = P.BehaviorFSM()
    assert jalan(fsm, 2.0, 0.0, [depan(10.0)]) == P.CHECK_OVERTAKE


def test_histeresis_kembali_ke_lane_keeping():
    """Kendaraan depan mempercepat -> batal, memakai ambang keluar bukan masuk."""
    fsm = P.BehaviorFSM()
    fsm.state = P.CHECK_OVERTAKE
    assert jalan(fsm, 1.0, 0.0, [depan(24.0, V_EGO - 1.0)]) == P.LANE_KEEPING


def test_tidak_bolak_balik_di_sekitar_ambang():
    """Tepat di ambang masuk: sekali masuk CHECK, tidak keluar lagi."""
    dv = V_EGO - V_LAMBAT
    gap_ambang = config.TTC_TRIGGER * dv
    fsm = P.BehaviorFSM()
    jalan(fsm, 1.0, 0.0, [depan(gap_ambang - 1.0)])
    urut = []
    for k in range(60):                       # celah bergoyang di sekitar ambang
        gap = gap_ambang + (2.0 if k % 2 else -2.0)
        fsm.update(1.0 + k * 0.05, 0.0, V_EGO, [depan(gap)])
        urut.append(fsm.state)
    assert P.LANE_KEEPING not in urut, set(urut)


def test_ambang_menyesuaikan_selisih_kecepatan():
    """Inti pemicu TTC: celah pemicu ikut selisih kecepatan, bukan tetap."""
    hasil = {}
    for v_lambat in (11.0, 7.0, 4.0):                  # dv = 2.9, 6.9, 9.9
        dv = V_EGO - v_lambat
        for gap in range(70, 4, -1):          # dari jauh ke dekat: cari celah terbesar
            fsm = P.BehaviorFSM()             # yang masih memicu = ambangnya
            if jalan(fsm, 0.5, 0.0, [depan(float(gap), v_lambat)]) != P.LANE_KEEPING:
                hasil[round(dv, 1)] = gap
                break
    assert 2.9 not in hasil, 'dv < DV_TRIGGER seharusnya tidak pernah memicu'
    assert hasil[6.9] < hasil[9.9], hasil     # makin cepat mendekat, makin jauh dipicu
    for dv, gap in hasil.items():
        assert abs(gap / dv - config.TTC_TRIGGER) < 0.6, (dv, gap, gap / dv)


def test_masuk_overtaking_pada_90_persen_lebar_lajur():
    fsm = P.BehaviorFSM()
    fsm.state = P.LANE_CHANGE_OVERTAKE
    obs = [depan(12.0)]
    d_kurang = 0.8 * config.LANE_WIDTH * config.SIDE_SIGN
    assert jalan(fsm, 1.0, d_kurang, obs) == P.LANE_CHANGE_OVERTAKE
    d_cukup = 0.95 * config.LANE_WIDTH * config.SIDE_SIGN
    assert jalan(fsm, 1.0, d_cukup, obs) == P.OVERTAKING


def test_abort_langsung_tanpa_dwell():
    """Kendaraan muncul di lajur tujuan saat pindah lajur -> batal seketika."""
    fsm = P.BehaviorFSM()
    fsm.state = P.LANE_CHANGE_OVERTAKE
    fsm.update(0.0, -1.0 * -config.SIDE_SIGN, V_EGO,
               [depan(12.0), di_lajur_salip(5.0)])
    assert fsm.state == P.LANE_KEEPING          # satu tick saja, bukan 0,3 detik
    assert fsm.abort_terakhir == 0.0


def test_kembali_setelah_unggul_pass_margin():
    fsm = P.BehaviorFSM()
    fsm.state = P.OVERTAKING
    d = Y_SALIP
    belum = -(config.PASS_MARGIN - 2.0)
    assert jalan(fsm, 1.0, d, [depan(belum)]) == P.OVERTAKING
    sudah = -(config.PASS_MARGIN + 2.0)
    assert jalan(fsm, 1.0, d, [depan(sudah)]) == P.LANE_CHANGE_RETURN


def test_selesai_saat_kembali_ke_tengah_lajur():
    fsm = P.BehaviorFSM()
    fsm.state = P.LANE_CHANGE_RETURN
    assert jalan(fsm, 1.0, -1.0, []) == P.LANE_CHANGE_RETURN
    assert jalan(fsm, 1.0, -0.1, []) == P.LANE_KEEPING


def test_y_goal_mengikuti_state():
    fsm = P.BehaviorFSM()
    for state, goal in [(P.LANE_KEEPING, 0.0), (P.CHECK_OVERTAKE, 0.0),
                        (P.LANE_CHANGE_OVERTAKE, Y_SALIP), (P.OVERTAKING, Y_SALIP),
                        (P.LANE_CHANGE_RETURN, 0.0)]:
        fsm.state = state
        assert fsm.y_goal == goal, (state, fsm.y_goal)


def test_urutan_lengkap_sesuai_tabel_bagian_6():
    """Skenario S1 sintetis: satu siklus penuh menyalip."""
    fsm, urut, t, gap, d = P.BehaviorFSM(), [], 0.0, 40.0, 0.0
    for _ in range(400):
        obs = [depan(gap)]
        s = fsm.update(t, d, V_EGO, obs)
        if not urut or urut[-1] != s:
            urut.append(s)
        gap -= (V_EGO - V_LAMBAT) * 0.05                 # ego mendekat
        if fsm.state in (P.LANE_CHANGE_OVERTAKE, P.OVERTAKING):
            d = max(d - 1.2 * 0.05, Y_SALIP) if config.SIDE_SIGN < 0 else \
                min(d + 1.2 * 0.05, Y_SALIP)
        elif fsm.state == P.LANE_CHANGE_RETURN:
            d = min(d + 1.2 * 0.05, 0.0) if config.SIDE_SIGN < 0 else \
                max(d - 1.2 * 0.05, 0.0)
        t += 0.05
    assert urut == [P.LANE_KEEPING, P.CHECK_OVERTAKE, P.LANE_CHANGE_OVERTAKE,
                    P.OVERTAKING, P.LANE_CHANGE_RETURN, P.LANE_KEEPING], urut


if __name__ == '__main__':
    for nama, fn in sorted(globals().items()):
        if nama.startswith('test_'):
            fn()
            print(f'ok  {nama}')
    print('semua lolos')
