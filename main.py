"""Entry point dan main loop (rencana kerja bagian 2.3).

Satu-satunya tempat yang mengatur frekuensi. Modul tidak tahu soal frekuensi --
mereka hanya dipanggil.

    python main.py                 # jalankan skenario S1, simpan log
    python main.py --skenario S3   # skenario lain dari config.SKENARIO
    python main.py --detik 25      # durasi lain
"""
import argparse
import json
import math
import os

import carla
import numpy as np

import config
import control
import evaluation
import localization
import perception
import planning
import simulation

# Urutan kolom log. Diindeks lewat NAMA, bukan angka -- menambah kolom di tengah
# sudah dua kali menggeser indeks dan memberi angka yang salah tanpa error.
KOLOM = ['t', 'x', 'y', 'yaw', 'v', 'y_ref', 'y_goal', 'dev_lajur', 'a_cmd',
         'delta_cmd', 'steer', 'throttle', 'brake', 'solve_ms', 'solver_ok',
         'n_layak', 'offset', 'x_tgt', 'y_tgt', 'v_goal']


def to_carla(cmd):
    return carla.VehicleControl(throttle=cmd.throttle, brake=cmd.brake, steer=cmd.steer)


def xref_tahan(ego, y_goal, v_des):
    """Acuan cadangan saat planner tidak menghasilkan rencana: tahan lajur."""
    t = np.arange(config.MPC_N + 1) * config.MPC_DT
    return np.vstack([ego.x + v_des * t, np.full_like(t, y_goal),
                      np.zeros_like(t), np.full_like(t, v_des)])


def xref_dari(traj, t_offset):
    t = t_offset + np.arange(config.MPC_N + 1) * config.MPC_DT
    return np.column_stack([traj.sample_at(tt) for tt in t])


def spawn_kendaraan(world, ref, ego_x, jarak, lajur):
    """Kendaraan lain `jarak` m dari posisi ego, di `lajur` (0 = lajur ego,
    1 = lajur menyalip). Bagian 11.1: posisi eksplisit, bukan acak."""
    s, x, y, psi, z = ref[0], ref[1], ref[2], ref[3], ref[4]
    s_t = ego_x + jarak
    p = float(np.interp(s_t, s, psi))
    d = lajur * config.SIDE_SIGN * config.LANE_WIDTH     # tegak lurus jalan, frame RH
    loc = carla.Location(x=float(np.interp(s_t, s, x)) - d * math.sin(p),
                         y=-(float(np.interp(s_t, s, y)) + d * math.cos(p)),
                         z=float(np.interp(s_t, s, z)) + 0.3)
    bp = world.get_blueprint_library().find('vehicle.nissan.patrol')
    return world.spawn_actor(bp, carla.Transform(loc, carla.Rotation(yaw=-math.degrees(p))))


def siapkan_jalan(world):
    """Reference path + koordinat s. Dipakai bersama main.py dan tuning.py."""
    sp = world.get_map().get_spawn_points()[config.SPAWN_IDX]
    ref = simulation.reference_path(world, sp.location, length_m=300.0, step=0.5)
    x, y = ref[0], ref[1]
    s = np.concatenate([[0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    return ref, (s, x, y, ref[2], ref[3])


def run(world, ego_actor, monitor, params, ref_rh, ref5, max_detik, kendaraan):
    """`kendaraan` = [(jarak, lajur, kecepatan), ...]; yang pertama = target."""
    frame = localization.PathFrame(ref_rh)
    loc = localization.CarlaGTLocalization(ego_actor, params['rear_axle_offset_x'])
    lihat = perception.GroundTruthPerception(world, ego_actor)
    fsm = planning.BehaviorFSM()
    mpc = control.MPCController(params)

    dt = config.FIXED_DELTA_SECONDS
    traj, t_traj, v_prev = None, 0.0, None
    a_filt, a_cmd_prev = 0.0, 0.0
    n_layak, offset_pilih = 0, 0.0
    log, states, posisi = [], [], []
    n_warm = int(config.WARMUP_DETIK / dt)
    aktor = []
    kirim = []                                # perintah aktor untuk tick berikutnya

    for k in range(-n_warm, int(max_detik / dt)):
        if k == 0:
            # Kendaraan lain baru di-spawn setelah transien reda, relatif posisi
            # ego yang SEBENARNYA -- bukan relatif titik spawn.
            ego_x = frame.ego(loc.update()).x
            aktor = [spawn_kendaraan(world, ref5, ego_x, jarak, lajur)
                     for jarak, lajur, _ in kendaraan]
        # Searah JALAN, bukan arah hadap aktor: hanya kecepatan yang dipaksa, jadi
        # gaya ban memutar arah hadap dan kendaraan bergeser lateral. Terukur di
        # S3: penghalang bergeser -3,50 -> -2,83 m, jarak bodi ke ego 0,87 m
        # padahal ego diam di lajurnya.
        kirim += [simulation.kecepatan(a, v, -math.degrees(frame.psi0))
                  for a, (_, _, v) in zip(aktor, kendaraan)]
        simulation.tick(world, kirim)
        t = k * dt

        ego = frame.ego(loc.update())                        # 20 Hz
        # perception melaporkan frame ego; konversi ke frame jalan di sini
        obs = localization.halangan_ego_ke_jalan(lihat.update(), ego)
        a_mentah = 0.0 if v_prev is None else (ego.v - v_prev) / dt
        v_prev = ego.v
        # potong lonjakan non-fisik lalu haluskan; mentahnya terlalu berderau
        # untuk dipakai langsung sebagai syarat batas maupun umpan balik PI
        a_mentah = float(np.clip(a_mentah, config.A_MIN, config.A_MAX))
        a_filt += config.A_FILTER_ALPHA * (a_mentah - a_filt)

        if k % 2 == 0:                                       # 10 Hz
            obs_rel = obs.copy()
            if len(obs_rel):
                obs_rel[:, 0] -= ego.x        # FSM memakai x relatif terhadap ego
            dy = ego.v * math.sin(ego.yaw)
            fsm.update(t, ego.y, ego.v, obs_rel, dy)
            # Percepatan awal lateral (y'') dan longitudinal (a0) dari RENCANA/
            # PERINTAH, bukan hasil ukur. Hasil ukur menutup lup planner-MPC: MPC
            # mengikuti kelengkungan awal rencana, percepatan itu terukur, lalu
            # jadi syarat awal rencana berikutnya. Di S3 satu tendangan kecil
            # tumbuh jadi simpangan 2,3 m keluar lajur (TUNING_MPC.md 13).
            ddy = 0.0 if traj is None else float(traj.lateral_at(t - t_traj)[2])
            traj, layak = planning.plan_lane_change(ego.y, dy, ddy, ego.x, ego.v,
                                                    a_cmd_prev, fsm.v_goal,
                                                    obstacles=obs, y_goal=fsm.y_goal)
            t_traj = t
            # bagian 11.5: jumlah kandidat lolos & offset terpilih = diagnostik
            # apakah sampling offset/T cocok. Kalau sering nol, samplingnya salah.
            n_layak = len(layak)
            offset_pilih = layak[0][1] if layak else 0.0

        xref = (xref_tahan(ego, fsm.y_goal, fsm.v_goal) if traj is None
                else xref_dari(traj, t - t_traj))
        cmd = mpc.compute(ego.as_vector(), xref, a_filt, obs)  # 20 Hz
        kirim = [carla.command.ApplyVehicleControl(ego_actor.id, to_carla(cmd))]
        a_cmd_prev = cmd.accel_cmd
        if not cmd.solver_ok:
            print(f'  solver gagal t={t:5.2f}s  state={fsm.state:<22} '
                  f'{mpc.last_status}  ({cmd.solve_time_ms:.0f} ms)')

        # y_ref (anchor planner) hampir sama dengan y ego per konstruksi, jadi
        # tidak bisa dipakai mengukur XTE. y_goal = tengah lajur yang dituju FSM;
        # metriknya ditetapkan saat menulis bab 4 (bagian 11.6: simpan mentah).
        if k >= 0:                     # fase pemanasan tidak dicatat
            # posisi kendaraan lain dari GROUND TRUTH, bukan perception: yang
            # dinilai penilai harus benar, yang dipakai mobil boleh berisik
            pos = []
            for a in aktor:
                tf = a.get_transform()
                # yaw ikut dicatat: yang dipaksa searah jalan hanya kecepatannya,
                # arah hadap bodi tetap bebas berputar (dipakai penilai jarak)
                pos.append((*frame.titik(tf.location.x, -tf.location.y),
                            localization.wrap(-math.radians(tf.rotation.yaw) - frame.psi0)))
            tx, ty = pos[0][:2]
            log.append([t, ego.x, ego.y, ego.yaw, ego.v, xref[1, 0], fsm.y_goal,
                        ego.y - fsm.y_goal, cmd.accel_cmd, cmd.delta_cmd, cmd.steer,
                        cmd.throttle, cmd.brake, cmd.solve_time_ms,
                        float(cmd.solver_ok), float(n_layak), offset_pilih, tx, ty,
                        fsm.v_goal])
            states.append(fsm.state)
            posisi.append(pos)

    return np.array(log), np.array(states), aktor, np.array(posisi)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--detik', type=float, default=20.0)
    ap.add_argument('--skenario', default='S1', choices=list(config.SKENARIO))
    args = ap.parse_args()
    kendaraan = config.SKENARIO[args.skenario]

    params = json.load(open(config.VEHICLE_PARAMS_JSON))
    with simulation.carla_world() as world:
        ref, ref5 = siapkan_jalan(world)

        with simulation.ego_vehicle(world) as ego:
            # bagian 11.1: ego mulai di v0
            simulation.tick(world, [simulation.kecepatan(ego, config.EGO_V0)])
            aktor = []
            with evaluation.pantau_tabrakan(world, ego) as monitor:
                try:
                    log, states, aktor, posisi = run(world, ego, monitor, params, ref,
                                                     ref5, args.detik, kendaraan)
                    dims = [(2 * a.bounding_box.extent.x, 2 * a.bounding_box.extent.y)
                            for a in aktor]
                finally:
                    for a in aktor:
                        a.destroy()

    path = os.path.join(config.OUT_DIR, f'run_{args.skenario.lower()}_mpc_gt.npz')
    np.savez(path, log=log, fsm_state=states, kolom=KOLOM, posisi_kendaraan=posisi,
             dim_kendaraan=np.array(dims), dim_ego=np.array([params['length'], params['width']]))
    k = {nama: i for i, nama in enumerate(KOLOM)}     # indeks lewat nama, bukan angka
    xte, ms = np.abs(log[:, k['dev_lajur']]), log[:, k['solve_ms']]
    tahan = states == 'LANE_KEEPING'
    print(f'\ndeviasi dari tengah lajur, saat LANE_KEEPING: '
          f'rata-rata {xte[tahan].mean():.3f} m, maks {xte[tahan].max():.3f} m')
    print(f'solve {ms[1:].mean():.1f} ms rata-rata, {ms[1:].max():.1f} ms maks '
          f'(tick pertama {ms[0]:.0f} ms), gagal '
          f'{int((log[:, k["solver_ok"]] == 0).sum())} dari {len(log)}')
    print(f'kecepatan {log[:, k["v"]].min() * 3.6:.0f}-{log[:, k["v"]].max() * 3.6:.0f} '
          f'km/jam, lateral {log[:, k["y"]].min():+.2f} .. {log[:, k["y"]].max():+.2f} m')
    print(f'v_goal minimum {log[:, k["v_goal"]].min() * 3.6:.1f} km/jam, '
          f'a_cmd minimum {log[:, k["a_cmd"]].min():+.2f} m/s²')
    urut = [states[0]] + [b for a, b in zip(states, states[1:]) if a != b]
    print(f'urutan state: {" -> ".join(urut)}')
    manuver = states != 'LANE_KEEPING'
    if manuver.any():
        n_lyk, off = log[manuver, k['n_layak']], log[manuver, k['offset']]
        print(f'kandidat lolos saat manuver: {n_lyk.min():.0f}-{n_lyk.max():.0f} dari 9'
              f'  (nol kandidat: {int((n_lyk == 0).sum())} tick)')
        print(f'offset terpilih: {sorted(set(np.round(off[off > 0], 1)))}')
    print(f'tabrakan: {monitor.ringkas()}')
    dim_ego = (params['length'], params['width'])
    lain = [(posisi[:, i, 0], posisi[:, i, 1], posisi[:, i, 2], dims[i])
            for i in range(1, len(aktor))]
    berhasil, kategori, rincian = evaluation.nilai_run(
        log[:, k['t']], log[:, k['x']], log[:, k['y']], states,
        log[:, k['x_tgt']], log[:, k['y_tgt']], monitor.tabrakan, dim_ego, dims[0], lain,
        yaw=log[:, k['yaw']], yaw_tgt=posisi[:, 0, 2])
    print()
    print(evaluation.ringkas_penilaian(berhasil, kategori, rincian))
    print(f'\nlog: {path}')


if __name__ == '__main__':
    main()
