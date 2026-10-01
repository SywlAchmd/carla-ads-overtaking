"""Entry point dan main loop (rencana kerja bagian 2.3).

Satu-satunya tempat yang mengatur frekuensi. Modul tidak tahu soal frekuensi --
mereka hanya dipanggil.

    python main.py                 # jalankan skenario S1, simpan log
    python main.py --scenario S3   # skenario lain dari config.SCENARIOS
    python main.py --seconds 25      # durasi lain
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
import overlay
import localization
import perception
import planning
import sensors
import simulation

# Urutan kolom log. Diindeks lewat NAMA, bukan angka -- menambah kolom di tengah
# sudah dua kali menggeser indeks dan memberi angka yang salah tanpa error.
COLUMNS = ['t', 'x', 'y', 'yaw', 'v', 'y_ref', 'y_goal', 'lane_dev', 'a_cmd',
         'delta_cmd', 'steer', 'throttle', 'brake', 'solve_ms', 'solver_ok',
         'n_feasible', 'offset', 'x_tgt', 'y_tgt', 'v_goal', 'x_est', 'y_est',
         'iterations', 't_plan', 'eps', 'eps_lat', 'y_plan_05', 'y_plan_20',
         # Posisi ego di frame PETA, hanya untuk PENILAIAN. Sejak jangkar peta
         # dibuang (bagian 28.3), `x`/`y` di atas ada di frame yang dijangkarkan
         # KAMERA -- itu benar untuk kendali, tetapi menilai "kembali ke lajur"
         # dengannya berarti bertanya apakah ego kembali ke lajur yang DIYAKININYA
         # sendiri. Alat ukur harus terpisah dari yang diukur.
         'x_map', 'y_map']


def to_carla(cmd):
    return carla.VehicleControl(throttle=cmd.throttle, brake=cmd.brake, steer=cmd.steer)


def xref_hold(ego, v_des, y=None):
    """Acuan garis lurus pada `y` (bawaan: posisi lateral ego SEKARANG).

    Sebagai acuan cadangan saat planner gagal, `y` dibiarkan bawaan -- tahan
    posisi yang sedang berlaku.

    Sempat memakai `y_goal` dan itu keliru: kehilangan kandidat di tengah manuver
    lalu berarti "lompat ke lajur tujuan sekarang". Di run vision S1 acuannya
    melompat 3,23 m sekaligus, MPC mengejarnya dengan delta -0,159 rad dan rem
    -4,43 m/s^2, dan laju lateral yang terkumpul membawa ego 1,96 m melewati
    tengah lajur tujuan. Menahan `ego.y` membuat kehilangan kandidat berarti
    "lanjutkan lurus" -- planner replan 10 Hz akan mengambil alih begitu ada
    kandidat lagi, mulai dari state saat itu.
    """
    y = ego.y if y is None else y
    t = np.arange(config.MPC_N + 1) * config.MPC_DT
    return np.vstack([ego.x + v_des * t, np.full_like(t, y),
                      np.zeros_like(t), np.full_like(t, v_des)])


def xref_from(traj, t_offset):
    t = t_offset + np.arange(config.MPC_N + 1) * config.MPC_DT
    return np.column_stack([traj.sample_at(tt) for tt in t])


def spawn_vehicles(world, ref, ego_x, dist, lane):
    """Kendaraan lain `dist` m dari posisi ego, di `lane` (0 = lajur ego,
    1 = lajur menyalip). Bagian 11.1: posisi eksplisit, bukan acak."""
    s, x, y, psi, z = ref[0], ref[1], ref[2], ref[3], ref[4]
    s_t = ego_x + dist
    p = float(np.interp(s_t, s, psi))
    d = lane * config.SIDE_SIGN * config.LANE_WIDTH     # tegak lurus jalan, frame RH
    loc = carla.Location(x=float(np.interp(s_t, s, x)) - d * math.sin(p),
                         y=-(float(np.interp(s_t, s, y)) + d * math.cos(p)),
                         z=float(np.interp(s_t, s, z)) + 0.3)
    bp = world.get_blueprint_library().find(config.OTHER_BP)
    return world.spawn_actor(bp, carla.Transform(loc, carla.Rotation(yaw=-math.degrees(p))))


def prepare_road(world):
    """Reference path + koordinat s. Dipakai bersama main.py dan tuning.py."""
    sp = world.get_map().get_spawn_points()[config.SPAWN_IDX]
    ref = simulation.reference_path(world, sp.location, length_m=300.0, step=0.5)
    x, y = ref[0], ref[1]
    s = np.concatenate([[0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    return ref, (s, x, y, ref[2], ref[3])


def run(world, ego_actor, monitor, params, ref_rh, ref5, max_seconds, vehicles, rig=None,
        net=None, recorder=None):
    """`vehicles` = [(jarak, lajur, kecepatan), ...]; yang pertama = target.

    `rig` terisi -> perception berbasis vision; None -> ground truth.
    """
    # Frame PETA. Dipakai untuk menempatkan kendaraan skenario, menggambar overlay,
    # dan sebagai pembanding -- bukan untuk kendali di jalur vision (bagian 28.4).
    map_frame = localization.PathFrame(ref_rh)
    frame = map_frame
    loc = localization.CarlaGTLocalization(ego_actor, params['rear_axle_offset_x'])
    vision = (perception.VisionPerception(net, rig) if rig
             else perception.GroundTruthPerception(world, ego_actor))
    fsm = planning.BehaviorFSM()
    mpc = control.MPCController(params)

    dt = config.FIXED_DELTA_SECONDS
    traj, t_traj, v_prev = None, 0.0, None
    psi_road = None              # arah jalan hasil ukur, ditapis (None = jalur GT)
    a_filt, a_cmd_prev = 0.0, 0.0
    n_feasible, offset_chosen, feasible_end = 0, 0.0, []
    t_plan = float('nan')
    log, states, positions = [], [], []
    n_warm = int(config.WARMUP_SECONDS / dt)
    actors = []
    send = []                                # perintah aktor untuk tick berikutnya

    for k in range(-n_warm, int(max_seconds / dt)):
        if k == 0:
            # Kendaraan lain baru di-spawn setelah transien reda, relatif posisi
            # ego yang SEBENARNYA -- bukan relatif titik spawn.
            ego_x = map_frame.ego(loc.update()).x
            actors = [spawn_vehicles(world, ref5, ego_x, dist, lane)
                     for dist, lane, _ in vehicles]
        # Searah JALAN, bukan arah hadap aktor: hanya kecepatan yang dipaksa, jadi
        # gaya ban memutar arah hadap dan kendaraan bergeser lateral. Terukur di
        # S3: penghalang bergeser -3,50 -> -2,83 m, jarak bodi ke ego 0,87 m
        # padahal ego diam di lajurnya.
        send += [simulation.velocity(a, v, -math.degrees(map_frame.psi0))
                  for a, (_, _, v) in zip(actors, vehicles)]
        simulation.tick(world, send)
        t = k * dt

        if k == -1 and rig is not None and getattr(vision, 'lane', None) is not None:
            # JANGKAR PETA DIBUANG (bagian 28.4). Arah jalan dan letak sumbu lajur
            # diambil dari kepala segmentasi, bukan dari `world.get_map()`. s = 0
            # tetap disamakan dengan frame peta -- itu konvensi, bukan geometri --
            # supaya log kedua jalur bisa dibandingkan angka per angka.
            g0, e0 = vision.lane, loc.update()
            if g0.lane_width is not None:
                frame = localization.PathFrame.from_perception(
                    e0, g0, x0=map_frame.ego(e0).x)
                p_map, p_seen = map_frame.ego(e0), frame.ego(e0)
                if recorder is not None:
                    recorder.pf = frame     # lintasan planner kini di frame kendali
                psi_road = frame.psi0
                print(f'anchor from perception: road heading '
                      f'{math.degrees(localization.wrap(frame.psi0 - map_frame.psi0)):+.3f} deg '
                      f'vs map, lane axis {p_seen.y - p_map.y:+.3f} m')

        st_rh = loc.update()
        ego = frame.ego(st_rh)                               # 20 Hz
        ego_map = map_frame.ego(st_rh)                     # hanya untuk penilaian
        # JEJAK arah jalan, jangan dibekukan. Titik asal digeser bersamaan supaya
        # (x, y) ego tidak melompat: yang dikoreksi hanya arah ke depan.
        geo_f = getattr(vision, 'lane', None)
        if psi_road is not None and geo_f is not None and geo_f.lane_width is not None:
            psi_road = localization.wrap(
                psi_road + config.ALPHA_ROAD_HEADING
                * localization.wrap(st_rh.yaw - geo_f.yaw - psi_road))
            frame = localization.PathFrame.from_pose(st_rh, psi_road, ego.x, ego.y)
            if recorder is not None:
                recorder.pf = frame
            ego = frame.ego(st_rh)
        a_raw = 0.0 if v_prev is None else (ego.v - v_prev) / dt
        v_prev = ego.v
        # potong lonjakan non-fisik lalu haluskan; mentahnya terlalu berderau
        # untuk dipakai langsung sebagai syarat batas maupun umpan balik PI
        a_raw = float(np.clip(a_raw, config.A_MIN, config.A_MAX))
        a_filt += config.A_FILTER_ALPHA * (a_raw - a_filt)
        # perception melaporkan frame ego; konversi ke frame jalan di sini.
        # Vision perlu kinematika ego (laju, percepatan, yaw rate) untuk
        # mengompensasi gerak ego di langkah prediksi Kalman -- bukan posisi ego.
        # Antrean kamera WAJIB dikuras tiap tick, termasuk saat pemanasan.
        image = rig.grab() if rig else None
        obs = localization.obstacles_ego_to_road(
            vision.update(image, dt, ego.v, a_filt, ego.yaw_rate) if rig
            else vision.update(), ego)

        # Geometri lajur & dimensi kendaraan dari perception, bukan dari peta HD
        # maupun bounding box simulator (bagian 28). None selama jalur GT atau
        # selama masker lajur belum terbaca -- pemakainya jatuh ke konstanta peta.
        lane = zone = lane_width = None
        geo = getattr(vision, 'lane', None)
        if geo is not None and geo.lane_width is not None:
            # lane_dev positif = ego di KIRI tengah lajur, sama seperti frame jalan.
            # Mentah di sini; penapisannya di `BehaviorFSM`, di tempat yang sama
            # dengan latch-nya. Sempat ditapis di sini dan itu KELIRU: tapisnya
            # ikut berjalan selama manuver, ketika `lane_dev` mengacu ke lajur
            # SALIP, sehingga saat kembali ia membawa nilai yang sudah tertarik ke
            # lajur seberang -- lompatan `y_goal` justru naik 0,147 -> 0,693 m.
            lane = (ego.y - geo.lane_dev, geo.lane_width)
            lane_width = geo.lane_width
        if getattr(vision, 'dimensions', None) is not None:
            # Zona aman memakai KENDARAAN DESAIN, bukan taksiran per-frame.
            # Taksiran dimensi dipakai untuk KETELITIAN (`face_correction`), zona aman
            # untuk KESELAMATAN, dan keduanya menuntut hal yang berbeda: bias
            # perception sebesar 0,26 m pada lebar sudah cukup menggeser zona dan
            # menggagalkan run. Margin keselamatan tidak boleh bisa menyusut oleh
            # galat penaksir. Kendaraan desain PDGJ 2021 lebih besar daripada target
            # mana pun di skenario, jadi zonanya konservatif dengan sendirinya.
            zone = config.zone_from_dimensions(config.PRIOR_LENGTH, config.PRIOR_WIDTH,
                                            lane_width)

        if k % 2 == 0:                                       # 10 Hz
            obs_rel = obs.copy()
            if len(obs_rel):
                obs_rel[:, 0] -= ego.x        # FSM memakai x relatif terhadap ego
            # Laju lateral awal planner dari RENCANA, sama alasannya dengan ddy
            # di bawah. Hasil ukur dipakai hanya saat belum ada rencana sama
            # sekali. Komentar lama di sini mengatakan lup dy0 masih terbuka --
            # itu sudah TIDAK benar sejak baris di bawah mengambil dari `traj`,
            # dan diverifikasi 28 Sep 2026: laju lateral rencana dan hasil ukur
            # sepakat sampai 0,012 m/s saat LANE_KEEPING.
            dy_meas = ego.v * math.sin(ego.yaw)
            dy = dy_meas if traj is None else float(traj.lateral_at(t - t_traj)[1])
            fsm.update(t, ego.y, ego.v, obs_rel, dy_meas, lane=lane)
            # Percepatan awal lateral (y'') dan longitudinal (a0) dari RENCANA/
            # PERINTAH, bukan hasil ukur. Hasil ukur menutup lup planner-MPC: MPC
            # mengikuti kelengkungan awal rencana, percepatan itu terukur, lalu
            # jadi syarat awal rencana berikutnya. Di S3 satu tendangan kecil
            # tumbuh jadi simpangan 2,3 m keluar lajur (TUNING_MPC.md 13).
            ddy = 0.0 if traj is None else float(traj.lateral_at(t - t_traj)[2])
            traj_new, feasible = planning.plan_lane_change(
                ego.y, dy, ddy, ego.x, ego.v, a_cmd_prev, fsm.v_goal,
                obstacles=obs, y_goal=fsm.y_goal, zone=zone, lane_width=lane_width)
            # KOMITMEN: replan yang gagal tidak membuang rencana yang sedang
            # berjalan (bagian 19.14). Menyeberang itu balapan antara kemajuan
            # lateral dan celah yang menutup, dan celah minimum yang dibutuhkan
            # JUSTRU MENGECIL saat ego makin menyeberang (19.13) -- jadi berhenti
            # di tengah adalah hal terburuk yang bisa dilakukan. Tanpa ini rencana
            # berkedip 10 kali per run, lima kepingannya lebih pendek dari 0,3 s.
            if traj_new is not None:
                traj, t_traj = traj_new, t
            elif traj is not None and t - t_traj > traj.duration():
                traj = None                      # kedaluwarsa, jangan dipegang selamanya
            # bagian 11.5: jumlah kandidat lolos & offset terpilih = diagnostik
            # apakah sampling offset/T cocok. Kalau sering nol, samplingnya salah.
            n_feasible = len(feasible)
            offset_chosen = feasible[0][1] if feasible else 0.0
            feasible_end = feasible
            t_plan = feasible[0][2] if feasible else float('nan')

        xref = (xref_hold(ego, fsm.v_goal) if traj is None
                else xref_from(traj, t - t_traj))
        # Acuan pada LOOKAHEAD TETAP, untuk galat pelacakan yang sah. |y - y_ref|
        # tidak bisa dipakai: planner me-anchor rencananya di posisi ego tiap
        # replan, jadi galatnya nol karena konstruksi (bagian 22.5). Yang ini
        # dibandingkan dengan posisi SEBENARNYA 0,5 dan 2,0 detik kemudian.
        y_plan_05 = float(xref[1, int(0.5 / config.MPC_DT)])
        y_plan_20 = float(xref[1, config.MPC_N])
        cmd = mpc.compute(ego.as_vector(), xref, a_filt, obs, zone=zone)  # 20 Hz
        send = [carla.command.ApplyVehicleControl(ego_actor.id, to_carla(cmd))]
        a_cmd_prev = cmd.accel_cmd
        if recorder is not None and k >= 0:
            recorder.extra(
                sensors.rgb_array(image['rgb']),
                rig.sensor['rgb'].get_transform().get_inverse_matrix(),
                vision.tracker.visible(), feasible_end, traj,
                [f't = {t:5.2f} s', f'{fsm.state}', f'{ego.v * 3.6:.1f} km/h',
                 f'candidates passed {n_feasible}/9, offset {offset_chosen:.1f} m',
                 f'solve {cmd.solve_time_ms:.0f} ms'],
                ego.v, mask=getattr(vision, 'mask', None),
                lane=getattr(vision, 'lane', None))
        if not cmd.solver_ok:
            print(f'  solver failed t={t:5.2f}s  state={fsm.state:<22} '
                  f'{mpc.last_status}  ({cmd.solve_time_ms:.0f} ms)')

        # y_ref (anchor planner) hampir sama dengan y ego per konstruksi, jadi
        # tidak bisa dipakai mengukur XTE. y_goal = tengah lajur yang dituju FSM;
        # metriknya ditetapkan saat menulis bab 4 (bagian 11.6: simpan mentah).
        if k >= 0:                     # fase pemanasan tidak dicatat
            # posisi kendaraan lain dari GROUND TRUTH, bukan perception: yang
            # dinilai penilai harus benar, yang dipakai mobil boleh berisik
            pos = []
            for a in actors:
                tf = a.get_transform()
                # yaw ikut dicatat: yang dipaksa searah jalan hanya kecepatannya,
                # arah hadap bodi tetap bebas berputar (dipakai penilai jarak)
                pos.append((*frame.points(tf.location.x, -tf.location.y),
                            localization.wrap(-math.radians(tf.rotation.yaw) - frame.psi0)))
            tx, ty = pos[0][:2]
            # Estimasi perception untuk target, DI SAMPING ground truth-nya: tanpa
            # ini tidak ada cara membedakan "kandidat habis karena estimasi meleset"
            # dari "kandidat habis karena geometrinya memang mepet".
            if len(obs):
                j = int(np.argmin(np.hypot(obs[:, 0] - tx, obs[:, 1] - ty)))
                x_est, y_est = float(obs[j, 0]), float(obs[j, 1])
            else:
                x_est = y_est = float('nan')
            log.append([t, ego.x, ego.y, ego.yaw, ego.v, xref[1, 0], fsm.y_goal,
                        ego.y - fsm.y_goal, cmd.accel_cmd, cmd.delta_cmd, cmd.steer,
                        cmd.throttle, cmd.brake, cmd.solve_time_ms,
                        float(cmd.solver_ok), float(n_feasible), offset_chosen, tx, ty,
                        fsm.v_goal, x_est, y_est,
                        float(mpc.last_iter), t_plan, mpc.last_eps, mpc.last_eps_lat,
                        y_plan_05, y_plan_20, ego_map.x, ego_map.y])
            states.append(fsm.state)
            positions.append(pos)

    # Dimensi & geometri lajur yang DITAKSIR perception, disandingkan dengan
    # bounding box simulator dan peta -- keduanya alat ukur di sini, bukan masukan.
    dim = getattr(vision, 'dimensions', None)
    if dim is not None and dim.n_observations:
        pj, lb = dim.size()
        tg = dim.height
        print(f'\nother vehicle dimensions, ESTIMATED from detection boxes '
              f'({dim.n_observations} observations, observed {dim.observed:.3f}):')
        print(f'  length  {pj:6.3f} m   true {config.OTHER_LENGTH:.3f}   '
              f'error {pj - config.OTHER_LENGTH:+.3f}')
        print(f'  width   {lb:6.3f} m   true {config.OTHER_WIDTH:.3f}   '
              f'error {lb - config.OTHER_WIDTH:+.3f}')
        if tg is not None:
            print(f'  height  {tg:6.3f} m   true {config.OTHER_HEIGHT:.3f}   '
                  f'error {tg - config.OTHER_HEIGHT:+.3f}')
    geo = getattr(vision, 'lane', None)
    if geo is not None and geo.lane_width is not None:
        print(f'lane width estimated {geo.lane_width:.3f} m   '
              f'map {config.LANE_WIDTH:.3f}   error {geo.lane_width - config.LANE_WIDTH:+.3f}')

    return np.array(log), np.array(states), actors, np.array(positions)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=20.0)
    ap.add_argument('--scenario', default='S1', choices=list(config.SCENARIOS))
    ap.add_argument('--perception', default='gt', choices=('gt', 'vision'))
    ap.add_argument('--record', action='store_true',
                    help='camera video + detections + planner candidates (needs --perception vision)')
    ap.add_argument('--weight', default=None,
                    help='alternate YOLOPX checkpoint, e.g. to compare per-marking versus '
                         'continuous lane annotation (section 30)')
    ap.add_argument('--suffix', default='',
                    help='output file name suffix, e.g. _after -- so recordings '
                         'and comparison logs do not overwrite each other')
    args = ap.parse_args()
    vehicles = config.SCENARIOS[args.scenario]

    params = json.load(open(config.VEHICLE_PARAMS_JSON))
    with simulation.carla_world() as world:
        ref, ref5 = prepare_road(world)

        with simulation.ego_vehicle(world) as ego:
            # bagian 11.1: ego mulai di v0
            simulation.tick(world, [simulation.velocity(ego, config.EGO_V0)])
            actors = []
            # Rig kamera hanya dipasang di mode vision: mode gt harus tetap
            # identik dengan run sebelumnya, tanpa beban render tambahan.
            rig = net = recorder = None
            if args.perception == 'vision':
                import yolopx
                net = yolopx.YOLOPX(args.weight)
                rig = sensors.CameraRig(world, ego, params)
                print(f'vision: YOLOPX epoch {net.epoch}, {net.device}')
                if args.record:
                    recorder = overlay.Recorder(localization.PathFrame(ref), ref5)
            elif args.record:
                ap.error('--record needs --perception vision')
            with evaluation.watch_collisions(world, ego) as monitor:
                try:
                    log, states, actors, positions = run(world, ego, monitor, params, ref,
                                                     ref5, args.seconds, vehicles, rig,
                                                     net, recorder)
                    dims = [(2 * a.bounding_box.extent.x, 2 * a.bounding_box.extent.y)
                            for a in actors]
                finally:
                    for a in actors:
                        a.destroy()
                    if rig:
                        rig.destroy()

    path = os.path.join(
        config.OUT_DIR,
        f'run_{args.scenario.lower()}_mpc_{args.perception}{args.suffix}.npz')
    np.savez(path, log=log, fsm_state=states, columns=COLUMNS, vehicle_positions=positions,
             vehicle_dims=np.array(dims), dim_ego=np.array([params['length'], params['width']]))
    k = {name: i for i, name in enumerate(COLUMNS)}     # indeks lewat nama, bukan angka
    xte, ms = np.abs(log[:, k['lane_dev']]), log[:, k['solve_ms']]
    hold = states == 'LANE_KEEPING'
    print(f'\ndeviation from lane center, during LANE_KEEPING: '
          f'mean {xte[hold].mean():.3f} m, max {xte[hold].max():.3f} m')
    print(f'solve {ms[1:].mean():.1f} ms mean, {ms[1:].max():.1f} ms max '
          f'(first tick {ms[0]:.0f} ms), failed '
          f'{int((log[:, k["solver_ok"]] == 0).sum())} of {len(log)}')
    print(f'speed {log[:, k["v"]].min() * 3.6:.0f}-{log[:, k["v"]].max() * 3.6:.0f} '
          f'km/h, lateral {log[:, k["y"]].min():+.2f} .. {log[:, k["y"]].max():+.2f} m')
    print(f'v_goal minimum {log[:, k["v_goal"]].min() * 3.6:.1f} km/h, '
          f'a_cmd minimum {log[:, k["a_cmd"]].min():+.2f} m/s²')
    order = [states[0]] + [b for a, b in zip(states, states[1:]) if a != b]
    print(f'state sequence: {" -> ".join(order)}')
    maneuver = states != 'LANE_KEEPING'
    if maneuver.any():
        n_feas, off = log[maneuver, k['n_feasible']], log[maneuver, k['offset']]
        print(f'candidates passed during maneuver: {n_feas.min():.0f}-{n_feas.max():.0f} of 9'
              f'  (zero candidates: {int((n_feas == 0).sum())} tick)')
        print(f'chosen offsets: {sorted(set(np.round(off[off > 0], 1)))}')
    print(f'collisions: {monitor.summary()}')
    dim_ego = (params['length'], params['width'])
    other = [(positions[:, i, 0], positions[:, i, 1], positions[:, i, 2], dims[i])
            for i in range(1, len(actors))]
    success, category, details = evaluation.evaluate_run(
        log[:, k['t']], log[:, k['x']], log[:, k['y']], states,
        log[:, k['x_tgt']], log[:, k['y_tgt']], monitor.collisions, dim_ego, dims[0], other,
        yaw=log[:, k['yaw']], yaw_tgt=positions[:, 0, 2],
        # Jarak antar bodi tidak bergantung frame -- ia selisih dua titik. Syarat
        # LAJUR bergantung, jadi ia dan hanya ia dinilai di frame peta.
        y_lane=log[:, k['y_map']])
    print()
    print(evaluation.summarize_verdict(success, category, details))
    if recorder is not None:
        recorder.save(f'vision_{args.scenario.lower()}{args.suffix}.mp4')
    print(f'\nlog: {path}')


if __name__ == '__main__':
    main()
