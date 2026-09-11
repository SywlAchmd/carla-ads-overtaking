"""Playback manuver menyalip dari keluaran local planner (Tahap 3).

BUKAN hasil kendali. Ego digerakkan tepat mengikuti lintasan yang dihasilkan
planner (physics dimatikan), kendaraan target bergerak konstan. Ini visualisasi
bentuk lintasan; controller baru ada di Tahap 5.

    python record_maneuver.py   ->  out/overtake_planner.mp4
"""
import argparse
import math

import carla
import numpy as np
from PIL import Image, ImageDraw

import config
import localization
import planning
import record_path
import simulation

V_EGO, V_TARGET = 13.9, 7.0        # m/s -- 50 dan 25 km/jam (skenario S1)
GAP_AWAL = 50.0                    # m, jarak awal ego ke kendaraan target
DURASI = 16.0                      # detik


def jalur_acuan(world, location, panjang=260.0):
    """Centerline lajur ego: s, x, y, psi (right-handed) dan z (CARLA)."""
    wp = world.get_map().get_waypoint(location, project_to_road=True,
                                      lane_type=carla.LaneType.Driving)
    pts = list(simulation.walk_lane(wp, panjang, 0.5))
    rh = np.array([localization.carla_xy_yaw_to_rh(p.transform.location,
                                                   p.transform.rotation.yaw) for p in pts])
    x, y, psi = rh[:, 0], rh[:, 1], np.unwrap(rh[:, 2])
    z = np.array([p.transform.location.z for p in pts])
    s = np.concatenate([[0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    return s, x, y, psi, z


def ke_transform(ref, s_val, d, psi_lokal, rear_offset_x):
    """(jarak sepanjang jalan, simpangan lateral, arah relatif) -> carla.Transform."""
    s, x, y, psi, z = ref
    xr, yr = np.interp(s_val, s, x), np.interp(s_val, s, y)
    pr, zr = np.interp(s_val, s, psi), np.interp(s_val, s, z)

    # geser tegak lurus; normal kiri di frame right-handed = (-sin, cos)
    xw, yw, pw = xr - d * math.sin(pr), yr + d * math.cos(pr), pr + psi_lokal

    yaw_c = -math.degrees(pw)                       # right-handed -> CARLA
    rad = math.radians(yaw_c)
    # planner bekerja di titik sumbu belakang; actor origin ada di depannya
    return carla.Transform(
        carla.Location(x=xw - rear_offset_x * math.cos(rad),
                       y=-yw - rear_offset_x * math.sin(rad), z=zr + 0.05),
        carla.Rotation(yaw=yaw_c))


def rangkai_receding(dt):
    """Replan 10 Hz + BehaviorFSM -- receding horizon (bagian 2 dan 6).

    Tiap 100 ms seluruh proses diulang: bangkitkan 9 kandidat dari posisi dan
    kecepatan lateral SEKARANG, saring, pilih termurah. Yang dieksekusi hanya
    100 ms pertamanya, lalu dihitung ulang.

    Keputusan KAPAN menyalip diserahkan ke planning.BehaviorFSM; di sini tinggal
    mengeksekusi 100 ms pertama dari rencana yang dipilih.
    """
    d, dd, ddd, s = 0.0, 0.0, 0.0, 0.0
    fsm = planning.BehaviorFSM()
    jejak, langkah, riwayat = [], [], []

    for step in range(int(DURASI / (2 * dt))):
        t = step * 2 * dt
        gap = GAP_AWAL + V_TARGET * t - s
        obs = [[gap, 0.0, V_TARGET, 0.0]]          # target di lajur ego, y absolut

        state = fsm.update(t, d, V_EGO, obs)       # Tahap 4 -- FSM sungguhan
        y_goal = fsm.y_goal
        riwayat.append((round(t, 2), state, round(gap, 1), round(d, 2)))
        if abs(y_goal - d) < 0.05 and abs(dd) < 0.02:
            rencana, feasible = None, []           # sudah di tempat, tidak perlu manuver
        else:
            rencana, feasible = planning.plan_lane_change(
                d, dd, ddd, 0, V_EGO, 0, V_EGO, obstacles=obs, y_goal=y_goal)

        langkah.append((s, [(tr.states[0], tr.states[1], tr is rencana)
                            for _, _, _, tr in feasible]))

        for j in (1, 2):                            # eksekusi 2 tick = 100 ms
            tt = j * dt
            if rencana is None:                     # abort / tahan lajur
                dn, ddn, dddn = d, 0.0, 0.0
            else:
                dn, ddn, dddn = rencana.lateral_at(tt)
            jejak.append((s + V_EGO * tt, dn, math.atan2(ddn, V_EGO)))
        d, dd, ddd = dn, ddn, dddn
        s += V_EGO * 2 * dt

    return jejak, langkah, riwayat


BIRU = (60, 150, 255)               # kandidat
HIJAU = (30, 210, 70)               # yang dieksekusi
FOV = 90.0                          # derajat, horizontal


def _titik(ref, s_val, d, dz=0.35):
    """(jarak sepanjang jalan, simpangan lateral) -> titik dunia CARLA."""
    s, x, y, psi, z = ref
    xr, yr = np.interp(s_val, s, x), np.interp(s_val, s, y)
    pr, zr = np.interp(s_val, s, psi), np.interp(s_val, s, z)
    return np.array([xr - d * math.sin(pr), -(yr + d * math.cos(pr)), zr + dz, 1.0])


def bangun_overlay(ref, langkah, n_tick):
    """Kipas kandidat untuk tiap tick, dihitung ulang tiap langkah 10 Hz."""
    tampil = [None] * n_tick
    for step, (s0, kand) in enumerate(langkah):
        garis = []
        for xs, ys, terpilih in kand:
            k = np.arange(0, len(xs), 3)
            garis.append((np.array([_titik(ref, s0 + xs[j], ys[j]) for j in k]),
                          s0 + xs[k], terpilih))
        for i in (2 * step, 2 * step + 1):
            if i < n_tick:
                tampil[i] = garis
    return tampil


def proyeksi(titik_dunia, cam_inv, ukuran):
    """Titik dunia -> piksel. UE4 x-maju/y-kanan/z-atas -> kamera standar."""
    lebar, tinggi = ukuran
    p = titik_dunia @ cam_inv.T
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    f = lebar / (2.0 * math.tan(math.radians(FOV) / 2.0))
    depan = x > 0.5
    u = f * y / np.where(depan, x, 1) + lebar / 2
    v = f * (-z) / np.where(depan, x, 1) + tinggi / 2
    return u, v, depan


def gambar_overlay(i, berkas, tampil, cam_invs, s_ego, ukuran):
    garis = tampil[i] if i < len(tampil) else None
    if not garis:
        return
    img = Image.open(berkas).convert('RGB')
    lapis = Image.new('RGBA', img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(lapis)
    for pts, sv, terpilih in garis:
        u, v, depan = proyeksi(pts, cam_invs[i], ukuran)
        ok = depan & (sv > s_ego + 4.0)                      # hanya di depan mobil
        xy = [(u[k], v[k]) for k in range(len(u)) if ok[k]]
        if len(xy) > 1:
            warna, tebal = (HIJAU + (255,), 6) if terpilih else (BIRU + (170,), 3)
            d.line(xy, fill=warna, width=tebal, joint='curve')
    Image.alpha_composite(img.convert('RGBA'), lapis).convert('RGB').save(berkas)


KAMERA = {   # lokasi, rotasi, ukuran frame, nama berkas
    'kejar': (carla.Location(x=-11, z=7.0), carla.Rotation(pitch=-18),
              (960, 540), 'overtake_planner.mp4'),
    # potret: jalan itu sempit tapi panjang, frame lebar cuma memboroskan rumput
    'atas':  (carla.Location(x=20, z=20.0), carla.Rotation(pitch=-90),
              (540, 960), 'overtake_topdown.mp4'),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--kamera', choices=list(KAMERA), default='kejar')
    args = ap.parse_args()
    loc, rot, ukuran, nama_file = KAMERA[args.kamera]
    dt = config.FIXED_DELTA_SECONDS
    jejak, langkah, riwayat = rangkai_receding(dt)
    prev = None
    print(f"{'t':>6}{'state':>22}{'celah':>8}{'lateral':>9}")
    for t_, st, gap, d_ in riwayat:
        if st != prev:
            print(f'{t_:>6.2f}{st:>22}{gap:>8.1f}{d_:>9.2f}')
            prev = st
    rear = -1.4329552057945847        # dari out/vehicle_params.json

    with simulation.carla_world() as world:
        sp = world.get_map().get_spawn_points()[config.SPAWN_IDX]
        ref = jalur_acuan(world, sp.location)
        print(f'{len(jejak)} tick, {len(jejak)*dt:.0f} detik, '
              f'ego menempuh {jejak[-1][0]:.0f} m')

        pose_ego = [ke_transform(ref, s, d, p, rear) for s, d, p in jejak]
        overlay = bangun_overlay(ref, langkah, len(jejak))
        rel = np.array(carla.Transform(loc, rot).get_matrix())
        cam_invs = [np.linalg.inv(np.array(pe.get_matrix()) @ rel) for pe in pose_ego]
        pose_target = [ke_transform(ref, GAP_AWAL + V_TARGET * i * dt, 0.0, 0.0, rear)
                       for i in range(len(jejak))]

        bp = world.get_blueprint_library().find('vehicle.nissan.patrol')
        with simulation.ego_vehicle(world) as ego:
            target = world.spawn_actor(bp, pose_target[0])
            world.tick()
            try:
                record_path.capture(
                    world, ego, pose_ego, nama_file,
                    cam_tf=carla.Transform(loc, rot),
                    extras=[(target, pose_target)],
                    size=ukuran,
                    annotate=lambda i, f: gambar_overlay(
                        i, f, overlay, cam_invs, jejak[min(i, len(jejak) - 1)][0], ukuran))
            finally:
                target.destroy()


if __name__ == '__main__':
    main()
