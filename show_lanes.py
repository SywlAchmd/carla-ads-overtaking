"""Gambar-gambar untuk bab metodologi.

    python show_lanes.py   ->  out/planner_candidates.png (tanpa CARLA)
                               out/lanes_topdown.png, out/lanes_camera.png
"""
import queue

import carla
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

import config
import localization
import planning
import simulation

LENGTH = 300.0


def lane_geometry(world, location):
    """Centerline lajur ego dan seluruh lajur searah di kanannya, frame right-handed."""
    wp0 = world.get_map().get_waypoint(location, project_to_road=True,
                                       lane_type=carla.LaneType.Driving)
    lanes, wp = [], wp0
    while wp is not None and wp.lane_type == carla.LaneType.Driving \
            and (wp.lane_id > 0) == (wp0.lane_id > 0):
        pts = list(simulation.walk_lane(wp, LENGTH, 1.0))
        xy = np.array([localization.carla_xy_yaw_to_rh(p.transform.location,
                                                       p.transform.rotation.yaw)[:2] for p in pts])
        lanes.append((xy, np.array([p.is_junction for p in pts]), wp.lane_width))
        wp = wp.get_right_lane()
    return lanes


def plot_topdown(lanes, path):
    """Koordinat sejajar jalan: s = maju, d = lateral (kerangka bagian 5)."""
    ego = lanes[0][0]
    p0 = ego[0]
    psi = np.arctan2(*(ego[20] - ego[0])[::-1])          # arah jalan
    R = np.array([[np.cos(psi), np.sin(psi)], [-np.sin(psi), np.cos(psi)]])

    fig, ax = plt.subplots(figsize=(14, 4.5))
    for i, (xy, is_jn, width) in enumerate(lanes):
        sd = (xy - p0) @ R.T
        label = 'Lajur ego' if i == 0 else f'Lajur kanan {i}'
        ax.plot(sd[:, 0], sd[:, 1], lw=2, label=f'{label}  (d = {sd[:, 1].mean():+.2f} m)')
        jn = sd[is_jn]
        if len(jn):
            ax.plot(jn[:, 0], jn[:, 1], '.', ms=3, color='crimson',
                    label='junction area' if i == 0 else None)
        ax.axhline(sd[:, 1].mean(), ls=':', lw=.6, color='gray')

    ax.plot(0, 0, 'k*', ms=18, label=f'Spawn {config.SPAWN_IDX}', zorder=5)
    ax.annotate('arah jalan', xy=(45, 0), xytext=(8, 0), va='center',
                arrowprops=dict(arrowstyle='-|>', lw=2, color='k'))
    ax.annotate('', xy=(150, -3.5), xytext=(150, 0),
                arrowprops=dict(arrowstyle='-|>', lw=2.5, color='tab:green'))
    ax.text(153, -1.9, 'manuver menyalip\ny_target = SIDE_SIGN x LANE_WIDTH = -3,50 m',
            fontsize=9, color='tab:green', va='center')

    ax.set_xlabel('s - distance along the road (m)')
    ax.set_ylabel('d - lateral offset (m)')
    ax.set_title(f'Test environment - Town04 spawn {config.SPAWN_IDX}, '
                 f'{LENGTH:.0f} m, 4 lajur searah 3,50 m')
    ax.set_ylim(-13, 4); ax.grid(alpha=.3)
    ax.legend(loc='upper right', fontsize=8, ncol=3)
    fig.tight_layout(); fig.savefig(path, dpi=150)
    print(f'Tampak atas : {path}')


def camera_shot(world, ego, path):
    bp = world.get_blueprint_library().find('sensor.camera.rgb')
    bp.set_attribute('image_size_x', '1280')
    bp.set_attribute('image_size_y', '720')
    cam = world.spawn_actor(bp, carla.Transform(carla.Location(x=-14, z=9),
                                                carla.Rotation(pitch=-22)), attach_to=ego)
    images = queue.Queue()
    cam.listen(images.put)
    try:
        for _ in range(12):          # beri waktu dunia ter-render
            world.tick()
            images.get(timeout=5.0)
        world.tick()
        images.get(timeout=5.0).save_to_disk(path)
    finally:
        cam.stop(); cam.destroy()
    print(f'Kamera      : {path}')


def plot_candidates(path):
    """Sampling & seleksi kandidat lintasan (bagian 5.4). Tidak butuh CARLA."""
    best, feasible = planning.plan_lane_change(0, 0, 0, 0, 13.9, 0, 13.9)
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(14, 4.6))

    for d in (0.0, config.SIDE_SIGN * config.LANE_WIDTH,
              config.SIDE_SIGN * 2 * config.LANE_WIDTH):
        ax.axhline(d, ls=':', lw=.8, color='gray')
    ax.axhline(config.SIDE_SIGN * config.LANE_WIDTH / 2, ls='--', lw=1, color='0.7')

    for cost, offset, T, tr in feasible:
        pilihan = tr is best
        ax.plot(tr.states[0], tr.states[1], lw=3 if pilihan else 1,
                color='tab:green' if pilihan else '0.65', zorder=3 if pilihan else 1,
                label=f'selected: offset {offset:.1f} m, T = {T:.1f} s, J = {cost:.1f}'
                      if pilihan else None)
        t = np.arange(tr.states.shape[1]) * tr.dt
        ddy = np.gradient(np.gradient(tr.states[1], tr.dt), tr.dt)
        ax2.plot(t, np.abs(ddy), lw=2.5 if pilihan else 1,
                 color='tab:green' if pilihan else '0.65', zorder=3 if pilihan else 1)

    # Kotak kecil di titik awal, bukan bodi sesuai skala: sumbu x tertekan ~7x
    # terhadap sumbu y, jadi bodi 5 m tergambar segemuk tembok dan justru
    # mengalihkan perhatian dari berkas lintasannya.
    ax.plot(0, 0, marker='s', ms=8, mfc='0.85', mec='0.3', mew=1.1, zorder=5)
    ax.annotate('ego', (0, 0), xytext=(0, -11), textcoords='offset points',
                ha='center', va='top', fontsize=8, color='0.3', zorder=6)
    ax.set_xlabel('x - along the road (m)'); ax.set_ylabel('y - lateral (m)')
    ax.set_title(f'{len(feasible)} of 9 candidates pass the feasibility filters')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(alpha=.3)

    ax2.axhline(config.MAX_LATERAL_ACCEL, color='crimson', lw=2,
                label=f'comfort limit {config.MAX_LATERAL_ACCEL:.0f} m/s2')
    ax2.set_xlabel('t (s)'); ax2.set_ylabel('|lateral acceleration| (m/s2)')
    ax2.set_title('Every candidate stays under the limit')
    # Ruang di atas batas: tanpa ini legend kanan atas menimpa garis merahnya.
    ax2.set_ylim(top=config.MAX_LATERAL_ACCEL * 1.28)
    ax2.legend(loc='upper right', fontsize=8); ax2.grid(alpha=.3)

    fig.suptitle('Local planner - quintic lateral + quartic longitudinal')
    fig.tight_layout(); fig.savefig(path, dpi=150)
    print(f'Kandidat    : {path}')


def main():
    plot_candidates(f'{config.OUT_DIR}/planner_candidates.png')
    with simulation.carla_world() as world:
        sp = world.get_map().get_spawn_points()[config.SPAWN_IDX]
        lanes = lane_geometry(world, sp.location)
        print(f'Lajur searah ditemukan: {len(lanes)} '
              f'(lebar {", ".join(f"{w:.2f}" for _, _, w in lanes)} m)')
        plot_topdown(lanes, f'{config.OUT_DIR}/lanes_topdown.png')
        with simulation.ego_vehicle(world) as ego:
            camera_shot(world, ego, f'{config.OUT_DIR}/lanes_camera.png')


if __name__ == '__main__':
    main()
