"""Sapuan parameter di SKENARIO PENUH dengan vision perception (bagian 10.6).

`tuning.py` menyapu bobot MPC lewat step response garis lurus, tanpa halangan
dan tanpa FSM -- dan itu memang satu-satunya cara yang sah sebelumnya, karena
skenario penuh belum terulang. Setelah lup planner-MPC distabilkan (bagian
19.14), dua run S1 vision berkode identik memberi jarak bodi 2,12 / 2,12 m dan
tick nol kandidat 38 / 38, jadi **satu run kini menggambarkan satu konfigurasi**
dan sapuan skenario penuh menjadi sah.

Yang disapu di sini justru yang TIDAK bisa disentuh step response: ambang FSM,
bobot pemilihan kandidat planner, dan slack yang menengahi kenyamanan versus
jarak aman.

    python tune_vision.py --sweep K_DEV 10,20,40
    python tune_vision.py --sweep MPC_RHO_LAT 20,50,200
    python tune_vision.py --sweep MPC_Q:1 20,60,150     # entri tuple, indeks 1
    python tune_vision.py                       # konfigurasi sekarang saja

Parameter dibaca modul hilir lewat `config.<NAMA>` saat dipanggil, jadi cukup
ditimpa di modul config sebelum run. Ego di-spawn ULANG tiap konfigurasi --
pelajaran `tuning.py`: putaran roda terbawa dan hasilnya jadi bergantung urutan.
"""
import argparse
import json

import numpy as np

import config
import evaluation
import main
import sensors
import simulation
import yolopx

# 20 s, sama dengan main.py. 15 s sempat dicoba dan memberi vonis PALSU
# (lane_departure di semua konfigurasi): manuver selesai ~15 s dan run
# terpotong sebelum PASS_HOLD 2,0 s terpenuhi.
SECONDS = 20.0


def _dev(log, states, k, before):
    """|ego - y_goal| saat LANE_KEEPING, sebelum atau sesudah manuver."""
    out = np.nonzero(states != 'LANE_KEEPING')[0]
    if not len(out):
        return float('nan')
    lk = states == 'LANE_KEEPING'
    m = (np.arange(len(states)) < out[0]) if before else (
        lk & (np.arange(len(states)) > out[-1]))
    return float(np.abs(log[m, k['lane_dev']]).mean()) if m.any() else float('nan')


def once(world, net, params, ref, ref5, seconds=None):
    """Satu run S1 -> metrik. `net` None berarti ground truth, bukan vision.

    Ego dan rig baru tiap panggilan -- pelajaran `tuning.py`: putaran roda
    terbawa antar konfigurasi dan hasilnya jadi bergantung urutan.
    """
    vehicles = config.SCENARIOS['S1']
    seconds = SECONDS if seconds is None else seconds
    with simulation.ego_vehicle(world) as ego:
        simulation.tick(world, [simulation.velocity(ego, config.EGO_V0)])
        rig = sensors.CameraRig(world, ego, params) if net is not None else None
        actors = []
        try:
            with evaluation.watch_collisions(world, ego) as monitor:
                log, states, actors, positions = main.run(
                    world, ego, monitor, params, ref, ref5, seconds, vehicles, rig, net)
                dims = [(2 * a.bounding_box.extent.x, 2 * a.bounding_box.extent.y)
                        for a in actors]
        finally:
            for a in actors:
                a.destroy()
            if rig is not None:
                rig.destroy()

    k = {n: i for i, n in enumerate(main.COLUMNS)}
    success, category, details = evaluation.evaluate_run(
        log[:, k['t']], log[:, k['x']], log[:, k['y']], states,
        log[:, k['x_tgt']], log[:, k['y_tgt']], monitor.collisions,
        (params['length'], params['width']), dims[0], [],
        yaw=log[:, k['yaw']], yaw_tgt=positions[:, 0, 2])
    present = log[:, k['n_feasible']] > 0
    g = present[::2]                                   # planner bekerja 10 Hz
    return dict(
        verdict='SUCCESS' if success else f'FAILED ({category})',
        min_dist=details.get('min_dist', float('nan')),
        lateral=log[:, k['y']].min(),
        no_candidates=int((~present).sum()),
        flicker=sum(1 for i in range(len(g) - 1) if g[i] != g[i + 1]),
        decel=log[:, k['a_cmd']].min(),
        duration=details.get('duration', float('nan')),
        # DIPISAH sebelum/sesudah manuver. Digabung, angkanya hampir seluruhnya
        # berisi EKOR TRANSIEN setelah kembali ke lajur, bukan kualitas menjaga
        # lajur -- jebakan yang sudah tercatat di WRITING_SUMMARY.md bagian 15.5
        # untuk jalur GT, dan menggigit jauh lebih keras di jalur vision: terukur
        # 0,034 m sebelum manuver versus 0,247 m sesudahnya.
        dev=_dev(log, states, k, before=True),
        dev_tail=_dev(log, states, k, before=False),
        solve_mean=log[1:, k['solve_ms']].mean(),
        solve_max=log[1:, k['solve_ms']].max(),
        solver_failures=int((log[:, k['solver_ok']] == 0).sum()),
        log=log, states=states)


def main_():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sweep', nargs=2, metavar=('NAME', 'VALUES'),
                    help='example: --sweep K_DEV 10,20,40')
    args = ap.parse_args()

    idx = None
    if args.sweep:
        name, value = args.sweep[0], [float(v) for v in args.sweep[1].split(',')]
        if ':' in name:                            # entri tuple, mis. MPC_Q:1
            name, idx = name.split(':')[0], int(name.split(':')[1])
        assert hasattr(config, name), f'config has no {name}'
    else:
        name, value = None, [None]
    original = getattr(config, name) if name else None

    def pairs(v):
        if idx is None:
            setattr(config, name, type(original)(v))
        else:
            t = list(original)
            t[idx] = float(v)
            setattr(config, name, tuple(t))

    params = json.load(open(config.VEHICLE_PARAMS_JSON))
    net = yolopx.YOLOPX()                          # sekali, dipakai semua konfigurasi
    print(f'YOLOPX epoch {net.epoch}, {net.device}')
    if name:
        print(f'sweeping {name}: {original} (current) -> {value}\n')

    with simulation.carla_world() as world:
        ref, ref5 = main.prepare_road(world)
        title = name if idx is None else f'{name}[{idx}]'
        print(f'{title or "konfigurasi":>12}{"verdict":>10}{"min dist":>11}{"lateral":>9}'
              f'{"zero cand":>10}{"flicker":>9}{"decel":>8}{"duration":>8}{"deviation":>9}')
        for v in value:
            if name:
                pairs(v)
            m = once(world, net, params, ref, ref5)
            label = f'{v:g}' if name else 'current'
            print(f'{label:>12}{m["verdict"]:>10}{m["min_dist"]:>11.2f}{m["lateral"]:>9.2f}'
                  f'{m["no_candidates"]:>10d}{m["flicker"]:>9d}{m["decel"]:>8.2f}'
                  f'{m["duration"]:>8.1f}{m["dev"]:>9.3f}')
    if name:
        setattr(config, name, original)


if __name__ == '__main__':
    main_()
