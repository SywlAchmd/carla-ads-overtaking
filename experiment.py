"""Tahap 9 — eksperimen penuh, matriks bagian 11.4.

Mengulang satu konfigurasi N kali dan melaporkan success rate berikut sebaran
tiap metrik. Kriteria lulus ditetapkan di `config` SEBELUM eksperimen dijalankan
(bagian 11.2), jadi vonisnya tidak subjektif.

    python experiment.py --perception gt --repeat 5
    python experiment.py --perception vision --repeat 10

Berapa ulangan yang perlu berbeda menurut mode:

  * **gt** deterministik bit-per-bit, jadi ulangan bukan untuk success rate
    melainkan untuk MEMBUKTIKAN determinisme itu masih berlaku. Kalau dua run
    memberi angka berbeda, ada yang salah (bagian "Hal yang Perlu Diperhatikan").
  * **vision** memakai render kamera yang tidak deterministik, jadi ulangan
    memang dibutuhkan. Sebarannya sendiri ikut dilaporkan.

Jalankan di mesin senggang dan dengan server CARLA yang baru direstart: waktu
solve terukur 26-31 ms saat senggang versus 70 ms saat CARLA sudah lama berjalan.
"""
import argparse
import json
import time

import numpy as np

import config
import main
import simulation
import tune_vision
import yolopx

METRICS = [('min_dist', 'min body-to-body distance [m]', '{:.3f}'),
          ('duration', 'maneuver duration [s]', '{:.2f}'),
          ('lateral', 'max lateral excursion [m]', '{:.3f}'),
          ('dev', 'lane deviation, BEFORE maneuver [m]', '{:.4f}'),
          ('dev_tail', 'lane deviation, tail AFTER maneuver [m]', '{:.4f}'),
          ('decel', 'deepest deceleration [m/s2]', '{:.2f}'),
          ('no_candidates', 'ticks without planner candidates', '{:.1f}'),
          ('solve_mean', 'mean solve time [ms]', '{:.2f}'),
          ('solve_max', 'max solve time [ms]', '{:.2f}')]


def main_():
    ap = argparse.ArgumentParser()
    ap.add_argument('--perception', default='vision', choices=('gt', 'vision'))
    ap.add_argument('--repeat', type=int, default=10)
    args = ap.parse_args()

    params = json.load(open(config.VEHICLE_PARAMS_JSON))
    net = yolopx.YOLOPX() if args.perception == 'vision' else None
    if net is not None:
        print(f'YOLOPX epoch {net.epoch}, {net.device}')
    print(f'S1, MPC + {args.perception}, {args.repeat} repeats\n')

    result = []
    with simulation.carla_world() as world:
        ref, ref5 = main.prepare_road(world)
        print(f'{"run":>4}{"verdict":>10}{"min dist":>11}{"duration":>8}{"lateral":>9}'
              f'{"zero cand":>10}{"solve mean":>12}{"solve max":>12}{"failed":>7}{"seconds":>8}')
        for i in range(args.repeat):
            t0 = time.time()
            m = tune_vision.once(world, net, params, ref, ref5)
            result.append(m)
            print(f'{i + 1:>4}{m["verdict"]:>10}{m["min_dist"]:>11.3f}{m["duration"]:>8.2f}'
                  f'{m["lateral"]:>9.3f}{m["no_candidates"]:>10d}{m["solve_mean"]:>12.2f}'
                  f'{m["solve_max"]:>12.2f}{m["solver_failures"]:>7d}{time.time() - t0:>8.0f}')

    passed = sum(1 for m in result if m['verdict'] == 'SUCCESS')
    print(f'\nSUCCESS RATE  {passed}/{len(result)} = {100.0 * passed / len(result):.0f}%')
    print(f'{"metric":<38}{"mean":>12}{"sd":>11}{"min":>11}{"max":>11}')
    for key, label, fmt in METRICS:
        v = np.array([float(m[key]) for m in result])
        print(f'{label:<38}' + ''.join(f'{fmt.format(x):>12}' if j == 0 else f'{fmt.format(x):>11}'
                                       for j, x in enumerate((v.mean(), v.std(), v.min(), v.max()))))
    if args.perception == 'gt':
        # `solve_ms` DIKECUALIKAN: itu jam dinding, dan jam dinding memang tidak
        # deterministik -- memasukkannya membuat uji determinisme selalu gagal
        # padahal kendalinya identik. `equal_nan` karena kolom estimasi berisi
        # NaN saat tidak ada deteksi, dan NaN != NaN.
        columns = [i for i, n in enumerate(main.COLUMNS) if n != 'solve_ms']
        same = all(np.array_equal(result[0]['log'][:, columns], m['log'][:, columns],
                                  equal_nan=True) for m in result[1:])
        print(f'\nlogs bit-for-bit identical (excluding timing columns): {"YES" if same else "NO"}')
        if not same:
            print('  -> determinism broken; do not continue until the cause is found')

    # Log MENTAH tiap ulangan ikut disimpan (bagian 11.6). Ringkasan saja tidak
    # cukup: metrik bab 4 baru didefinisikan saat menulis, dan bagian 15.5
    # menunjukkan metrik yang tampak wajar bisa mengukur hal lain -- tanpa data
    # mentah, mendefinisikan ulang berarti menjalankan ulang seluruh eksperimen.
    path = f'{config.OUT_DIR}/experiment_s1_{args.perception}.npz'
    np.savez(path, **{k: np.array([m[k] for m in result]) for k, _, _ in METRICS},
             verdict=np.array([m['verdict'] for m in result]),
             solver_failures=np.array([m['solver_failures'] for m in result]),
             columns=main.COLUMNS,
             log=np.stack([m['log'] for m in result]),
             fsm_state=np.stack([m['states'] for m in result]))
    print(f'\n{path}')


if __name__ == '__main__':
    main_()
