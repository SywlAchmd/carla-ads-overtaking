"""Tahap 9 — eksperimen penuh, matriks bagian 11.4.

Mengulang satu konfigurasi N kali dan melaporkan success rate berikut sebaran
tiap metrik. Kriteria lulus ditetapkan di `config` SEBELUM eksperimen dijalankan
(bagian 11.2), jadi vonisnya tidak subjektif.

    python experiment.py --perception gt --ulang 5
    python experiment.py --perception vision --ulang 10

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

METRIK = [('jarak_min', 'jarak min antar bodi [m]', '{:.3f}'),
          ('durasi', 'durasi manuver [s]', '{:.2f}'),
          ('lateral', 'simpangan lateral terjauh [m]', '{:.3f}'),
          ('dev', 'deviasi lajur saat LANE_KEEPING [m]', '{:.4f}'),
          ('rem', 'perlambatan terdalam [m/s2]', '{:.2f}'),
          ('nol_kandidat', 'tick tanpa kandidat planner', '{:.1f}'),
          ('solve_rata', 'waktu solve rata-rata [ms]', '{:.2f}'),
          ('solve_maks', 'waktu solve maksimum [ms]', '{:.2f}')]


def main_():
    ap = argparse.ArgumentParser()
    ap.add_argument('--perception', default='vision', choices=('gt', 'vision'))
    ap.add_argument('--ulang', type=int, default=10)
    args = ap.parse_args()

    params = json.load(open(config.VEHICLE_PARAMS_JSON))
    net = yolopx.YOLOPX() if args.perception == 'vision' else None
    if net is not None:
        print(f'YOLOPX epoch {net.epoch}, {net.device}')
    print(f'S1, MPC + {args.perception}, {args.ulang} ulangan\n')

    hasil = []
    with simulation.carla_world() as world:
        ref, ref5 = main.siapkan_jalan(world)
        print(f'{"run":>4}{"vonis":>10}{"jarak min":>11}{"durasi":>8}{"lateral":>9}'
              f'{"nol kand":>10}{"solve rata":>12}{"solve maks":>12}{"gagal":>7}{"detik":>8}')
        for i in range(args.ulang):
            t0 = time.time()
            m = tune_vision.sekali(world, net, params, ref, ref5)
            hasil.append(m)
            print(f'{i + 1:>4}{m["vonis"]:>10}{m["jarak_min"]:>11.3f}{m["durasi"]:>8.2f}'
                  f'{m["lateral"]:>9.3f}{m["nol_kandidat"]:>10d}{m["solve_rata"]:>12.2f}'
                  f'{m["solve_maks"]:>12.2f}{m["gagal_solver"]:>7d}{time.time() - t0:>8.0f}')

    lulus = sum(1 for m in hasil if m['vonis'] == 'BERHASIL')
    print(f'\nSUCCESS RATE  {lulus}/{len(hasil)} = {100.0 * lulus / len(hasil):.0f}%')
    print(f'{"metrik":<38}{"rata-rata":>12}{"sd":>11}{"min":>11}{"maks":>11}')
    for kunci, label, fmt in METRIK:
        v = np.array([float(m[kunci]) for m in hasil])
        print(f'{label:<38}' + ''.join(f'{fmt.format(x):>12}' if j == 0 else f'{fmt.format(x):>11}'
                                       for j, x in enumerate((v.mean(), v.std(), v.min(), v.max()))))
    if args.perception == 'gt':
        # `solve_ms` DIKECUALIKAN: itu jam dinding, dan jam dinding memang tidak
        # deterministik -- memasukkannya membuat uji determinisme selalu gagal
        # padahal kendalinya identik. `equal_nan` karena kolom estimasi berisi
        # NaN saat tidak ada deteksi, dan NaN != NaN.
        kolom = [i for i, n in enumerate(main.KOLOM) if n != 'solve_ms']
        sama = all(np.array_equal(hasil[0]['log'][:, kolom], m['log'][:, kolom],
                                  equal_nan=True) for m in hasil[1:])
        print(f'\nlog identik bit-per-bit (tanpa kolom waktu): {"YA" if sama else "TIDAK"}')
        if not sama:
            print('  -> determinisme rusak; jangan lanjut sebelum sebabnya ketemu')

    # Log MENTAH tiap ulangan ikut disimpan (bagian 11.6). Ringkasan saja tidak
    # cukup: metrik bab 4 baru didefinisikan saat menulis, dan bagian 15.5
    # menunjukkan metrik yang tampak wajar bisa mengukur hal lain -- tanpa data
    # mentah, mendefinisikan ulang berarti menjalankan ulang seluruh eksperimen.
    jalur = f'{config.OUT_DIR}/experiment_s1_{args.perception}.npz'
    np.savez(jalur, **{k: np.array([m[k] for m in hasil]) for k, _, _ in METRIK},
             vonis=np.array([m['vonis'] for m in hasil]),
             gagal_solver=np.array([m['gagal_solver'] for m in hasil]),
             kolom=main.KOLOM,
             log=np.stack([m['log'] for m in hasil]),
             fsm_state=np.stack([m['states'] for m in hasil]))
    print(f'\n{jalur}')


if __name__ == '__main__':
    main_()
