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

    python tuning_vision.py --sweep K_DEV 10,20,40
    python tuning_vision.py --sweep MPC_RHO_LAT 20,50,200
    python tuning_vision.py --sweep MPC_Q:1 20,60,150     # entri tuple, indeks 1
    python tuning_vision.py                       # konfigurasi sekarang saja

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
# terpotong sebelum LULUS_TAHAN 2,0 s terpenuhi.
DETIK = 20.0


def sekali(world, net, params, ref, ref5, detik=None):
    """Satu run S1 -> metrik. `net` None berarti ground truth, bukan vision.

    Ego dan rig baru tiap panggilan -- pelajaran `tuning.py`: putaran roda
    terbawa antar konfigurasi dan hasilnya jadi bergantung urutan.
    """
    kendaraan = config.SKENARIO['S1']
    detik = DETIK if detik is None else detik
    with simulation.ego_vehicle(world) as ego:
        simulation.tick(world, [simulation.kecepatan(ego, config.EGO_V0)])
        rig = sensors.RigKamera(world, ego, params) if net is not None else None
        aktor = []
        try:
            with evaluation.pantau_tabrakan(world, ego) as monitor:
                log, states, aktor, posisi = main.run(
                    world, ego, monitor, params, ref, ref5, detik, kendaraan, rig, net)
                dims = [(2 * a.bounding_box.extent.x, 2 * a.bounding_box.extent.y)
                        for a in aktor]
        finally:
            for a in aktor:
                a.destroy()
            if rig is not None:
                rig.destroy()

    k = {n: i for i, n in enumerate(main.KOLOM)}
    berhasil, kategori, rincian = evaluation.nilai_run(
        log[:, k['t']], log[:, k['x']], log[:, k['y']], states,
        log[:, k['x_tgt']], log[:, k['y_tgt']], monitor.tabrakan,
        (params['length'], params['width']), dims[0], [],
        yaw=log[:, k['yaw']], yaw_tgt=posisi[:, 0, 2])
    ada = log[:, k['n_layak']] > 0
    g = ada[::2]                                   # planner bekerja 10 Hz
    return dict(
        vonis='BERHASIL' if berhasil else f'GAGAL ({kategori})',
        jarak_min=rincian.get('jarak_min', float('nan')),
        lateral=log[:, k['y']].min(),
        nol_kandidat=int((~ada).sum()),
        kedipan=sum(1 for i in range(len(g) - 1) if g[i] != g[i + 1]),
        rem=log[:, k['a_cmd']].min(),
        durasi=rincian.get('durasi', float('nan')),
        dev=np.abs(log[states == 'LANE_KEEPING', k['dev_lajur']]).mean(),
        solve_rata=log[1:, k['solve_ms']].mean(),
        solve_maks=log[1:, k['solve_ms']].max(),
        gagal_solver=int((log[:, k['solver_ok']] == 0).sum()),
        log=log, states=states)


def main_():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sweep', nargs=2, metavar=('NAMA', 'NILAI'),
                    help='contoh: --sweep K_DEV 10,20,40')
    args = ap.parse_args()

    idx = None
    if args.sweep:
        nama, nilai = args.sweep[0], [float(v) for v in args.sweep[1].split(',')]
        if ':' in nama:                            # entri tuple, mis. MPC_Q:1
            nama, idx = nama.split(':')[0], int(nama.split(':')[1])
        assert hasattr(config, nama), f'config tidak punya {nama}'
    else:
        nama, nilai = None, [None]
    asli = getattr(config, nama) if nama else None

    def pasang(v):
        if idx is None:
            setattr(config, nama, type(asli)(v))
        else:
            t = list(asli)
            t[idx] = float(v)
            setattr(config, nama, tuple(t))

    params = json.load(open(config.VEHICLE_PARAMS_JSON))
    net = yolopx.YOLOPX()                          # sekali, dipakai semua konfigurasi
    print(f'YOLOPX epoch {net.epoch}, {net.device}')
    if nama:
        print(f'menyapu {nama}: {asli} (sekarang) -> {nilai}\n')

    with simulation.carla_world() as world:
        ref, ref5 = main.siapkan_jalan(world)
        judul = nama if idx is None else f'{nama}[{idx}]'
        print(f'{judul or "konfigurasi":>12}{"vonis":>10}{"jarak min":>11}{"lateral":>9}'
              f'{"nol kand":>10}{"kedipan":>9}{"rem":>8}{"durasi":>8}{"deviasi":>9}')
        for v in nilai:
            if nama:
                pasang(v)
            m = sekali(world, net, params, ref, ref5)
            label = f'{v:g}' if nama else 'sekarang'
            print(f'{label:>12}{m["vonis"]:>10}{m["jarak_min"]:>11.2f}{m["lateral"]:>9.2f}'
                  f'{m["nol_kandidat"]:>10d}{m["kedipan"]:>9d}{m["rem"]:>8.2f}'
                  f'{m["durasi"]:>8.1f}{m["dev"]:>9.3f}')
    if nama:
        setattr(config, nama, asli)


if __name__ == '__main__':
    main_()
