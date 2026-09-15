"""Ukur YOLOPX terhadap ground truth simulator (rencana kerja bagian 10, 11.4).

Kendaraan target ditaruh pada beberapa jarak di depan ego, lalu tiap frame dinilai:
apakah target terdeteksi, berapa positif palsu, dan seberapa tepat jarak yang
dibaca dari depth camera. Angka pelatihan tidak dipakai -- split-nya masih bocor.

    python cek_deteksi.py                      # sapuan jarak, lajur ego
    python cek_deteksi.py --lajur 1            # target di lajur menyalip
"""
import argparse
import json

import cv2
import numpy as np

import config
import main
import sensors
import simulation
import yolopx

JARAK = (10, 15, 20, 25, 30, 40, 50, 60, 80)
AMBANG = (0.3, 0.4, 0.5, 0.6, 0.7)
IOU_COCOK = 0.3                    # kotak dianggap mengenai target di atas ini
SIMPAN = (15, 30, 60)              # jarak yang gambarnya disimpan


def matriks_kamera():
    w, h = config.KAMERA_LEBAR, config.KAMERA_TINGGI
    f = w / (2.0 * np.tan(np.radians(config.KAMERA_FOV) / 2.0))
    return np.array([[f, 0, w / 2.0], [0, f, h / 2.0], [0, 0, 1.0]])


def kotak_ground_truth(actor, kamera, K):
    """Kotak 2D target hasil proyeksi bounding box 3D-nya. Kembali (x1,y1,x2,y2) atau None."""
    w2c = np.array(kamera.get_transform().get_inverse_matrix())
    uv = []
    for v in actor.bounding_box.get_world_vertices(actor.get_transform()):
        p = w2c @ np.array([v.x, v.y, v.z, 1.0])
        p = np.array([p[1], -p[2], p[0]])          # sumbu UE -> sumbu kamera baku
        if p[2] < 0.1:
            continue
        piksel = K @ p
        uv.append(piksel[:2] / piksel[2])
    if len(uv) < 4:
        return None
    uv = np.array(uv)
    return (max(uv[:, 0].min(), 0), max(uv[:, 1].min(), 0),
            min(uv[:, 0].max(), config.KAMERA_LEBAR), min(uv[:, 1].max(), config.KAMERA_TINGGI))


def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    potong = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    luas = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - potong
    return potong / luas if luas > 0 else 0.0


def tempel_segmentasi(rgb, da, ll):
    """Warnai area jalan (hijau) dan garis lajur (merah) di atas citra."""
    out = rgb.copy()
    besar = lambda m: cv2.resize(m.astype(np.uint8), (rgb.shape[1], rgb.shape[0]),
                                 interpolation=cv2.INTER_NEAREST).astype(bool)
    out[besar(da)] = (0.5 * out[besar(da)] + 0.5 * np.array([0, 200, 0])).astype(np.uint8)
    out[besar(ll)] = (0.4 * out[besar(ll)] + 0.6 * np.array([0, 0, 255])).astype(np.uint8)
    return out


def main_():
    ap = argparse.ArgumentParser()
    ap.add_argument('--lajur', type=int, default=0, help='0 = lajur ego, 1 = lajur menyalip')
    ap.add_argument('--weight', default=None, help='bawaan: config.YOLOPX_WEIGHT (fine-tuned)')
    ap.add_argument('--tag', default='', help='akhiran nama berkas gambar')
    args = ap.parse_args()

    params = json.load(open(config.VEHICLE_PARAMS_JSON))
    net = yolopx.YOLOPX(weight=args.weight)
    K = matriks_kamera()
    print(f'YOLOPX epoch {net.epoch}, perangkat {net.device}, half={net.half}')
    print(f'{"jarak":>6}{"GT piksel":>11}{"deteksi":>9}{"conf":>6}{"IoU":>6}'
          f'{"depth":>8}{"ke pusat":>9}{"ke muka":>9}   positif palsu per ambang ' + str(AMBANG))

    with simulation.carla_world() as world:
        ref, ref5 = main.siapkan_jalan(world)
        with simulation.ego_vehicle(world) as ego:
            with sensors.RigKamera(world, ego, params) as rig:
                for _ in range(10):
                    simulation.tick(world)
                    rig.ambil()
                ego_x = main.localization.PathFrame(ref).ego(
                    main.localization.CarlaGTLocalization(ego, params['rear_axle_offset_x']).update()).x

                for jarak in JARAK:
                    target = main.spawn_kendaraan(world, ref5, ego_x, float(jarak), args.lajur)
                    target.set_simulate_physics(False)
                    for _ in range(4):
                        simulation.tick(world)
                        frame = rig.ambil()
                    rgb = np.frombuffer(frame['rgb'].raw_data, dtype=np.uint8).reshape(
                        frame['rgb'].height, frame['rgb'].width, 4)[:, :, :3][:, :, ::-1].copy()
                    depth = sensors.depth_meter(frame['depth'])

                    kotak, da, ll = net.infer(rgb, conf=min(AMBANG))
                    gt = kotak_ground_truth(target, rig.sensor['rgb'], K)
                    kam = rig.sensor['rgb'].get_transform().location
                    tl = target.get_transform().location
                    jarak_gt = np.hypot(tl.x - kam.x, tl.y - kam.y)

                    cocok = [(iou(gt, b[:4]), b) for b in kotak] if gt is not None else []
                    cocok = [c for c in cocok if c[0] >= IOU_COCOK]
                    terbaik = max(cocok, key=lambda c: c[1][4]) if cocok else None
                    palsu = [sum(1 for b in kotak
                                 if b[4] >= a and (gt is None or iou(gt, b[:4]) < IOU_COCOK))
                             for a in AMBANG]

                    if terbaik is not None:
                        b = terbaik[1]
                        cx, cy = int((b[0] + b[2]) / 2), int((b[1] + b[3]) / 2)
                        d = float(np.median(depth[max(cy - 3, 0):cy + 4, max(cx - 3, 0):cx + 4]))
                        # depth membaca permukaan yang terlihat, ground truth ke pusat bodi:
                        # selisihnya setengah panjang kendaraan (bagian 10 CATATAN)
                        muka = jarak_gt - config.LAIN_PANJANG / 2.0
                        print(f'{jarak:>6}{(gt[2]-gt[0]):>8.0f}px{"ya":>9}{b[4]:>6.2f}{terbaik[0]:>6.2f}'
                              f'{d:>8.1f}{d - jarak_gt:>+9.2f}{d - muka:>+9.2f}   {palsu}')
                    else:
                        lebar_gt = (gt[2] - gt[0]) if gt is not None else 0
                        print(f'{jarak:>6}{lebar_gt:>8.0f}px{"TIDAK":>9}{"-":>6}{"-":>6}'
                              f'{"-":>8}{"-":>9}{"-":>9}   {palsu}')

                    # positif palsu >= 0,5: di dalam area jalan atau tidak? Kalau di luar,
                    # menggerbangnya dengan segmentasi lebih murah daripada menaikkan ambang.
                    for b in kotak:
                        if b[4] >= 0.5 and (gt is None or iou(gt, b[:4]) < IOU_COCOK):
                            sy, sx = da.shape[0] / rgb.shape[0], da.shape[1] / rgb.shape[1]
                            cy, cx = int((b[1] + b[3]) / 2 * sy), int((b[0] + b[2]) / 2 * sx)
                            kaki = min(int(b[3] * sy), da.shape[0] - 1)   # tepi bawah kotak
                            print(f'       palsu conf {b[4]:.2f} kotak {[int(v) for v in b[:4]]} '
                                  f'| pusat di area jalan: {bool(da[cy, cx])}, '
                                  f'kaki di area jalan: {bool(da[kaki, cx])}')
                    if jarak in SIMPAN:
                        gambar = tempel_segmentasi(rgb, da, ll)[:, :, ::-1].copy()
                        for b in kotak:
                            if b[4] >= config.DETEKSI_CONF:
                                cv2.rectangle(gambar, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])),
                                              (0, 255, 255), 2)
                                cv2.putText(gambar, f'{b[4]:.2f}', (int(b[0]), int(b[1]) - 5),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                        if gt is not None:
                            cv2.rectangle(gambar, (int(gt[0]), int(gt[1])), (int(gt[2]), int(gt[3])),
                                          (255, 255, 255), 1)
                        jalur = f'{config.OUT_DIR}/deteksi_{jarak}m_lajur{args.lajur}{args.tag}.png'
                        cv2.imwrite(jalur, gambar)
                        print(f'       gambar -> {jalur}')
                    target.destroy()


if __name__ == '__main__':
    main_()
