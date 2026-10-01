"""Ukur YOLOPX terhadap ground truth simulator (rencana kerja bagian 10, 11.4).

Kendaraan target ditaruh pada beberapa jarak di depan ego, lalu tiap frame dinilai:
apakah target terdeteksi, berapa positif palsu, dan seberapa tepat jarak yang
dibaca dari depth camera. Angka pelatihan tidak dipakai -- split-nya masih bocor.

    python check_detection.py                      # sapuan jarak, lajur ego
    python check_detection.py --lane 1            # target di lajur menyalip
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

DISTANCE = (10, 15, 20, 25, 30, 40, 50, 60, 80)
THRESHOLD = (0.3, 0.4, 0.5, 0.6, 0.7)
IOU_MATCH = 0.3                    # kotak dianggap mengenai target di atas ini
SAVE = (15, 30, 60)              # jarak yang gambarnya disimpan


def camera_matrix():
    w, h = config.CAMERA_WIDTH, config.CAMERA_HEIGHT
    f = w / (2.0 * np.tan(np.radians(config.CAMERA_FOV) / 2.0))
    return np.array([[f, 0, w / 2.0], [0, f, h / 2.0], [0, 0, 1.0]])


def gt_boxes(actor, camera, K):
    """Kotak 2D target hasil proyeksi bounding box 3D-nya. Kembali (x1,y1,x2,y2) atau None."""
    w2c = np.array(camera.get_transform().get_inverse_matrix())
    uv = []
    for v in actor.bounding_box.get_world_vertices(actor.get_transform()):
        p = w2c @ np.array([v.x, v.y, v.z, 1.0])
        p = np.array([p[1], -p[2], p[0]])          # sumbu UE -> sumbu kamera baku
        if p[2] < 0.1:
            continue
        pixels = K @ p
        uv.append(pixels[:2] / pixels[2])
    if len(uv) < 4:
        return None
    uv = np.array(uv)
    return (max(uv[:, 0].min(), 0), max(uv[:, 1].min(), 0),
            min(uv[:, 0].max(), config.CAMERA_WIDTH), min(uv[:, 1].max(), config.CAMERA_HEIGHT))


def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / area if area > 0 else 0.0


def paste_segmentation(rgb, da, ll):
    """Warnai area jalan (hijau) dan garis lajur (merah) di atas citra."""
    out = rgb.copy()
    large = lambda m: cv2.resize(m.astype(np.uint8), (rgb.shape[1], rgb.shape[0]),
                                 interpolation=cv2.INTER_NEAREST).astype(bool)
    out[large(da)] = (0.5 * out[large(da)] + 0.5 * np.array([0, 200, 0])).astype(np.uint8)
    out[large(ll)] = (0.4 * out[large(ll)] + 0.6 * np.array([0, 0, 255])).astype(np.uint8)
    return out


def main_():
    ap = argparse.ArgumentParser()
    ap.add_argument('--lane', type=int, default=0, help='0 = ego lane, 1 = overtaking lane')
    ap.add_argument('--weight', default=None, help='default: config.YOLOPX_WEIGHT (fine-tuned)')
    ap.add_argument('--tag', default='', help='image file name suffix')
    args = ap.parse_args()

    params = json.load(open(config.VEHICLE_PARAMS_JSON))
    net = yolopx.YOLOPX(weight=args.weight)
    K = camera_matrix()
    print(f'YOLOPX epoch {net.epoch}, device {net.device}, half={net.half}')
    print(f'{"dist":>6}{"GT pixels":>11}{"detected":>9}{"conf":>6}{"IoU":>6}'
          f'{"depth":>8}{"to center":>9}{"to face":>9}   false positives per threshold ' + str(THRESHOLD))

    with simulation.carla_world() as world:
        ref, ref5 = main.prepare_road(world)
        with simulation.ego_vehicle(world) as ego:
            with sensors.CameraRig(world, ego, params) as rig:
                for _ in range(10):
                    simulation.tick(world)
                    rig.grab()
                ego_x = main.localization.PathFrame(ref).ego(
                    main.localization.CarlaGTLocalization(ego, params['rear_axle_offset_x']).update()).x

                for dist in DISTANCE:
                    target = main.spawn_vehicles(world, ref5, ego_x, float(dist), args.lane)
                    target.set_simulate_physics(False)
                    for _ in range(4):
                        simulation.tick(world)
                        frame = rig.grab()
                    rgb = np.frombuffer(frame['rgb'].raw_data, dtype=np.uint8).reshape(
                        frame['rgb'].height, frame['rgb'].width, 4)[:, :, :3][:, :, ::-1].copy()
                    depth = sensors.depth_meter(frame['depth'])

                    box, da, ll = net.infer(rgb, conf=min(THRESHOLD))
                    gt = gt_boxes(target, rig.sensor['rgb'], K)
                    cam = rig.sensor['rgb'].get_transform().location
                    tl = target.get_transform().location
                    dist_gt = np.hypot(tl.x - cam.x, tl.y - cam.y)

                    match = [(iou(gt, b[:4]), b) for b in box] if gt is not None else []
                    match = [c for c in match if c[0] >= IOU_MATCH]
                    best = max(match, key=lambda c: c[1][4]) if match else None
                    fake = [sum(1 for b in box
                                 if b[4] >= a and (gt is None or iou(gt, b[:4]) < IOU_MATCH))
                             for a in THRESHOLD]

                    if best is not None:
                        b = best[1]
                        cx, cy = int((b[0] + b[2]) / 2), int((b[1] + b[3]) / 2)
                        d = float(np.median(depth[max(cy - 3, 0):cy + 4, max(cx - 3, 0):cx + 4]))
                        # depth membaca permukaan yang terlihat, ground truth ke pusat bodi:
                        # selisihnya setengah panjang kendaraan (bagian 10 CATATAN)
                        face = dist_gt - config.OTHER_LENGTH / 2.0
                        print(f'{dist:>6}{(gt[2]-gt[0]):>8.0f}px{"yes":>9}{b[4]:>6.2f}{best[0]:>6.2f}'
                              f'{d:>8.1f}{d - dist_gt:>+9.2f}{d - face:>+9.2f}   {fake}')
                    else:
                        width_gt = (gt[2] - gt[0]) if gt is not None else 0
                        print(f'{dist:>6}{width_gt:>8.0f}px{"NO":>9}{"-":>6}{"-":>6}'
                              f'{"-":>8}{"-":>9}{"-":>9}   {fake}')

                    # positif palsu >= 0,5: di dalam area jalan atau tidak? Kalau di luar,
                    # menggerbangnya dengan segmentasi lebih murah daripada menaikkan ambang.
                    for b in box:
                        if b[4] >= 0.5 and (gt is None or iou(gt, b[:4]) < IOU_MATCH):
                            sy, sx = da.shape[0] / rgb.shape[0], da.shape[1] / rgb.shape[1]
                            cy, cx = int((b[1] + b[3]) / 2 * sy), int((b[0] + b[2]) / 2 * sx)
                            foot = min(int(b[3] * sy), da.shape[0] - 1)   # tepi bawah kotak
                            print(f'       false conf {b[4]:.2f} box {[int(v) for v in b[:4]]} '
                                  f'| center on road area: {bool(da[cy, cx])}, '
                                  f'foot on road area: {bool(da[foot, cx])}')
                    if dist in SAVE:
                        draw = paste_segmentation(rgb, da, ll)[:, :, ::-1].copy()
                        for b in box:
                            if b[4] >= config.DETECTION_CONF:
                                cv2.rectangle(draw, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])),
                                              (0, 255, 255), 2)
                                cv2.putText(draw, f'{b[4]:.2f}', (int(b[0]), int(b[1]) - 5),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                        if gt is not None:
                            cv2.rectangle(draw, (int(gt[0]), int(gt[1])), (int(gt[2]), int(gt[3])),
                                          (255, 255, 255), 1)
                        path = f'{config.OUT_DIR}/detection_{dist}m_lane{args.lane}{args.tag}.png'
                        cv2.imwrite(path, draw)
                        print(f'       image -> {path}')
                    target.destroy()


if __name__ == '__main__':
    main_()
