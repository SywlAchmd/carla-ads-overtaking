"""Instrumen pengukuran (rencana kerja bagian 2.1, 11.2, 11.6).

Modul ini BOLEH mengimpor carla. Larangan aturan 2.4 hanya berlaku untuk
`planning.py` dan `control.py`, dan justru larangan itulah yang menegakkan batas
di bawah secara struktural: keduanya tidak bisa membaca sensor tabrakan bahkan
kalau ada yang mencoba, karena mereka tidak boleh menyentuh CARLA sama sekali.

BATAS: sensor tabrakan dipakai untuk MENILAI run, tidak pernah jadi masukan
kendali. Kendaraan uji tidak memiliki akses terhadap informasi ini.
"""
import contextlib
import math

import carla
import numpy as np

import config


class CollisionMonitor:
    """Mencatat tabrakan. Tidak menghentikan simulasi.

    Run yang menabrak tetap dijalankan sampai selesai -- menabrak adalah HASIL
    yang dicatat, bukan error. Menghentikan simulasi berarti kehilangan data
    setelah tabrakan dan tidak bisa menganalisis apa yang terjadi.
    """

    def __init__(self, world, ego):
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=ego)
        self.events = []                    # (waktu, aktor, impuls)
        self.sensor.listen(self._record)

    def _record(self, e):
        i = e.normal_impulse
        self.events.append((float(e.timestamp), str(e.other_actor.type_id),
                              math.sqrt(i.x ** 2 + i.y ** 2 + i.z ** 2)))

    @property
    def collisions(self):
        return len(self.events) > 0

    def summary(self):
        if not self.events:
            return 'no collisions'
        t, actors, imp = self.events[0]
        return (f'{len(self.events)} collision(s), first at t={t:.2f} '
                f'with {actors} (impulse {imp:.0f})')

    def destroy(self):
        self.sensor.stop()
        self.sensor.destroy()


@contextlib.contextmanager
def watch_collisions(world, ego):
    m = CollisionMonitor(world, ego)
    try:
        yield m
    finally:
        m.destroy()


def _angle(dx, dy, yaw, dim):
    """(N, 4, 2) sudut kotak: pusat di (dx, dy), hadap `yaw`, dimensi (panjang, lebar)."""
    hp, hl = dim[0] / 2.0, dim[1] / 2.0
    c, s = np.cos(yaw), np.sin(yaw)
    corners = np.array([[hp, hl], [hp, -hl], [-hp, -hl], [-hp, hl]])
    return np.stack([dx[:, None] + corners[:, 0] * c[:, None] - corners[:, 1] * s[:, None],
                     dy[:, None] + corners[:, 0] * s[:, None] + corners[:, 1] * c[:, None]], axis=-1)


def _point_to_edge(p, a, b):
    """Jarak titik p (N,M,2) ke ruas a-b (N,K,2), hasil (N,M,K)."""
    ab = b - a                                        # (N,K,2)
    d = p[:, :, None, :] - a[:, None, :, :]           # (N,M,K,2)
    length = np.sum(ab ** 2, axis=-1)[:, None, :]
    t = np.clip(np.sum(d * ab[:, None, :, :], axis=-1) / np.maximum(length, 1e-12), 0.0, 1.0)
    projection = a[:, None, :, :] + t[..., None] * ab[:, None, :, :]
    return np.hypot(*(p[:, :, None, :] - projection).transpose(3, 0, 1, 2))


def _overlapping(A, B):
    """Sumbu pemisah (SAT) untuk dua kotak cembung: True bila saling menembus."""
    stack = np.ones(len(A), dtype=bool)
    for box in (A, B):
        for i in (0, 1):                              # dua sumbu unik tiap kotak
            side = box[:, i + 1] - box[:, i]
            n = np.stack([-side[:, 1], side[:, 0]], axis=-1)
            n /= np.maximum(np.hypot(n[:, 0], n[:, 1]), 1e-12)[:, None]
            pa, pb = np.einsum('nkj,nj->nk', A, n), np.einsum('nkj,nj->nk', B, n)
            stack &= ~((pa.max(1) < pb.min(1)) | (pb.max(1) < pa.min(1)))
    return stack


def box_distance(dx, dy, dim_a, dim_b, yaw_a=0.0, yaw_b=0.0):
    """Jarak antar BODI kendaraan, bukan antar titik pusat. (dx, dy) = pusat ke pusat.

    Jarak pusat-ke-pusat menyesatkan: Charger 1,88 m dan MKZ 1,84 m lebar,
    jadi pusat berjarak 1,5 m sudah saling menembus.

    Kedua kotak diputar menurut sudut hadapnya masing-masing. Yang dipaksa
    searah jalan hanya KECEPATAN kendaraan lain, arah hadapnya tetap bebas.
    Sudut hadap ego terukur sampai 5,1 derajat saat kedua bodi berdampingan;
    mengabaikannya melebihkan jarak sampai ~0,2 m.

    Untuk poligon cembung yang terpisah, jarak minimum selalu jatuh di pasangan
    titik-sudut ke sisi, jadi cukup memeriksa kedua arah; yang bertumpuk = 0.
    """
    dx, dy = np.atleast_1d(np.asarray(dx, dtype=float)), np.atleast_1d(np.asarray(dy, dtype=float))
    yaw = np.broadcast_to(np.asarray(yaw_a, dtype=float), dx.shape)
    A = _angle(dx, dy, yaw, dim_a)
    B = _angle(np.zeros_like(dx), np.zeros_like(dy),
               np.broadcast_to(np.asarray(yaw_b, dtype=float), dx.shape), dim_b)
    edges = lambda K: (K, np.roll(K, -1, axis=1))
    d = np.minimum(_point_to_edge(A, *edges(B)).min(axis=(1, 2)),
                   _point_to_edge(B, *edges(A)).min(axis=(1, 2)))
    return np.where(_overlapping(A, B), 0.0, d)


def evaluate_run(t, x, y, states, x_tgt, y_tgt, collisions, dim_ego, dim_tgt, other=(),
              yaw=None, yaw_tgt=0.0, y_lane=None):
    """Menilai satu run terhadap lima syarat bagian 11.2.

    `x`, `y` = titik sumbu belakang ego (state MPC); jarak antar bodi diukur dari
    PUSAT bodi, jadi digeser AXLE_TO_CENTER menurut `yaw`. Syarat lajur tetap
    memakai `y` sumbu belakang, sama seperti acuan yang dijejak pengendali.
    `other` = [(x, y, yaw, dim), ...] kendaraan selain target; ikut syarat jarak aman.
    `y_lane` = simpangan lateral untuk SYARAT LAJUR saja; None berarti memakai `y`.
    Dipisah sejak jangkar peta dibuang (WRITING_SUMMARY.md bagian 28.3): `y`
    kemudian ada di frame yang dijangkarkan kamera, dan menilai "kembali ke lajur"
    dengannya berarti bertanya apakah ego kembali ke lajur yang DIYAKININYA
    sendiri. Syarat jarak antar bodi tidak terpengaruh -- ia selisih dua titik,
    jadi nilainya sama di frame mana pun.
    Kembali: (berhasil, kategori, rincian). Kategori mengikuti bagian 11.2 --
    `abort` BUKAN kegagalan sistem melainkan keputusan FSM untuk tidak menyalip.
    """
    r = {}
    out = np.nonzero(states != 'LANE_KEEPING')[0]
    yaw = np.zeros_like(x) if yaw is None else np.asarray(yaw, dtype=float)
    xc = x + config.AXLE_TO_CENTER * np.cos(yaw)
    yc = y + config.AXLE_TO_CENTER * np.sin(yaw)

    # Syarat 4 dan 3 berlaku sepanjang run, bukan hanya saat manuver
    r['min_dist'] = float(min(box_distance(xo - xc, yo - yc, dim_ego, d, yaw, yo_yaw).min()
                               for xo, yo, yo_yaw, d in
                               [(x_tgt, y_tgt, yaw_tgt, dim_tgt), *other]))
    r['collision'] = bool(collisions)

    if len(out) == 0:
        r['note'] = 'FSM never left LANE_KEEPING'
        return False, 'abort', r

    i0 = out[0]
    r['t_start'] = float(t[i0])

    # Syarat 1: ego melewati target sejauh panjang kendaraan
    passed = np.nonzero((xc - x_tgt) > dim_ego[0])[0]
    r['passed'] = bool(len(passed))
    if not r['passed']:
        r['note'] = 'ego never passed the target vehicle'
        return False, 'abort', r

    # Syarat 2: kembali ke lajur semula dan bertahan
    n_hold = int(round(config.PASS_HOLD / (t[1] - t[0])))
    in_lane = np.abs(y if y_lane is None else y_lane) < config.PASS_LATERAL
    in_lane[:passed[0]] = False                       # hanya setelah melewati
    moving, i_done = 0, None
    for i, present in enumerate(in_lane):
        moving = moving + 1 if present else 0
        if moving >= n_hold:
            i_done = i
            break
    r['returned_to_lane'] = i_done is not None
    if i_done is None:
        r['note'] = (f'never held {config.PASS_HOLD:.0f} s '
                        f'within {config.PASS_LATERAL} m of the lane center')
        return False, 'lane_departure', r

    # Syarat 5: durasi manuver
    r['duration'] = float(t[i_done] - t[i0])
    if r['duration'] > config.MANEUVER_LIMIT:
        r['note'] = f'maneuver {r["duration"]:.1f} s, limit {config.MANEUVER_LIMIT:.0f}'
        return False, 'timeout', r

    if r['collision']:
        r['note'] = 'a collision occurred'
        return False, 'collision', r

    if r['min_dist'] <= config.SAFE_DISTANCE:
        r['note'] = f'minimum distance {r["min_dist"]:.2f} m <= {config.SAFE_DISTANCE}'
        return False, 'collision', r

    r['note'] = 'all criteria met'
    return True, 'success', r


def summarize_verdict(success, category, r):
    rows = [f'{"SUCCESS" if success else "FAILED"} — {category}: {r["note"]}']
    for key, fmt in (('duration', '{:.1f} s'), ('min_dist', '{:.2f} m')):
        if key in r:
            rows.append(f'  {key:<12} {fmt.format(r[key])}')
    return chr(10).join(rows)
