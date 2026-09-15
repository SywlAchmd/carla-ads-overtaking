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
        self.kejadian = []                    # (waktu, aktor, impuls)
        self.sensor.listen(self._catat)

    def _catat(self, e):
        i = e.normal_impulse
        self.kejadian.append((float(e.timestamp), str(e.other_actor.type_id),
                              math.sqrt(i.x ** 2 + i.y ** 2 + i.z ** 2)))

    @property
    def tabrakan(self):
        return len(self.kejadian) > 0

    def ringkas(self):
        if not self.kejadian:
            return 'tidak ada tabrakan'
        t, aktor, imp = self.kejadian[0]
        return (f'{len(self.kejadian)} tabrakan, pertama pada t={t:.2f} '
                f'dengan {aktor} (impuls {imp:.0f})')

    def destroy(self):
        self.sensor.stop()
        self.sensor.destroy()


@contextlib.contextmanager
def pantau_tabrakan(world, ego):
    m = CollisionMonitor(world, ego)
    try:
        yield m
    finally:
        m.destroy()


def _sudut(dx, dy, yaw, dim):
    """(N, 4, 2) sudut kotak: pusat di (dx, dy), hadap `yaw`, dimensi (panjang, lebar)."""
    hp, hl = dim[0] / 2.0, dim[1] / 2.0
    c, s = np.cos(yaw), np.sin(yaw)
    lok = np.array([[hp, hl], [hp, -hl], [-hp, -hl], [-hp, hl]])
    return np.stack([dx[:, None] + lok[:, 0] * c[:, None] - lok[:, 1] * s[:, None],
                     dy[:, None] + lok[:, 0] * s[:, None] + lok[:, 1] * c[:, None]], axis=-1)


def _titik_ke_sisi(p, a, b):
    """Jarak titik p (N,M,2) ke ruas a-b (N,K,2), hasil (N,M,K)."""
    ab = b - a                                        # (N,K,2)
    d = p[:, :, None, :] - a[:, None, :, :]           # (N,M,K,2)
    panjang = np.sum(ab ** 2, axis=-1)[:, None, :]
    t = np.clip(np.sum(d * ab[:, None, :, :], axis=-1) / np.maximum(panjang, 1e-12), 0.0, 1.0)
    proyeksi = a[:, None, :, :] + t[..., None] * ab[:, None, :, :]
    return np.hypot(*(p[:, :, None, :] - proyeksi).transpose(3, 0, 1, 2))


def _bertumpuk(A, B):
    """Sumbu pemisah (SAT) untuk dua kotak cembung: True bila saling menembus."""
    tumpuk = np.ones(len(A), dtype=bool)
    for kotak in (A, B):
        for i in (0, 1):                              # dua sumbu unik tiap kotak
            sisi = kotak[:, i + 1] - kotak[:, i]
            n = np.stack([-sisi[:, 1], sisi[:, 0]], axis=-1)
            n /= np.maximum(np.hypot(n[:, 0], n[:, 1]), 1e-12)[:, None]
            pa, pb = np.einsum('nkj,nj->nk', A, n), np.einsum('nkj,nj->nk', B, n)
            tumpuk &= ~((pa.max(1) < pb.min(1)) | (pb.max(1) < pa.min(1)))
    return tumpuk


def jarak_kotak(dx, dy, dim_a, dim_b, yaw_a=0.0, yaw_b=0.0):
    """Jarak antar BODI kendaraan, bukan antar titik pusat. (dx, dy) = pusat ke pusat.

    Jarak pusat-ke-pusat menyesatkan: Charger 1,88 m dan Patrol 1,93 m lebar,
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
    A = _sudut(dx, dy, yaw, dim_a)
    B = _sudut(np.zeros_like(dx), np.zeros_like(dy),
               np.broadcast_to(np.asarray(yaw_b, dtype=float), dx.shape), dim_b)
    tepi = lambda K: (K, np.roll(K, -1, axis=1))
    d = np.minimum(_titik_ke_sisi(A, *tepi(B)).min(axis=(1, 2)),
                   _titik_ke_sisi(B, *tepi(A)).min(axis=(1, 2)))
    return np.where(_bertumpuk(A, B), 0.0, d)


def nilai_run(t, x, y, states, x_tgt, y_tgt, tabrakan, dim_ego, dim_tgt, lain=(),
              yaw=None, yaw_tgt=0.0):
    """Menilai satu run terhadap lima syarat bagian 11.2.

    `x`, `y` = titik sumbu belakang ego (state MPC); jarak antar bodi diukur dari
    PUSAT bodi, jadi digeser SUMBU_KE_PUSAT menurut `yaw`. Syarat lajur tetap
    memakai `y` sumbu belakang, sama seperti acuan yang dijejak pengendali.
    `lain` = [(x, y, yaw, dim), ...] kendaraan selain target; ikut syarat jarak aman.
    Kembali: (berhasil, kategori, rincian). Kategori mengikuti bagian 11.2 --
    `abort` BUKAN kegagalan sistem melainkan keputusan FSM untuk tidak menyalip.
    """
    r = {}
    keluar = np.nonzero(states != 'LANE_KEEPING')[0]
    yaw = np.zeros_like(x) if yaw is None else np.asarray(yaw, dtype=float)
    xc = x + config.SUMBU_KE_PUSAT * np.cos(yaw)
    yc = y + config.SUMBU_KE_PUSAT * np.sin(yaw)

    # Syarat 4 dan 3 berlaku sepanjang run, bukan hanya saat manuver
    r['jarak_min'] = float(min(jarak_kotak(xo - xc, yo - yc, dim_ego, d, yaw, yo_yaw).min()
                               for xo, yo, yo_yaw, d in
                               [(x_tgt, y_tgt, yaw_tgt, dim_tgt), *lain]))
    r['tabrakan'] = bool(tabrakan)

    if len(keluar) == 0:
        r['catatan'] = 'FSM tidak pernah keluar dari LANE_KEEPING'
        return False, 'abort', r

    i0 = keluar[0]
    r['t_mulai'] = float(t[i0])

    # Syarat 1: ego melewati target sejauh panjang kendaraan
    lewat = np.nonzero((xc - x_tgt) > dim_ego[0])[0]
    r['melewati'] = bool(len(lewat))
    if not r['melewati']:
        r['catatan'] = 'ego tidak pernah melewati kendaraan target'
        return False, 'abort', r

    # Syarat 2: kembali ke lajur semula dan bertahan
    n_tahan = int(round(config.LULUS_TAHAN / (t[1] - t[0])))
    di_lajur = np.abs(y) < config.LULUS_LATERAL
    di_lajur[:lewat[0]] = False                       # hanya setelah melewati
    berjalan, i_selesai = 0, None
    for i, ada in enumerate(di_lajur):
        berjalan = berjalan + 1 if ada else 0
        if berjalan >= n_tahan:
            i_selesai = i
            break
    r['kembali_ke_lajur'] = i_selesai is not None
    if i_selesai is None:
        r['catatan'] = (f'tidak pernah bertahan {config.LULUS_TAHAN:.0f} detik '
                        f'dalam {config.LULUS_LATERAL} m dari tengah lajur')
        return False, 'lane_departure', r

    # Syarat 5: durasi manuver
    r['durasi'] = float(t[i_selesai] - t[i0])
    if r['durasi'] > config.BATAS_MANUVER:
        r['catatan'] = f'manuver {r["durasi"]:.1f} detik, batas {config.BATAS_MANUVER:.0f}'
        return False, 'timeout', r

    if r['tabrakan']:
        r['catatan'] = 'terjadi tabrakan'
        return False, 'collision', r

    if r['jarak_min'] <= config.JARAK_AMAN:
        r['catatan'] = f'jarak minimum {r["jarak_min"]:.2f} m <= {config.JARAK_AMAN}'
        return False, 'collision', r

    r['catatan'] = 'seluruh syarat terpenuhi'
    return True, 'berhasil', r


def ringkas_penilaian(berhasil, kategori, r):
    baris = [f'{"BERHASIL" if berhasil else "GAGAL"} — {kategori}: {r["catatan"]}']
    for kunci, fmt in (('durasi', '{:.1f} detik'), ('jarak_min', '{:.2f} m')):
        if kunci in r:
            baris.append(f'  {kunci:<12} {fmt.format(r[kunci])}')
    return chr(10).join(baris)
