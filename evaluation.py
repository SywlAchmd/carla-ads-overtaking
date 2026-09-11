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


def jarak_kotak(dx, dy, dim_a, dim_b):
    """Pemisahan antar bodi kendaraan, bukan antar titik pusat.

    Jarak pusat-ke-pusat menyesatkan: Charger 1,88 m dan Patrol 2,1 m lebar,
    jadi pusat berjarak 1,5 m sudah saling menembus.

    Pendekatan kotak sejajar sumbu. Sah di sini karena jalannya lurus dan sudut
    hadap ego tidak pernah melewati ~7 derajat saat pindah lajur.
    """
    celah_x = np.abs(dx) - (dim_a[0] + dim_b[0]) / 2.0
    celah_y = np.abs(dy) - (dim_a[1] + dim_b[1]) / 2.0
    return np.hypot(np.maximum(celah_x, 0.0), np.maximum(celah_y, 0.0))


def nilai_run(t, x, y, states, x_tgt, y_tgt, tabrakan, dim_ego, dim_tgt):
    """Menilai satu run terhadap lima syarat bagian 11.2.

    Kembali: (berhasil, kategori, rincian). Kategori mengikuti bagian 11.2 --
    `abort` BUKAN kegagalan sistem melainkan keputusan FSM untuk tidak menyalip.
    """
    r = {}
    keluar = np.nonzero(states != 'LANE_KEEPING')[0]

    # Syarat 4 dan 3 berlaku sepanjang run, bukan hanya saat manuver
    r['jarak_min'] = float(jarak_kotak(x_tgt - x, y_tgt - y, dim_ego, dim_tgt).min())
    r['tabrakan'] = bool(tabrakan)

    if len(keluar) == 0:
        r['catatan'] = 'FSM tidak pernah keluar dari LANE_KEEPING'
        return False, 'abort', r

    i0 = keluar[0]
    r['t_mulai'] = float(t[i0])

    # Syarat 1: ego melewati target sejauh panjang kendaraan
    lewat = np.nonzero((x - x_tgt) > dim_ego[0])[0]
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
