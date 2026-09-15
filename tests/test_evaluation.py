"""Uji kriteria keberhasilan bagian 11.2 dengan run sintetis. Tanpa CARLA."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config                                                          # noqa: E402
import evaluation as E                                                 # noqa: E402

DIM_EGO, DIM_TGT = (5.01, 1.88), (5.50, 2.10)
DT, V_EGO, V_TGT = 0.05, 13.4, 7.0


def run_sintetis(detik=20.0, salip=True, tunda_kembali=0.0, gap_awal=60.0):
    """Ego menyalip target: lurus -> geser ke -3,5 -> lewat -> kembali."""
    t = np.arange(0, detik, DT)
    x = V_EGO * t
    x_tgt = gap_awal + V_TGT * t
    y_tgt = np.zeros_like(t)
    y = np.zeros_like(t)
    states = np.array(['LANE_KEEPING'] * len(t), dtype=object)
    if salip:
        mulai, T = 3.0, 4.0
        for i, tt in enumerate(t):
            s = np.clip((tt - mulai) / T, 0, 1)
            keluar = -3.5 * s ** 3 * (10 - 15 * s + 6 * s ** 2)
            kembali_mulai = mulai + T + 3.0 + tunda_kembali
            u = np.clip((tt - kembali_mulai) / T, 0, 1)
            y[i] = keluar + 3.5 * u ** 3 * (10 - 15 * u + 6 * u ** 2)
            if mulai <= tt < kembali_mulai + T:
                states[i] = 'OVERTAKING'
    return t, x, y, states, x_tgt, y_tgt


def test_run_bersih_dinyatakan_berhasil():
    t, x, y, st, xt, yt = run_sintetis()
    ok, kat, r = E.nilai_run(t, x, y, st, xt, yt, False, DIM_EGO, DIM_TGT)
    assert ok and kat == 'berhasil', (kat, r)
    assert r['jarak_min'] > config.JARAK_AMAN
    assert r['durasi'] < config.BATAS_MANUVER


def test_tabrakan_menggagalkan():
    t, x, y, st, xt, yt = run_sintetis()
    ok, kat, _ = E.nilai_run(t, x, y, st, xt, yt, True, DIM_EGO, DIM_TGT)
    assert not ok and kat == 'collision'


def test_tidak_pernah_menyalip_dihitung_abort():
    """Bagian 11.2: abort BUKAN kegagalan sistem, melainkan keputusan FSM."""
    t, x, y, st, xt, yt = run_sintetis(salip=False)
    ok, kat, _ = E.nilai_run(t, x, y, st, xt, yt, False, DIM_EGO, DIM_TGT)
    assert not ok and kat == 'abort'


def test_tidak_kembali_ke_lajur_dihitung_lane_departure():
    t, x, y, st, xt, yt = run_sintetis(detik=12.0, tunda_kembali=90.0)
    ok, kat, _ = E.nilai_run(t, x, y, st, xt, yt, False, DIM_EGO, DIM_TGT)
    assert not ok and kat == 'lane_departure'


def test_manuver_terlalu_lama_dihitung_timeout():
    t, x, y, st, xt, yt = run_sintetis(detik=40.0, tunda_kembali=22.0)
    ok, kat, r = E.nilai_run(t, x, y, st, xt, yt, False, DIM_EGO, DIM_TGT)
    assert not ok and kat == 'timeout', (kat, r)
    assert r['durasi'] > config.BATAS_MANUVER


def test_jarak_diukur_antar_bodi_bukan_antar_pusat():
    """Pusat berjarak 1,5 m lateral berarti kedua bodi SUDAH bertumpuk."""
    d = E.jarak_kotak(np.array([0.0]), np.array([1.5]), DIM_EGO, DIM_TGT)
    assert d[0] == 0.0, d
    # berdampingan satu lajur penuh: bersih
    d = E.jarak_kotak(np.array([0.0]), np.array([3.5]), DIM_EGO, DIM_TGT)
    assert abs(d[0] - (3.5 - (1.88 + 2.10) / 2)) < 1e-9, d
    # berurutan sejajar
    d = E.jarak_kotak(np.array([10.0]), np.array([0.0]), DIM_EGO, DIM_TGT)
    assert abs(d[0] - (10.0 - (5.01 + 5.50) / 2)) < 1e-9, d


def test_berdampingan_terlalu_rapat_digagalkan():
    """Menyalip dengan simpangan lateral cuma 2,5 m -> bodi berjarak 0,51 m."""
    t, x, y, st, xt, yt = run_sintetis()
    y = y * (2.5 / 3.5)
    ok, kat, r = E.nilai_run(t, x, y, st, xt, yt, False, DIM_EGO, DIM_TGT)
    assert not ok and kat == 'collision', (kat, r)
    assert r['jarak_min'] <= config.JARAK_AMAN


def test_yaw_ego_memperkecil_jarak_bodi():
    """Kotak ego ikut berputar. Mengabaikan yaw melebihkan jarak: pada 5 derajat
    dan simpangan lateral 3,5 m, selisihnya ~0,2 m."""
    lurus = E.jarak_kotak(np.array([0.0]), np.array([3.5]), DIM_EGO, DIM_TGT)[0]
    miring = E.jarak_kotak(np.array([0.0]), np.array([3.5]), DIM_EGO, DIM_TGT,
                           np.radians(5.0))[0]
    assert lurus - miring > 0.15, (lurus, miring)
    # sejajar sumbu harus tetap sama dengan rumus analitik
    assert abs(lurus - (3.5 - (DIM_EGO[1] + DIM_TGT[1]) / 2)) < 1e-9
    assert E.jarak_kotak(np.array([10.0]), np.array([0.0]), DIM_EGO, DIM_TGT,
                         np.radians(5.0))[0] > 0.0


if __name__ == '__main__':
    for nama, fn in sorted(globals().items()):
        if nama.startswith('test_'):
            fn()
            print(f'ok  {nama}')
    print('semua lolos')
