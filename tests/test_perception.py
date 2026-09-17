"""Uji koreksi permukaan -> pusat bodi. Tanpa simulator, tanpa torch.

Yang dikunci: geseran berpindah sumbu mengikuti permukaan yang terlihat. Versi
pertama menambahkan setengah panjang tanpa syarat dan melaporkan target sampai
+3,82 m terlalu jauh ke depan saat ego berdampingan (bagian 19.6).
"""
import os
import sys

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, AKAR)

import config
import perception


def _kotak(rasio, tinggi=100.0):
    return [0.0, 0.0, rasio * tinggi, tinggi]


def test_tampak_belakang_menggeser_memanjang_saja():
    dx, dy = perception.koreksi_muka(_kotak(config.AR_BELAKANG))
    assert abs(dx - config.LAIN_PANJANG / 2) < 1e-6, dx
    assert dy == 0.0


def test_tampak_samping_menggeser_melintang_saja():
    dx, dy = perception.koreksi_muka(_kotak(config.AR_SAMPING))
    assert abs(dx) < 1e-6, dx
    assert abs(dy - config.LAIN_LEBAR / 2) < 1e-6, dy


def test_serong_mencampur_keduanya():
    dx, dy = perception.koreksi_muka(_kotak((config.AR_BELAKANG + config.AR_SAMPING) / 2))
    assert abs(dx - config.LAIN_PANJANG / 4) < 1e-6, dx
    assert abs(dy - config.LAIN_LEBAR / 4) < 1e-6, dy


def test_di_luar_rentang_dijepit():
    """Kotak lebih persegi atau lebih panjang dari yang mungkin tidak boleh
    memberi geseran negatif atau melebihi setengah dimensi."""
    for rasio in (0.2, 0.9, 4.0, 12.0):
        dx, dy = perception.koreksi_muka(_kotak(rasio))
        assert 0.0 <= dx <= config.LAIN_PANJANG / 2 + 1e-9, (rasio, dx)
        assert 0.0 <= dy <= config.LAIN_LEBAR / 2 + 1e-9, (rasio, dy)


def test_kotak_setinggi_nol_tidak_meledak():
    perception.koreksi_muka([10.0, 50.0, 30.0, 50.0])


if __name__ == '__main__':
    for nama, fn in sorted(globals().items()):
        if nama.startswith('test_'):
            fn()
            print(f'ok  {nama}')
    print('semua lolos')
