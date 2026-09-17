"""Uji frame ego -> frame jalan. Tanpa simulator.

Mengunci titik acuan halangan, yang sudah tiga kali jadi sumber bias 1,433 m
(bias XTE Tahap 1, kotak penilai jarak, dan konversi ini sendiri).
"""
import math
import os
import sys

import numpy as np

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, AKAR)

import config
import localization


def _ego(x=100.0, y=0.0, yaw=0.0, v=13.4):
    return localization.EgoState(x, y, yaw, v, 0.0, 0.0)


def test_halangan_dijangkar_di_pusat_bodi_ego():
    """Halangan 40 m di depan pusat bodi -> pusat bodi ego + 40 di frame jalan."""
    ego = _ego()
    out = localization.halangan_ego_ke_jalan([[40.0, 0.0, -6.4, 0.0]], ego)
    assert abs(out[0, 0] - (ego.x + config.SUMBU_KE_PUSAT + 40.0)) < 1e-9, out


def test_selisih_ke_pusat_bodi_ego_kembali_utuh():
    """Yang dipakai zona planner/MPC: pusat bodi ego dikurangkan lagi, harus
    memberi kembali jarak relatif yang dilihat perception -- tanpa sisa."""
    for yaw in (0.0, 0.2, -0.35):
        ego = _ego(yaw=yaw)
        out = localization.halangan_ego_ke_jalan([[40.0, -3.5, 0.0, 0.0]], ego)
        xc = ego.x + config.SUMBU_KE_PUSAT * math.cos(yaw)
        yc = ego.y + config.SUMBU_KE_PUSAT * math.sin(yaw)
        dx, dy = out[0, 0] - xc, out[0, 1] - yc
        assert abs(math.hypot(dx, dy) - math.hypot(40.0, 3.5)) < 1e-9, (yaw, dx, dy)


def test_celah_fsm_diukur_dari_sumbu_belakang():
    """main.py mengurangkan ego.x (sumbu belakang) untuk FSM; `_v_ikut`
    mendokumentasikan celahnya diukur dari sumbu belakang. Harus cocok."""
    ego = _ego()
    out = localization.halangan_ego_ke_jalan([[40.0, 0.0, 0.0, 0.0]], ego)
    assert abs((out[0, 0] - ego.x) - (40.0 + config.SUMBU_KE_PUSAT)) < 1e-9


def test_kosong_tetap_berbentuk():
    assert localization.halangan_ego_ke_jalan([], _ego()).shape == (0, 4)


if __name__ == '__main__':
    for nama, fn in sorted(globals().items()):
        if nama.startswith('test_'):
            fn()
            print(f'ok  {nama}')
    print('semua lolos')
