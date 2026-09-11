"""Menegakkan aturan bagian 2.4: planning.py dan control.py murni numerik.

Aturan itu bukan soal kerapian. Dia yang membuat MPC, FSM, dan polinomial bisa
diuji di terminal dalam hitungan detik tanpa menyalakan simulator -- dan
sekaligus mencegah sensor tabrakan (instrumen pengukuran) bocor ke jalur kendali.
"""
import ast
import os
import sys

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, AKAR)

MURNI = ('planning.py', 'control.py')
TERLARANG = {'carla', 'simulation', 'perception', 'evaluation', 'localization', 'main'}


def _impor(berkas):
    pohon = ast.parse(open(os.path.join(AKAR, berkas)).read())
    nama = set()
    for n in ast.walk(pohon):
        if isinstance(n, ast.Import):
            nama |= {a.name.split('.')[0] for a in n.names}
        elif isinstance(n, ast.ImportFrom) and n.module:
            nama.add(n.module.split('.')[0])
    return nama


def test_modul_numerik_tidak_menyentuh_carla():
    for berkas in MURNI:
        haram = _impor(berkas) & TERLARANG
        assert not haram, f'{berkas} mengimpor {haram} -- melanggar aturan 2.4'


def test_modul_numerik_bisa_diimpor_tanpa_server():
    """Kalau ada carla yang menyelinap, impor ini masih lolos -- server tidak
    dibutuhkan untuk mengimpor. Uji di atas yang menangkapnya; ini memastikan
    tidak ada efek samping saat impor."""
    import control, planning                                  # noqa: F401
    assert hasattr(planning, 'BehaviorFSM') and hasattr(control, 'MPCController')


def test_sensor_tabrakan_hanya_di_evaluation():
    """Sensor tabrakan adalah instrumen pengukuran, bukan masukan kendali."""
    for berkas in ('planning.py', 'control.py', 'perception.py'):
        isi = open(os.path.join(AKAR, berkas)).read()
        assert 'collision' not in isi.lower(), f'{berkas} menyentuh sensor tabrakan'
    assert 'sensor.other.collision' in open(os.path.join(AKAR, 'evaluation.py')).read()


if __name__ == '__main__':
    for nama, fn in sorted(globals().items()):
        if nama.startswith('test_'):
            fn()
            print(f'ok  {nama}')
    print('semua lolos')
