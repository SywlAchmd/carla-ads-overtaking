"""Menegakkan aturan bagian 2.4: planning.py dan control.py murni numerik.

Aturan itu bukan soal kerapian. Dia yang membuat MPC, FSM, dan polinomial bisa
diuji di terminal dalam hitungan detik tanpa menyalakan simulator -- dan
sekaligus mencegah sensor tabrakan (instrumen pengukuran) bocor ke jalur kendali.
"""
import ast
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

# tracking.py ikut murni: Kalman filter dan asosiasi harus bisa diuji tanpa
# simulator, torch, maupun kamera -- lihat tests/test_tracking.py.
PURE = ('planning.py', 'control.py', 'tracking.py')
FORBIDDEN = {'carla', 'simulation', 'perception', 'evaluation', 'localization', 'main'}


def _imports(filename):
    tree = ast.parse(open(os.path.join(ROOT, filename)).read())
    name = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Import):
            name |= {a.name.split('.')[0] for a in n.names}
        elif isinstance(n, ast.ImportFrom) and n.module:
            name.add(n.module.split('.')[0])
    return name


def test_numeric_modules_do_not_touch_carla():
    for filename in PURE:
        forbidden = _imports(filename) & FORBIDDEN
        assert not forbidden, f'{filename} imports {forbidden} -- violates rule 2.4'


def test_numeric_modules_import_without_server():
    """Kalau ada carla yang menyelinap, impor ini masih lolos -- server tidak
    dibutuhkan untuk mengimpor. Uji di atas yang menangkapnya; ini memastikan
    tidak ada efek samping saat impor."""
    import control, planning                                  # noqa: F401
    assert hasattr(planning, 'BehaviorFSM') and hasattr(control, 'MPCController')


def test_collision_sensor_only_in_evaluation():
    """Sensor tabrakan adalah instrumen pengukuran, bukan masukan kendali."""
    for filename in ('planning.py', 'control.py', 'perception.py'):
        body = open(os.path.join(ROOT, filename)).read()
        assert 'collision' not in body.lower(), f'{filename} touches the collision sensor'
    assert 'sensor.other.collision' in open(os.path.join(ROOT, 'evaluation.py')).read()


def test_actor_commands_go_through_simulation_tick():
    """Perintah aktor asinkron balapan dengan world.tick() dan merusak determinisme
    (docstring simulation.tick). Wajib dikirim lewat simulation.tick."""
    for filename in os.listdir(ROOT):
        if filename.endswith('.py') and filename != 'simulation.py':
            body = open(os.path.join(ROOT, filename)).read()
            for pattern in ('.apply_control(', '.set_target_velocity(', '.set_transform('):
                assert pattern not in body, f'{filename} calls {pattern} directly -- use simulation.tick'


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            fn()
            print(f'ok  {name}')
    print('all passed')
