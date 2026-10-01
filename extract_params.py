"""Tahap 1 bagian 3.1 -- ekstraksi parameter fisik ego dari CARLA.

Wheelbase tidak diekspos CARLA, jadi dihitung dari posisi roda. Offset sumbu
belakang terhadap origin actor ikut diukur supaya konversi titik referensi (3.2)
tidak perlu mengasumsikan L/2.

    python extract_params.py  ->  out/vehicle_params.json
"""
import json
import math

import numpy as np

import config
import simulation


def local_offsets(ego, wheels):
    """Posisi roda (cm, frame dunia) -> offset terhadap origin actor (m, frame kendaraan)."""
    M = np.array(ego.get_transform().get_inverse_matrix())
    return [(M @ [w.position.x / 100.0, w.position.y / 100.0,
                  w.position.z / 100.0, 1.0])[:3] for w in wheels]


def extract(ego):
    pc = ego.get_physics_control()
    fl, fr, rl, rr = local_offsets(ego, pc.wheels)      # 0=FL 1=FR 2=RL 3=RR
    front, rear = (fl + fr) / 2, (rl + rr) / 2
    delta_max_phys = math.radians(pc.wheels[0].max_steer_angle)
    bbox = ego.bounding_box

    return {
        'blueprint': config.EGO_BP,

        # Prediction model
        'L': float(np.linalg.norm(front - rear)),
        'rear_axle_offset_x': float(rear[0]),           # <0: di belakang origin actor
        'rear_axle_offset_y': float(rear[1]),           # harus ~0

        # Constraint kemudi
        'delta_max_phys': delta_max_phys,
        'delta_max': min(0.5, delta_max_phys),          # batas konservatif, bagian 3.1
        # Satuan sumbu-x (km/jam vs m/s) tidak didokumentasikan CARLA; ukur empiris.
        'steering_curve': [(p.x, p.y) for p in pc.steering_curve],

        # Elips penghindaran tabrakan
        'length': 2 * bbox.extent.x,
        'width': 2 * bbox.extent.y,

        # Kalibrasi throttle map
        'mass': pc.mass,
        'drag_coefficient': pc.drag_coefficient,
    }


def report(p):
    print(f"\n{'Parameter':<24}{'Value':>12}  Unit")
    print('-' * 54)
    rows = [
        ('Wheelbase (L)', p['L'], 'm'),
        ('Rear axle offset', p['rear_axle_offset_x'], 'm (vehicle frame)'),
        ('delta_max physical', p['delta_max_phys'], f"rad ({math.degrees(p['delta_max_phys']):.1f} deg)"),
        ('delta_max used', p['delta_max'], 'rad'),
        ('Length', p['length'], 'm'),
        ('Width', p['width'], 'm'),
        ('Mass', p['mass'], 'kg'),
        ('Drag coefficient', p['drag_coefficient'], '-'),
    ]
    for name, val, unit in rows:
        print(f'{name:<24}{val:>12.4f}  {unit}')
    print(f"\nsteering_curve: {p['steering_curve']}")

    if abs(p['rear_axle_offset_y']) > 0.02:
        print(f"\nWARNING: rear axle is shifted laterally "
              f"{abs(p['rear_axle_offset_y']):.3f} m from the actor origin.")

    print('\nReference point conversion (3.2), used in localization.py:')
    print(f"    x_rear = tf.location.x + ({p['rear_axle_offset_x']:.4f}) * cos(yaw)")
    print(f"    y_rear = tf.location.y + ({p['rear_axle_offset_x']:.4f}) * sin(yaw)")
    print(f"  (not L/2 = {p['L'] / 2:.4f} m -- the difference is "
          f"{abs(abs(p['rear_axle_offset_x']) - p['L'] / 2):.4f} m)")


def main():
    with simulation.carla_world() as world:
        with simulation.ego_vehicle(world) as ego:
            params = extract(ego)

    with open(config.VEHICLE_PARAMS_JSON, 'w') as f:
        json.dump(params, f, indent=2)

    report(params)
    print(f'\nSaved: {config.VEHICLE_PARAMS_JSON}')


if __name__ == '__main__':
    main()
