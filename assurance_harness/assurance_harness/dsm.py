import numpy as np

def ground_height(x: float, y: float) -> float:
    # Simple synthetic DSM: flat baseline + smooth ridge (meters)
    return 2.0 + 1.5 * np.exp(-0.01 * (x**2 + y**2))

def line_of_sight_clear(drone_pos, gcs_pos, steps: int = 50) -> bool:
    # Conservative geometric LOS check along the segment from GCS -> drone.
    # If the straight line ever goes at/below ground, LOS is blocked.
    x0, y0, z0 = gcs_pos
    x1, y1, z1 = drone_pos
    for i in range(1, steps + 1):
        t = i / steps  # 0..1
        x = x0 + t * (x1 - x0)
        y = y0 + t * (y1 - y0)
        z = z0 + t * (z1 - z0)
        if z <= ground_height(x, y):
            return False
    return True
