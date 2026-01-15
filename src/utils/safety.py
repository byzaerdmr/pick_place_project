def clamp(v: float, vmin: float, vmax: float) -> float:
    # restrict a value within a specified range
    return max(vmin, min(v, vmax))


def clamp_xyz(
    target: list[float],
    xlim: tuple[float, float],
    ylim: tuple[float, float],
    zlim: tuple[float, float],
) -> list[float]:
    # restrict 3d coordinates to workspace boundaries
    x, y, z = target
    return [
        clamp(x, xlim[0], xlim[1]),
        clamp(y, ylim[0], ylim[1]),
        clamp(z, zlim[0], zlim[1]),
    ]


def safe_heights(table_z: float) -> tuple[float, float]:
    # define vertical clearance heights based on table level
    safe_z = table_z + 0.15 # height for safe horizontal movement
    pick_z = table_z + 0.04 # height just above the table for placement
    return safe_z, pick_z


def workspace_limits(table_z: float) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
    # define 3d bounding box for safe robot operation
    xlim = (0.30, 0.75) # longitudinal range
    ylim = (-0.35, 0.35) # lateral range
    # minimum z is slightly above table to avoid hard collisions
    zlim = (table_z + 0.02, table_z + 0.50)
    return xlim, ylim, zlim