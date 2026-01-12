def clamp(v: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(v, vmax))


def clamp_xyz(
    target: list[float],
    xlim: tuple[float, float],
    ylim: tuple[float, float],
    zlim: tuple[float, float],
) -> list[float]:
    x, y, z = target
    return [
        clamp(x, xlim[0], xlim[1]),
        clamp(y, ylim[0], ylim[1]),
        clamp(z, zlim[0], zlim[1]),
    ]


def safe_heights(table_z: float) -> tuple[float, float]:
    safe_z = table_z + 0.12
    pick_z = table_z + 0.03
    return safe_z, pick_z


def workspace_limits(table_z: float) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
    xlim = (0.30, 0.75)
    ylim = (-0.30, 0.30)
    zlim = (table_z + 0.02, table_z + 0.45)
    return xlim, ylim, zlim
