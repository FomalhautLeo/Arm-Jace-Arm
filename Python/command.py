from arm_model import (
    forward_kinematics_2d,
    solve_ik_by_alpha_search,
    cost,
)


THETA_LIMITS = {
    "theta1": (-100, 100),
    "theta2": (-80, 80),
    "theta3": (-80, 80),
    "alpha": (-60, 60),
}

WARN_MARGIN = 8.0


def make_q_command(
    theta: tuple[float, float, float],
    time_ms: int = 1000,
    joint0: float = 0,
    joint4: float = 0,
    joint5: float = 0,
) -> str:
    theta1, theta2, theta3 = theta

    return (
        f"$Q:"
        f"{round(joint0)},"
        f"{round(theta1)},"
        f"{round(theta2)},"
        f"{round(theta3)},"
        f"{round(joint4)},"
        f"{round(joint5)},"
        f"{time_ms}!"
    )


def check_near_limit(name: str, value: float) -> str | None:
    low, high = THETA_LIMITS[name]

    if value - low < WARN_MARGIN:
        return f"[WARN] {name} near lower limit: {value:.2f}"

    if high - value < WARN_MARGIN:
        return f"[WARN] {name} near upper limit: {value:.2f}"

    return None


def print_warnings(theta: tuple[float, float, float]) -> None:
    theta1, theta2, theta3 = theta
    alpha = theta1 + theta2 + theta3

    values = {
        "theta1": theta1,
        "theta2": theta2,
        "theta3": theta3,
        "alpha": alpha,
    }

    for name, value in values.items():
        warning = check_near_limit(name, value)
        if warning is not None:
            print(warning)
