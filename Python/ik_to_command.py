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


def main() -> None:
    targets = [
        # 高处安全点：先测这些
        (50, 420),
        (100, 400),
        (-50, 420),
        (-100, 400),
        # 稍低一点的验证点：安全点没问题后再测
        (80, 360),
        (120, 360),
        (-80, 360),
        (-120, 360),
        # 更靠前/更低的边界观察点：最后再测
        (150, 350),
        (-150, 350),
    ]

    for x, z in targets:
        print("-" * 40)
        print(f"target: x={x:.2f}, z={z:.2f}")

        try:
            theta = solve_ik_by_alpha_search(x, z)
        except ValueError as e:
            print(f"unreachable: {e}")
            continue

        theta1, theta2, theta3 = theta
        alpha = theta1 + theta2 + theta3
        x2, z2 = forward_kinematics_2d(theta1, theta2, theta3)
        cmd = make_q_command(theta, time_ms=1000)

        print("theta:", tuple(round(t, 2) for t in theta))
        print(f"alpha: {alpha:.2f}")
        print(f"cost: {cost(theta1, theta2, theta3):.2f}")
        print(f"fk: x={x2:.2f}, z={z2:.2f}")
        print_warnings(theta)
        print("cmd:", cmd)


if __name__ == "__main__":
    main()
