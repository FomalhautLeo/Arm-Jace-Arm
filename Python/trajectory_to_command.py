from trajectory import generate_line_trajectory, generate_joint_trajectory


def make_q_command(
    theta: tuple[float, float, float],
    time_ms: int = 300,
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


def main(mode: str = "joint") -> None:
    # mode = "joint"：关节空间插值，动作更平滑，默认使用
    # mode = "cartesian"：末端空间直线插值，路径更直，但可能出现 IK 解波动
    start = (50, 420)
    end = (100, 400)

    if mode == "joint":
        trajectory = generate_joint_trajectory(start=start, end=end, steps=5)
    elif mode == "cartesian":
        trajectory = generate_line_trajectory(
            start=start, end=end, steps=5, allow_unreachable=False
        )
    else:
        raise ValueError(f"Unknown mode: {mode}")

    for i, (x, z, theta) in enumerate(trajectory):
        alpha = sum(theta)
        cmd = make_q_command(theta, time_ms=800)

        print("-" * 40)
        print(f"step {i}")
        print(f"target: x={x:.2f}, z={z:.2f}")
        print("theta:", tuple(round(v, 2) for v in theta))
        print(f"alpha: {alpha:.2f}")
        print("cmd:", cmd)


if __name__ == "__main__":
    main()
