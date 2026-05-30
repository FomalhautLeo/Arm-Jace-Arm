import matplotlib.pyplot as plt

from arm_model import joint_points_2d, solve_ik_by_alpha_search


def draw_workspace() -> None:
    reachable_x = []
    reachable_z = []

    for x in range(-300, 301, 10):
        for z in range(50, 501, 10):
            try:
                solve_ik_by_alpha_search(x, z)
                reachable_x.append(x)
                reachable_z.append(z)
            except ValueError:
                continue

    plt.scatter(reachable_x, reachable_z, s=5, alpha=0.15, label="workspace")


def plot_target_pose(x: float, z: float) -> None:
    theta = solve_ik_by_alpha_search(x, z)
    points = joint_points_2d(*theta)

    xs = [p[0] for p in points]
    zs = [p[1] for p in points]

    plt.plot(xs, zs, marker="o", linewidth=2, label=f"pose ({x}, {z})")
    plt.scatter([x], [z], marker="x", s=100)

    alpha = sum(theta)
    print("-" * 40)
    print(f"target: x={x:.2f}, z={z:.2f}")
    print("theta:", tuple(round(t, 2) for t in theta))
    print(f"alpha: {alpha:.2f}")


def main() -> None:
    targets = [
        (50, 420),
        (100, 400),
        (150, 350),
        (-100, 400),
        (-150, 350),
    ]

    plt.figure()

    draw_workspace()

    for x, z in targets:
        try:
            plot_target_pose(x, z)
        except ValueError as e:
            print("-" * 40)
            print(f"target: x={x:.2f}, z={z:.2f}")
            print(f"unreachable: {e}")

    plt.axhline(0)
    plt.axvline(0)

    plt.gca().set_aspect("equal", adjustable="box")
    plt.xlabel("x / mm")
    plt.ylabel("z / mm")
    plt.title("IK Selected Poses with Workspace")
    plt.grid(True)
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
