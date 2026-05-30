import matplotlib.pyplot as plt

from arm_model import joint_points_2d


def plot_pose(theta: tuple[float, float, float]) -> None:
    points = joint_points_2d(*theta)

    xs = [p[0] for p in points]
    zs = [p[1] for p in points]

    plt.plot(xs, zs, marker="o", label=str(theta))


def main() -> None:
    poses = [
        (0, 0, 0),
        (10, 0, 0),
        (10, 10, 0),
        (10, 10, 10),
        (-10, 0, 0),
        (-30, -30, -30),
        (45, 20, 20),
    ]

    plt.figure()

    for pose in poses:
        plot_pose(pose)

    plt.axhline(0)
    plt.axvline(0)

    plt.gca().set_aspect("equal", adjustable="box")
    plt.xlabel("x / mm")
    plt.ylabel("z / mm")
    plt.title("Arm Forward Kinematics 2D")
    plt.grid(True)
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
