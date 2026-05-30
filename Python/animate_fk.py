import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from arm_model import joint_points_2d


def main() -> None:
    fig, ax = plt.subplots()

    (line,) = ax.plot([], [], marker="o")
    text = ax.text(0.02, 0.95, "", transform=ax.transAxes)

    ax.set_xlim(-300, 400)
    ax.set_ylim(0, 500)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.set_xlabel("x / mm")
    ax.set_ylabel("z / mm")
    ax.set_title("Arm FK Animation")

    frames = list(range(-45, 46))

    def update(frame: int):
        theta1 = frame
        theta2 = frame
        theta3 = frame

        points = joint_points_2d(theta1, theta2, theta3)
        xs = [p[0] for p in points]
        zs = [p[1] for p in points]

        line.set_data(xs, zs)
        text.set_text(f"theta=({theta1}, {theta2}, {theta3})")

        return line, text

    ani = FuncAnimation(fig, update, frames=frames, interval=80, blit=True)  # noqa: F841
    plt.show()


if __name__ == "__main__":
    main()
