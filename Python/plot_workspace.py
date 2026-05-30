import matplotlib.pyplot as plt

from arm_model import solve_ik_by_alpha_search


def main() -> None:
    reachable_x = []
    reachable_z = []
    unreachable_x = []
    unreachable_z = []

    for x in range(-300, 301, 10):
        for z in range(50, 501, 10):
            try:
                solve_ik_by_alpha_search(x, z)
                reachable_x.append(x)
                reachable_z.append(z)
            except ValueError:
                unreachable_x.append(x)
                unreachable_z.append(z)

    plt.figure()

    plt.scatter(unreachable_x, unreachable_z, s=6, alpha=0.15, label="unreachable")
    plt.scatter(reachable_x, reachable_z, s=8, alpha=0.8, label="reachable")

    plt.axhline(0)
    plt.axvline(0)

    plt.gca().set_aspect("equal", adjustable="box")
    plt.xlabel("x / mm")
    plt.ylabel("z / mm")
    plt.title("Arm Reachable Workspace")
    plt.grid(True)
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
