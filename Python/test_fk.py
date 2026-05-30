from arm_model import forward_kinematics_2d


def main() -> None:
    test_cases = [
        (0, 0, 0),
        (10, 0, 0),
        (10, 10, 0),
        (10, 10, 10),
        (-10, 0, 0),
        (-30, -30, -30),
        (45, 20, 20),
    ]

    for theta in test_cases:
        x, z = forward_kinematics_2d(*theta)
        print(f"{theta} -> x={x:.2f} mm, z={z:.2f} mm")


if __name__ == "__main__":
    main()
