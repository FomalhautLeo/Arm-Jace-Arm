from arm_model import forward_kinematics_2d, inverse_kinematics_2d


def main() -> None:
    test_cases = [
        (0, 0, 0),
        (10, 0, 0),
        (10, 10, 0),
        (10, 10, 10),
        (20, 20, 20),
        (-10, 20, 0),
    ]

    for theta in test_cases:
        theta1, theta2, theta3 = theta

        x, z = forward_kinematics_2d(theta1, theta2, theta3)

        alpha = theta1 + theta2 + theta3

        ik_theta = inverse_kinematics_2d(x, z, alpha)

        x2, z2 = forward_kinematics_2d(*ik_theta)

        print("original theta:", theta)
        print(f"target: x={x:.2f}, z={z:.2f}, alpha={alpha:.2f}")
        print("ik theta:", tuple(round(v, 2) for v in ik_theta))
        print(f"fk again: x={x2:.2f}, z={z2:.2f}")
        print("-" * 40)


if __name__ == "__main__":
    main()
