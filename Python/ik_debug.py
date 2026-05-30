from arm_model import inverse_kinematics_2d, is_valid_theta


def debug_ik_candidates(x: float, z: float) -> None:
    print(f"target: ({x}, {z})")

    for alpha in range(-80, 81, 5):
        for elbow in (1, -1):
            try:
                theta = inverse_kinematics_2d(x, z, alpha, elbow)
            except ValueError:
                continue

            theta1, theta2, theta3 = theta

            if not is_valid_theta(theta1, theta2, theta3):
                continue

            print(
                f"alpha={alpha:>4}, elbow={elbow:>2}, "
                f"theta=({theta1:7.2f}, {theta2:7.2f}, {theta3:7.2f})"
            )

debug_ik_candidates(100, 330)
debug_ik_candidates(110, 320)
debug_ik_candidates(120, 300)
