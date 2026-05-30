from arm_model import forward_kinematics_2d, solve_ik_by_alpha_search

targets = [
    (100, 400),
    (150, 350),
    (200, 300),
    (50, 420),
]

for x, z in targets:
    print("-" * 40)
    print(f"target: x={x:.2f}, z={z:.2f}")
    try:
        theta = solve_ik_by_alpha_search(x, z)
    except ValueError as e:
        print(e)
        continue
    alpha = sum(theta)
    x2, z2 = forward_kinematics_2d(*theta)

    print("theta:", tuple(round(t, 2) for t in theta))
    print("best alpha:", alpha)
    print(f"fk: x={x2:.2f}, z={z2:.2f}")
