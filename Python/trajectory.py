from arm_model import solve_ik_by_alpha_search, forward_kinematics_2d

import math


def interpolate_line(
    start: tuple[float, float],
    end: tuple[float, float],
    steps: int,
) -> list[tuple[float, float]]:
    """
    线性插值
    """
    if steps <= 0:
        raise ValueError("steps must be positive")

    x0, z0 = start
    x1, z1 = end

    x_step = (x1 - x0) / steps
    z_step = (z1 - z0) / steps

    points = []

    for i in range(steps + 1):
        x = x0 + i * x_step
        z = z0 + i * z_step
        points.append((x, z))

    return points


def generate_line_trajectory(
    start: tuple[float, float],
    end: tuple[float, float],
    steps: int = 10,
    allow_unreachable: bool = False,
) -> list[tuple[float, float, tuple[float, float, float]]]:
    """
    通过插值生成姿态序列
    """
    trajectory = []
    points = interpolate_line(start, end, steps)
    prev_theta: tuple[float, float, float] | None = None
    for x, z in points:
        # 逐个求解
        try:
            theta = solve_ik_by_alpha_search(x=x, z=z, prev_theta=prev_theta)
        except ValueError:
            if not allow_unreachable:
                raise ValueError(f"trajectory point ({x:.2f}, {z:.2f}) is unreachable")

            theta = (math.nan, math.nan, math.nan)

        # 插入解
        trajectory.append((x, z, theta))
        # 记录当前角度
        if not any(math.isnan(v) for v in theta):
            prev_theta = theta
    return trajectory


def interpolate_joint(
    start_theta: tuple[float, float, float],
    end_theta: tuple[float, float, float],
    steps: int,
) -> list[tuple[float, float, float]]:
    """
    关节空间插值
    """
    if steps <= 0:
        raise ValueError("steps must be positive")

    t1_0, t2_0, t3_0 = start_theta
    t1_1, t2_1, t3_1 = end_theta

    step1 = (t1_1 - t1_0) / steps
    step2 = (t2_1 - t2_0) / steps
    step3 = (t3_1 - t3_0) / steps

    result = []

    for i in range(steps + 1):
        theta1 = t1_0 + i * step1
        theta2 = t2_0 + i * step2
        theta3 = t3_0 + i * step3

        result.append((theta1, theta2, theta3))

    return result


def generate_joint_trajectory(
    start: tuple[float, float],
    end: tuple[float, float],
    steps: int = 10,
) -> list[tuple[float, float, tuple[float, float, float]]]:
    """
    通过关节空间插值生成轨迹。

    start/end 是目标末端点。
    先分别求出起点和终点 IK，
    再在关节角度空间中线性插值。

    返回：
        (x, z, theta)

    注意：
        x/z 是插值后的 theta 通过正运动学算出来的实际末端位置，
        不保证严格落在 start-end 直线上。
    """
    start_theta = solve_ik_by_alpha_search(x=start[0], z=start[1])
    end_theta = solve_ik_by_alpha_search(x=end[0], z=end[1], prev_theta=start_theta)

    theta_list = interpolate_joint(start_theta, end_theta, steps)

    trajectory = []

    for theta in theta_list:
        x, z = forward_kinematics_2d(*theta)
        trajectory.append((x, z, theta))

    return trajectory


def generate_joint_trajectory_from_theta(
    start_theta: tuple[float, float, float],
    end: tuple[float, float],
    steps: int = 10,
) -> list[tuple[float, float, tuple[float, float, float]]]:
    """
    已知起点关节角，生成到目标末端点的关节空间轨迹。
    用于动作序列中，避免段与段之间重新求起点 IK 导致姿态跳变。
    """
    end_theta = solve_ik_by_alpha_search(
        x=end[0],
        z=end[1],
        prev_theta=start_theta,
    )

    theta_list = interpolate_joint(start_theta, end_theta, steps)

    trajectory = []

    for theta in theta_list:
        x, z = forward_kinematics_2d(*theta)
        trajectory.append((x, z, theta))

    return trajectory
