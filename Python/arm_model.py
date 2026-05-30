import math
from dataclasses import dataclass


@dataclass
class ArmLinks:
    """
    机械臂连杆参数，单位：mm。

    l0: 底座高度，即地面/底座平面到 joint1 转轴中心的高度
    l1: 大臂长度，joint1 -> joint2
    l2: 小臂长度，joint2 -> joint3
    l3: 腕部/夹爪末端长度，joint3 -> 夹爪末端中心
    """

    l0: float = 100
    l1: float = 105
    l2: float = 75
    l3: float = 180


def deg2rad(deg: float) -> float:
    """
    角度制转弧度制。
    """
    return deg * math.pi / 180.0


def rad2deg(rad: float) -> float:
    """
    弧度制转角度制。
    """
    return rad * 180.0 / math.pi


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


def forward_kinematics_2d(
    theta1: float,
    theta2: float,
    theta3: float,
    links: ArmLinks = ArmLinks(),
) -> tuple[float, float]:
    """
    计算机械臂在侧视平面中的末端坐标。

    坐标系定义：
    - x 正方向：机械臂向前伸出的方向
    - z 正方向：竖直向上
    - 原点：底座中心在地面/底座平面上的投影点

    角度定义：
    - theta1: 大臂相对 home 的角度
    - theta2: 小臂相对大臂的角度
    - theta3: 腕部相对小臂的角度

    这里输入的 theta 是“软件关节角度”：
    - theta = 0 时，对应舵机 home 位置
    - 正角度表示向 x 正方向前伸

    公式中的 phi 是每段连杆相对 x 正方向的绝对角度。
    因为 home 姿态下连杆近似竖直向上，所以：

        phi = pi/2 - theta累计角

    返回：
    - x: 末端在前后方向上的位置，单位 mm
    - z: 末端高度，单位 mm
    """

    phi1 = math.pi / 2 - deg2rad(theta1)
    phi2 = math.pi / 2 - deg2rad(theta1 + theta2)
    phi3 = math.pi / 2 - deg2rad(theta1 + theta2 + theta3)

    x = (
        links.l1 * math.cos(phi1)
        + links.l2 * math.cos(phi2)
        + links.l3 * math.cos(phi3)
    )

    z = (
        links.l0
        + links.l1 * math.sin(phi1)
        + links.l2 * math.sin(phi2)
        + links.l3 * math.sin(phi3)
    )

    return x, z


def joint_points_2d(
    theta1: float,
    theta2: float,
    theta3: float,
    links: ArmLinks = ArmLinks(),
) -> list[tuple[float, float]]:
    """
    计算侧视平面中每个关节点的位置。

    返回值依次为：
    - p0: joint1 转轴位置，也就是大臂起点
    - p1: joint2 转轴位置，也就是大臂末端 / 小臂起点
    - p2: joint3 转轴位置，也就是小臂末端 / 腕部起点
    - p3: 夹爪末端中心位置

    这个函数主要用于画图和动画。
    forward_kinematics_2d 只返回末端点；
    joint_points_2d 返回完整骨架点，方便观察机械臂姿态是否符合直觉。
    """

    phi1 = math.pi / 2 - deg2rad(theta1)
    phi2 = math.pi / 2 - deg2rad(theta1 + theta2)
    phi3 = math.pi / 2 - deg2rad(theta1 + theta2 + theta3)

    # joint1 位置：底座高度处
    p0 = (0.0, links.l0)

    # joint2 位置：从 joint1 沿大臂方向走 l1
    p1 = (
        p0[0] + links.l1 * math.cos(phi1),
        p0[1] + links.l1 * math.sin(phi1),
    )

    # joint3 位置：从 joint2 沿小臂方向走 l2
    p2 = (
        p1[0] + links.l2 * math.cos(phi2),
        p1[1] + links.l2 * math.sin(phi2),
    )

    # 末端位置：从 joint3 沿腕部/夹爪方向走 l3
    p3 = (
        p2[0] + links.l3 * math.cos(phi3),
        p2[1] + links.l3 * math.sin(phi3),
    )

    return [p0, p1, p2, p3]


def inverse_kinematics_2d(
    x: float, z: float, alpha: float, elbow: int = 1, links: ArmLinks = ArmLinks()
) -> tuple[float, float, float]:
    """
    给定 alpha 的逆运动学计算
    """
    # alpha 为末端连杆（腕部）与竖直方向的夹角
    # alpha = theta1 + theta2 + theta3
    #
    # L2 与 L3 连接处（joint3）的坐标:
    # j3_x = x - L3 * sin(alpha)
    # j3_z = z - L3 * cos(alpha)
    #
    # 又有:
    # j3_x      = L1 * sin(theta1) + L2 * sin(theta1 + theta2)
    # j3_z - L0 = L1 * cos(theta1) + L2 * cos(theta1 + theta2)
    #
    # 令 u = j3_x, v = j3_z - L0
    # 则 J1 到 J3 的距离为:
    # r = sqrt(u^2 + v^2)
    #
    # 由余弦定理:
    # cos(theta2) = (r^2 - L1^2 - L2^2) / (2 * L1 * L2)
    # theta2 = +-arccos((r^2 - L1^2 - L2^2) / (2 * L1 * L2))
    #
    # 令 L1 与 r 的夹角为 beta:
    # cos(beta) = (r^2 + L1^2 - L2^2) / (2 * r * L1)
    # beta = arccos((r^2 + L1^2 - L2^2) / (2 * r * L1))
    #
    # 令 r 与竖直方向的夹角为 gamma:
    # gamma = atan2(u, v)
    #
    # 当前有两个肘部解：
    # elbow =  1: theta2 = +acos(...), theta1 = gamma - beta
    # elbow = -1: theta2 = -acos(...), theta1 = gamma + beta
    #
    # 最后：
    # theta3 = alpha - theta1 - theta2
    if elbow not in (1, -1):
        raise ValueError("elbow must be 1 or -1")

    alpha_rad = deg2rad(alpha)

    l1 = links.l1
    l2 = links.l2
    l3 = links.l3

    j3_x = x - l3 * math.sin(alpha_rad)
    j3_z = z - l3 * math.cos(alpha_rad)
    u = j3_x
    v = j3_z - links.l0

    r = math.sqrt(u**2 + v**2)
    # 可达性检查
    if r > l1 + l2:
        raise ValueError("target too far")
    if r < abs(l1 - l2):
        raise ValueError("target too close")
    if r == 0:
        raise ValueError("target at joint origin")

    cos_theta2 = (r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_theta2 = clamp(cos_theta2, -1.0, 1.0)

    cos_beta = (r**2 + l1**2 - l2**2) / (2 * r * l1)
    cos_beta = clamp(cos_beta, -1.0, 1.0)
    beta = math.acos(cos_beta)

    gamma = math.atan2(u, v)

    if elbow == 1:
        theta2 = math.acos(cos_theta2)
        theta1 = gamma - beta
    else:
        theta2 = -math.acos(cos_theta2)
        theta1 = gamma + beta
    theta3 = alpha_rad - theta1 - theta2

    return rad2deg(theta1), rad2deg(theta2), rad2deg(theta3)


# def cost(
#     theta1: float,
#     theta2: float,
#     theta3: float,
#     prev_theta: tuple[float, float, float] | None = None,
# ) -> float:
#     base = theta1**2 + theta2**2 + theta3**2

#     CONTINUITY_WEIGHT = 20.0

#     # 不鼓励大臂后仰
#     if theta1 < 0:
#         base += 9.0 * theta1**2

#     # 轨迹连续性：不鼓励和上一帧差太多
#     if prev_theta is not None:
#         p1, p2, p3 = prev_theta
#         base += CONTINUITY_WEIGHT * (
#             (theta1 - p1) ** 2 + (theta2 - p2) ** 2 + (theta3 - p3) ** 2
#         )


#     return base
def cost(
    theta1: float,
    theta2: float,
    theta3: float,
    prev_theta: tuple[float, float, float] | None = None,
) -> float:
    alpha = theta1 + theta2 + theta3

    alpha_target = 20.0

    base = theta1**2 + theta2**2 + theta3**2 + 5.0 * (alpha - alpha_target) ** 2

    if prev_theta is not None:
        p1, p2, p3 = prev_theta
        base += 10.0 * ((theta1 - p1) ** 2 + (theta2 - p2) ** 2 + (theta3 - p3) ** 2)

    return base


def is_valid_theta(theta1: float, theta2: float, theta3: float) -> bool:
    alpha = theta1 + theta2 + theta3
    return (
        -100 <= theta1 <= 100
        and -80 <= theta2 <= 80
        and -80 <= theta3 <= 80
        and -60 <= alpha <= 60
    )


def is_valid_transition(
    theta: tuple[float, float, float],
    prev_theta: tuple[float, float, float] | None,
    max_delta: float = 35.0,
) -> bool:
    if prev_theta is None:
        return True

    return all(abs(a - b) <= max_delta for a, b in zip(theta, prev_theta))


def solve_ik_by_alpha_search(
    x: float,
    z: float,
    alpha_min: int = -80,
    alpha_max: int = 80,
    prev_theta: tuple[float, float, float] | None = None,
) -> tuple[float, float, float]:
    """搜索 alpha 寻找相对优解"""
    min_cost = 1e9
    min_cost_theta: tuple[float, float, float] | None = None
    for alpha in range(alpha_min, alpha_max + 1):
        # 遍历所有 alpha
        for elbow in (1, -1):
            try:
                theta1, theta2, theta3 = inverse_kinematics_2d(x, z, alpha, elbow)
            except ValueError:
                # 无解，直接忽略
                continue
            if not is_valid_theta(theta1, theta2, theta3):
                continue
            if not is_valid_transition((theta1, theta2, theta3), prev_theta):
                continue
            cur_cost = cost(theta1, theta2, theta3, prev_theta)
            if cur_cost < min_cost:
                # 记录最小惩罚值
                min_cost = cur_cost
                min_cost_theta = (theta1, theta2, theta3)
    if min_cost_theta is None:
        raise ValueError("cannot resolve alpha")
    return min_cost_theta
