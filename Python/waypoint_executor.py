from arm_model import solve_ik_by_alpha_search
from command import make_q_command
from serial_sender import send_commands


def generate_waypoint_commands(
    waypoints: list[tuple[float, float]],
    time_ms: int = 1500,
) -> list[str]:
    if len(waypoints) == 0:
        raise ValueError("waypoints must not be empty")

    commands = []
    prev_theta = None

    for i, (x, z) in enumerate(waypoints):
        try:
            theta = solve_ik_by_alpha_search(
                x=x,
                z=z,
                prev_theta=prev_theta,
            )
        except ValueError as e:
            raise ValueError(
                f"waypoint {i} ({x:.2f}, {z:.2f}) cannot be reached continuously"
            ) from e

        cmd = make_q_command(theta, time_ms=time_ms)
        commands.append(cmd)

        prev_theta = theta

    return commands


def main():
    waypoints = [
        (40, 420),
        (60, 390),
        (80, 360),
        (100, 335),
        # (110, 320),
    ]

    time_ms = 1200
    wait_interval = 100
    commands = generate_waypoint_commands(waypoints, time_ms=time_ms)

    send_commands(
        commands,
        port="COM5",
        baudrate=115200,
        interval=time_ms + wait_interval,
    )


if __name__ == "__main__":
    main()
