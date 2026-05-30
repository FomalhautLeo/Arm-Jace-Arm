import time
import serial

from arm_model import solve_ik_by_alpha_search, forward_kinematics_2d
from command import make_q_command, print_warnings
from serial_sender import send_command_to_open_port


def main() -> None:
    prev_theta = None

    port = "COM5"
    baudrate = 115200

    with serial.Serial(port, baudrate, timeout=1) as ser:
        time.sleep(2.0)

        while True:
            text = input("target x z, or q to quit: ").strip()

            if text.lower() in ("q", "quit", "exit"):
                break
            elif text.lower() == "reset":
                prev_theta = None
                print("prev_theta reset.")
                continue
            elif text.lower() == "home":
                prev_theta = None
                print("Joints go home")
                send_command_to_open_port(ser, "$H:2000!")
                continue

            try:
                x_str, z_str = text.split()
                x = float(x_str)
                z = float(z_str)
            except ValueError:
                print("Invalid input. Example: 100 360")
                continue

            try:
                theta = solve_ik_by_alpha_search(
                    x=x,
                    z=z,
                    prev_theta=prev_theta,
                )
            except ValueError as e:
                print(f"unreachable from current pose: {e}")

                try:
                    theta_free = solve_ik_by_alpha_search(x=x, z=z, prev_theta=None)
                    print("[INFO] target is reachable without continuity constraint")
                    print("free theta:", tuple(round(v, 2) for v in theta_free))
                    print("hint: type 'reset' if you want to switch branch")
                except ValueError:
                    print("[INFO] target is not reachable even without prev_theta")

                continue

            theta1, theta2, theta3 = theta
            alpha = theta1 + theta2 + theta3
            x2, z2 = forward_kinematics_2d(theta1, theta2, theta3)
            cmd = make_q_command(theta, time_ms=1200)

            print("-" * 40)
            print(f"target: x={x:.2f}, z={z:.2f}")
            print("theta:", tuple(round(v, 2) for v in theta))
            print(f"alpha: {alpha:.2f}")
            print(f"fk: x={x2:.2f}, z={z2:.2f}")
            print_warnings(theta)
            print("cmd:", cmd)

            confirm = input("Send this command? y/N: ").strip()
            if confirm and confirm[0].lower() == "y":
                send_command_to_open_port(ser, cmd)
                prev_theta = theta


if __name__ == "__main__":
    main()
