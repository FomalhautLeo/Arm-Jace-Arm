import time
import serial


def send_command_to_open_port(ser: serial.Serial, cmd: str) -> None:
    line = cmd + "\r\n"
    print(f"[SEND] {cmd}")
    ser.write(line.encode("utf-8"))


def send_commands_to_port(
    port: str,
    baudrate: int,
    commands: list[str],
    interval: float,
) -> None:
    """向制定串口发送数据"""
    with serial.Serial(port, baudrate, timeout=1) as ser:
        time.sleep(2.0)

        for i, cmd in enumerate(commands):
            line = cmd + "\r\n"
            print(f"[SEND {i}] {cmd}")
            ser.write(line.encode("utf-8"))
            time.sleep(interval / 1000)


def send_commands(
    commands: list[str],
    port: str = "COM5",
    baudrate: int = 115200,
    interval: float = 1700,
) -> None:
    """发送命令"""
    print("Commands to send:")
    for i, cmd in enumerate(commands):
        print(f"{i}: {cmd}")

    confirm = input("Send to robot? Type YES to continue: ")
    if not confirm or confirm[0].upper() != "Y":
        print("Cancelled.")
        return

    send_commands_to_port(
        port=port,
        baudrate=baudrate,
        commands=commands,
        interval=interval,
    )
