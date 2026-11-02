"""Control robot velocity from the keyboard using the public package API."""

import argparse
import math
import os
import sys
import time
from contextlib import contextmanager

from rosbridge_api import RosbridgeRobot


@contextmanager
def terminal_keys():
    """Read Linux terminal keys without Enter and restore terminal settings."""
    if not sys.stdin.isatty():
        raise RuntimeError("Run teleop in an interactive terminal and keep it focused")
    import select
    import termios
    import tty

    fd = sys.stdin.fileno()
    original = termios.tcgetattr(fd)

    def read(timeout):
        if select.select([fd], [], [], timeout)[0]:
            return os.read(fd, 1024)  # b"" means EOF; None means no input yet.
        return None

    try:
        tty.setcbreak(fd)
        yield read
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, original)


def terminal_teleop(robot, publish_rate, key_timeout):
    """Use repeated key input as a deadman signal; terminals have no releases."""
    commands = {
        ord("w"): (0.1, 0.0),
        ord("s"): (-0.1, 0.0),
        ord("a"): (0.0, 0.8),
        ord("d"): (0.0, -0.8),
    }
    command = (0.0, 0.0)
    expires_at = math.inf
    next_publish = time.monotonic()
    print(
        "Focus this terminal. W/S: forward/backward | A/D: turn | Space: stop | Q/Escape: quit"
    )
    print(
        f"One direction at a time; stops after {key_timeout:g}s without a key repeat."
    )
    with terminal_keys() as read:
        while robot.is_connected:
            timeout = max(0.0, min(next_publish, expires_at) - time.monotonic())
            keys = read(timeout)
            now = time.monotonic()
            if keys == b"":
                return
            if keys is not None:
                for key in keys.lower():
                    if key in (ord("q"), 27, 3, 4):  # Q, Escape, Ctrl+C/D
                        return
                    command = commands.get(key, (0.0, 0.0))
                    expires_at = (
                        now + key_timeout if command != (0.0, 0.0) else math.inf
                    )
            expired = now >= expires_at
            if expired:
                command, expires_at = (0.0, 0.0), math.inf
            if keys is not None or expired or now >= next_publish:
                robot.send_command(*command)
                next_publish = now + 1.0 / publish_rate


def teleop_example(
    robot: RosbridgeRobot,
    publish_rate: float = 10.0,
    *,
    key_timeout: float = 0.3,
) -> None:
    """Run keyboard teleop and stop on exit, interrupt or input failure."""
    if not math.isfinite(publish_rate) or publish_rate <= 0:
        raise ValueError("publish_rate must be positive and finite")
    if not math.isfinite(key_timeout) or key_timeout <= 0:
        raise ValueError("key_timeout must be positive and finite")
    try:
        robot.stop()
        terminal_teleop(robot, publish_rate, key_timeout)
    finally:
        if robot.is_connected:
            robot.stop()


def main() -> None:
    """Connect to a robot and run the keyboard teleop example."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host", help="Robot hostname or IP address")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--rate", type=float, default=10.0, help="Publish rate")
    parser.add_argument(
        "--key-timeout",
        type=float,
        default=0.3,
        help="Stop after this many seconds without key input",
    )
    args = parser.parse_args()
    if not math.isfinite(args.rate) or args.rate <= 0:
        parser.error("--rate must be positive and finite")
    if not math.isfinite(args.key_timeout) or args.key_timeout <= 0:
        parser.error("--key-timeout must be positive and finite")

    try:
        with RosbridgeRobot(args.host, port=args.port) as robot:
            teleop_example(
                robot,
                publish_rate=args.rate,
                key_timeout=args.key_timeout,
            )
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
