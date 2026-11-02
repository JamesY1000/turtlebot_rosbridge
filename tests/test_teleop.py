"""Exercise teleop with fake commands and an isolated pseudo-terminal, never a robot."""

import importlib.util
import os
import unittest
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location(
    "teleop_example", Path(__file__).resolve().parents[1] / "examples/teleop.py"
)
teleop = importlib.util.module_from_spec(spec)
spec.loader.exec_module(teleop)


class FakeRobot:
    is_connected = True

    def __init__(self):
        self.commands = []

    def send_command(self, linear, angular):
        self.commands.append((linear, angular))

    def stop(self):
        self.send_command(0.0, 0.0)


class TeleopTests(unittest.TestCase):
    def run_keys(self, events, *, rate=10, key_timeout=0.3):
        """Supply (source time, bytes/exception) events on a deterministic clock."""
        robot = FakeRobot()
        now = [0.0]
        events = list(events)

        @contextmanager
        def keys():
            def read(timeout):
                if not events:
                    return b"q"
                when, value = events[0]
                if when > now[0] + timeout:
                    now[0] += timeout
                    return None
                now[0] = when
                events.pop(0)
                if isinstance(value, BaseException):
                    raise value
                return value

            yield read

        with (
            patch.object(teleop, "terminal_keys", keys),
            patch.object(teleop.time, "monotonic", side_effect=lambda: now[0]),
            patch("builtins.print"),
        ):
            teleop.teleop_example(robot, rate, key_timeout=key_timeout)
        return robot.commands

    def test_wasd_uppercase_stop_and_quit(self):
        commands = self.run_keys(
            [
                (0, b"W"),
                (0.02, b"s"),
                (0.04, b"A"),
                (0.06, b"d"),
                (0.08, b" "),
                (0.1, b"q"),
            ]
        )
        self.assertEqual(
            commands,
            [
                (0.0, 0.0),
                (0.1, 0.0),
                (-0.1, 0.0),
                (0.0, 0.8),
                (0.0, -0.8),
                (0.0, 0.0),
                (0.0, 0.0),
            ],
        )

    def test_no_key_repeat_stops_even_at_low_publish_rate(self):
        commands = self.run_keys([(0, b"w"), (0.35, b"q")], rate=1)
        self.assertEqual(commands, [(0.0, 0.0), (0.1, 0.0), (0.0, 0.0), (0.0, 0.0)])

    def test_key_repeats_renew_timeout(self):
        commands = self.run_keys([(0, b"w"), (0.2, b"w"), (0.4, b"w"), (0.6, b"q")])
        self.assertTrue(all(command == (0.1, 0.0) for command in commands[1:-1]))

    def test_unknown_key_stops_without_waiting_for_timeout(self):
        commands = self.run_keys([(0, b"w"), (0.01, b"x"), (0.02, b"q")])
        self.assertEqual(commands, [(0.0, 0.0), (0.1, 0.0), (0.0, 0.0), (0.0, 0.0)])

    def test_eof_escape_and_control_keys_stop(self):
        for key in (b"", b"\x1b", b"\x03", b"\x04"):
            with self.subTest(key=key):
                commands = self.run_keys([(0, b"w"), (0.01, key)])
                self.assertEqual(commands[-1], (0.0, 0.0))

    def test_keyboard_interrupt_and_reader_error_stop(self):
        for error in (KeyboardInterrupt(), OSError("input failed")):
            robot = FakeRobot()
            reader = Mock(side_effect=[b"w", error])
            with patch.object(teleop, "terminal_keys") as keys, patch("builtins.print"):
                keys.return_value.__enter__.return_value = reader
                with self.assertRaises(type(error)):
                    teleop.teleop_example(robot)
            self.assertIn((0.1, 0.0), robot.commands)
            self.assertEqual(robot.commands[-1], (0.0, 0.0))

    def test_invalid_options_do_not_publish_motion(self):
        for kwargs in (
            {"publish_rate": 0},
            {"publish_rate": float("nan")},
            {"key_timeout": float("inf")},
            {"key_timeout": -1},
        ):
            robot = FakeRobot()
            with self.assertRaises(ValueError):
                teleop.teleop_example(robot, **kwargs)
            self.assertFalse(robot.commands)

    @unittest.skipUnless(os.name == "posix", "POSIX terminal required")
    def test_real_terminal_reads_without_enter_and_restores_settings(self):
        import termios

        master, slave = os.openpty()
        try:
            with (
                os.fdopen(slave, "r") as stream,
                patch.object(teleop.sys, "stdin", stream),
            ):
                original = termios.tcgetattr(stream.fileno())
                with (
                    self.assertRaisesRegex(RuntimeError, "test exit"),
                    teleop.terminal_keys() as read,
                ):
                    os.write(master, b"w ")
                    self.assertEqual(read(0.5), b"w ")
                    self.assertIsNone(read(0))
                    raise RuntimeError("test exit")
                self.assertEqual(termios.tcgetattr(stream.fileno()), original)
        finally:
            os.close(master)

    def test_noninteractive_input_reports_actionable_error(self):
        stream = Mock()
        stream.isatty.return_value = False
        with (
            patch.object(teleop.sys, "stdin", stream),
            self.assertRaisesRegex(RuntimeError, "interactive terminal"),
            teleop.terminal_keys(),
        ):
            self.fail("Should reject piped input")

    def test_cli_defaults_to_terminal_without_keyboard_flag(self):
        with (
            patch.object(teleop.sys, "argv", ["teleop.py", "robot.invalid"]),
            patch.object(teleop, "RosbridgeRobot") as client,
            patch.object(teleop, "teleop_example") as run,
        ):
            teleop.main()
        run.assert_called_once_with(
            client.return_value.__enter__.return_value,
            publish_rate=10.0,
            key_timeout=0.3,
        )


if __name__ == "__main__":
    unittest.main()
