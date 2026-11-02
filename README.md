# Student Rosbridge API

A small Python API for students who use an ML model or robotics modelling
toolbox to control a ROS robot through rosbridge. It exposes only the main
operations a student control loop needs:

- sign in to borrow the robot and another sign-in for the battery   
- log into `turtlebot-MMR wifi`, pwd: `Turtlebot`
- ping 192.168.1.2XX where XX is the TurtleBot ID   
- query the latest compressed camera image as a BGR NumPy array;
- query the latest laser scan as angle and range NumPy arrays;
- optionally retrieve the complete camera/scan ROS message with `raw=True`;
- publish linear and angular velocity to `/cmd_vel`;
- stop and disconnect safely.

## Robot requirements

The robot must:

1. run a rosbridge WebSocket server reachable by the student's computer;
2. accept `geometry_msgs/msg/Twist` messages on `/cmd_vel`;
3. publish `sensor_msgs/msg/CompressedImage` messages on
   `/camera/image_raw/compressed`;
4. publish `sensor_msgs/msg/LaserScan` messages on `/scan`.

Topic names can be changed when constructing `RosbridgeRobot`.

## Install locally for testing

From this directory:

```bash
python -m pip install .
```

Include the keyboard teleoperation dependency with:

```bash
python -m pip install ".[teleop]"
```

For active development, use an editable installation:

```bash
python -m pip install -e ".[teleop,dev]"
```

## ML/control-loop example

```python
from rosbridge_api import RosbridgeRobot

with RosbridgeRobot("192.168.1.213") as robot:
    while True:
        image = robot.query_image(timeout=2.0, wait_for_new=True)
        if image is None:
            continue

        linear_velocity, angular_velocity = controller(image)
        robot.send_command(linear_velocity, angular_velocity)
```

`query_image()` returns a copy, so student code may safely annotate or resize it.
Its colour order is BGR, as expected by OpenCV.

## Raw sensor messages

Pass `raw=True` to the existing image or scan query to get a deep copy of the
complete ROS message dictionary, including its original `header` and payload:

```python
from rosbridge_api import RosbridgeRobot

with RosbridgeRobot("192.168.1.213") as robot:
    message = robot.query_image(timeout=2.0, wait_for_new=True, raw=True)
    if message is not None:
        header = message.get("header", {})
        print(header.get("stamp"), header.get("frame_id"))
        compressed_data = message["data"]  # Base64 string, not decoded pixels

    scan = robot.query_laser_scan(timeout=2.0, wait_for_new=True, raw=True)
    if scan is not None:
        ranges = scan["ranges"]  # Original list, not a NumPy array
        scan_header = scan.get("header", {})
```

Without `raw=True`, return types stay unchanged: a BGR NumPy image or `LaserScan`.
Both modes use the latest successfully decoded message; payloads rejected by
the existing decoder do not replace either cached representation. Raw results
preserve all message fields without parsing timestamps or adding receipt times.
ROS 2 stamps use `sec`/`nanosec`; ROS 1 uses `secs`/`nsecs`. Missing or unusual
headers are returned as received, so validation belongs in the application.

Timeout and `wait_for_new` behaviour are the same in both modes. Each stream
shares one query cursor: a raw read also consumes the latest update for a later
decoded read using `wait_for_new=True`, and vice versa. Reading cached data does
not make it fresh. To associate headers with pixels or ranges, use one raw
message and decode its payload; separate queries can return different updates.

Raw dictionaries are deep copies, so editing nested headers or range lists does
not change the API's cache. `query_joint_states()` already returns the original
ROS message dictionary and is unchanged by this option.

## Included examples

The runnable examples are kept separately from the library in `examples/`.
After installing the package, run them from the project directory:

```bash
python examples/camera.py 192.168.1.213
python examples/laser_scan.py 192.168.1.213
python examples/teleop.py 192.168.1.213
```

The camera and laser-scan visualisers exit with `Q` or Escape.

### Keyboard teleoperation

Teleop reads the **focused Linux terminal**, using Python's standard library.
This supports Wayland/X11 desktops and interactive SSH sessions.
There is no keyboard selection flag or `pynput` requirement. No Enter key is needed:

```bash
python examples/teleop.py ROBOT_IP
```

Use W/S for forward/backward, A/D for turning, Space to stop, and Q/Escape or
Ctrl+C to exit. Uppercase keys also work. Teleop selects one direction
at a time: A/D turn in place rather than adding a turn to a held W/S command.
Other keys stop motion.

Terminals do not report key release. Each movement key enables motion for at
most **0.3 seconds**, renewed by subsequent key repeats. Holding a key uses the
desktop's repeat settings; there may be a brief stop before the initial repeat
starts. `--key-timeout 0.6` accommodates a longer repeat delay, but also permits
motion for up to 0.6 seconds after release or loss of terminal focus. Space
stops immediately. Keep the terminal focused while controlling the robot.

Use a real interactive Linux terminal. Redirected input and IDE output/debug consoles are not
supported. Compared with the previous global listener, keys are only read while
the terminal is focused, combined held keys are not tracked, and release stopping
uses the timeout above. The ROS connection and velocity command API are unchanged.

Teleop sends zero velocity on startup and exit/error while connected.
As with other remote command examples, stopping after a lost connection or a
killed process depends on the robot's own command timeout.

Input tests exercise a Linux pseudo-terminal. No Windows-specific input code is included.

## Test and build

```bash
python -m unittest discover -s tests -v
python -m build
python -m twine check --strict dist/*
```

The tests mock rosbridge and do not require a physical robot.

Before publishing, choose the final project name, add author/project metadata,
choose a licence, and verify installation from the generated wheel in a clean
virtual environment.
