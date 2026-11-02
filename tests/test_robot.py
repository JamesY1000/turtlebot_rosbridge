"""Tests for the public robot API without a physical robot."""

import base64
import threading
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from rosbridge_api import LaserScan, RosbridgeRobot
from rosbridge_api import robot as robot_module


class FakeRos:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.is_connected = False
        self.terminated = False

    def run(self, timeout=10):
        self.is_connected = True

    def terminate(self):
        self.is_connected = False
        self.terminated = True


class FakeTopic:
    instances = []

    def __init__(self, client, name, message_type):
        self.client = client
        self.name = name
        self.message_type = message_type
        self.messages = []
        self.callback = None
        self.advertised = False
        self.unsubscribed = False
        self.__class__.instances.append(self)

    def advertise(self):
        self.advertised = True

    def unadvertise(self):
        self.advertised = False

    def publish(self, message):
        self.messages.append(message)

    def subscribe(self, callback):
        self.callback = callback

    def unsubscribe(self):
        self.unsubscribed = True


class RosbridgeRobotTest(unittest.TestCase):
    def setUp(self):
        FakeTopic.instances.clear()
        self.ros_patch = patch.object(robot_module.roslibpy, "Ros", FakeRos)
        self.topic_patch = patch.object(robot_module.roslibpy, "Topic", FakeTopic)
        self.ros_patch.start()
        self.topic_patch.start()
        self.addCleanup(self.ros_patch.stop)
        self.addCleanup(self.topic_patch.stop)

        self.robot = RosbridgeRobot("robot.local")
        self.robot.setup()
        self.addCleanup(self.robot.cleanup)

    def topic(self, name):
        return next(topic for topic in FakeTopic.instances if topic.name == name)

    def image_message(self, **fields):
        source = np.full((3, 4, 3), [10, 20, 30], dtype=np.uint8)
        success, encoded = cv2.imencode(".png", source)
        self.assertTrue(success)
        return {
            "format": "png",
            "data": base64.b64encode(encoded).decode("ascii"),
            **fields,
        }

    def scan_message(self, **fields):
        return {
            "angle_min": -0.5,
            "angle_max": 0.5,
            "angle_increment": 0.5,
            "range_min": 0.12,
            "range_max": 8.0,
            "ranges": [1.0, 2.0, 3.0],
            "intensities": [10.0, 20.0, 30.0],
            "scan_time": 0.1,
            **fields,
        }

    def streams(self):
        return (
            ("/camera/image_raw/compressed", self.image_message, self.robot.query_image),
            ("/scan", self.scan_message, self.robot.query_laser_scan),
        )

    def test_setup_and_send_command(self):
        self.assertTrue(self.robot.is_connected)
        self.assertEqual(set(self.robot._publishers), {"cmd_vel"})
        self.assertEqual(
            set(self.robot._subscribers),
            {"image", "laser_scan", "joint_states"},
        )
        self.robot.send_command(0.2, -0.4)

        message = self.topic("/cmd_vel").messages[-1]
        self.assertEqual(message["linear"]["x"], 0.2)
        self.assertEqual(message["angular"]["z"], -0.4)

    def test_query_decodes_compressed_image_and_returns_copy(self):
        source = np.full((3, 4, 3), 127, dtype=np.uint8)
        success, encoded = cv2.imencode(".png", source)
        self.assertTrue(success)
        payload = base64.b64encode(encoded).decode("ascii")

        self.topic("/camera/image_raw/compressed").callback({"data": payload})
        first = self.robot.query_image(timeout=0)
        second = self.robot.query_image(timeout=0)

        np.testing.assert_array_equal(first, source)
        self.assertIsNot(first, second)

    def test_query_timeout(self):
        self.assertIsNone(self.robot.query_image(timeout=0.01))

    def test_query_laser_scan_returns_numpy_arrays_and_copy(self):
        payload = {
            "angle_min": -1.0,
            "angle_max": 1.0,
            "angle_increment": 0.5,
            "time_increment": 0.001,
            "scan_time": 0.1,
            "range_min": 0.12,
            "range_max": 8.0,
            "ranges": [1.0, 2.0, float("inf"), 4.0, float("nan")],
            "intensities": [10.0, 20.0, 0.0, 40.0, 0.0],
        }
        self.topic("/scan").callback(payload)

        first = self.robot.query_laser_scan(timeout=0)
        second = self.robot.query_laser_scan(timeout=0)

        np.testing.assert_allclose(
            first.angles, [-1.0, -0.5, 0.0, 0.5, 1.0]
        )
        np.testing.assert_allclose(first.ranges, payload["ranges"], equal_nan=True)
        np.testing.assert_allclose(first.intensities, payload["intensities"])
        self.assertEqual(first.range_min, 0.12)
        self.assertEqual(first.range_max, 8.0)
        self.assertIsNot(first.ranges, second.ranges)

    def test_laser_scan_timeout(self):
        self.assertIsNone(self.robot.query_laser_scan(timeout=0.01))

    def test_wait_for_new_laser_scan_times_out_after_returning_latest(self):
        payload = {
            "angle_min": 0.0,
            "angle_max": 0.0,
            "angle_increment": 1.0,
            "range_min": 0.1,
            "range_max": 5.0,
            "ranges": [1.0],
        }
        self.topic("/scan").callback(payload)
        self.assertIsNotNone(self.robot.query_laser_scan(timeout=0))
        self.assertIsNone(
            self.robot.query_laser_scan(timeout=0.01, wait_for_new=True)
        )

    def test_cleanup_stops_robot_and_wakes_query(self):
        image_result = []
        scan_result = []
        image_thread = threading.Thread(
            target=lambda: image_result.append(self.robot.query_image())
        )
        scan_thread = threading.Thread(
            target=lambda: scan_result.append(self.robot.query_laser_scan())
        )
        image_thread.start()
        scan_thread.start()
        time.sleep(0.02)

        self.robot.cleanup()
        image_thread.join(timeout=1.0)
        scan_thread.join(timeout=1.0)

        self.assertFalse(image_thread.is_alive())
        self.assertFalse(scan_thread.is_alive())
        self.assertEqual(image_result, [None])
        self.assertEqual(scan_result, [None])
        self.assertEqual(self.topic("/cmd_vel").messages[-1]["linear"]["x"], 0.0)
        self.assertTrue(self.topic("/scan").unsubscribed)
        self.assertEqual(self.robot._publishers, {})
        self.assertEqual(self.robot._subscribers, {})
        self.assertFalse(self.robot.is_connected)

    def test_methods_require_setup(self):
        self.robot.cleanup()
        with self.assertRaisesRegex(RuntimeError, "setup"):
            self.robot.send_command(0.1, 0.0)

    def test_negative_timeout_is_rejected(self):
        with self.assertRaises(ValueError):
            self.robot.query_image(timeout=-1)

    def test_raw_messages_preserve_complete_payload_and_header(self):
        headers = (
            {"stamp": {"sec": 1_700_000_000, "nanosec": 123_456_789},
             "frame_id": "sensor"},
            {"stamp": {"secs": 42, "nsecs": 7}, "frame_id": "sensor", "seq": 8},
        )
        for topic, make_message, query in self.streams():
            for header in headers:
                with self.subTest(topic=topic, header=header):
                    message = make_message(header=header, extra={"values": [1, 2]})
                    self.topic(topic).callback(message)
                    raw = query(timeout=0, raw=True)
                    self.assertIsInstance(raw, dict)
                    self.assertEqual(raw, message)
                    self.assertIsNot(raw, message)

        image = self.robot.query_image(timeout=0, raw=False)
        scan = self.robot.query_laser_scan(timeout=0, raw=False)
        np.testing.assert_array_equal(image[0, 0], [10, 20, 30])
        self.assertIsInstance(scan, LaserScan)
        np.testing.assert_allclose(scan.ranges, [1.0, 2.0, 3.0])

    def test_raw_message_copies_protect_nested_header_and_payload(self):
        for topic, make_message, query in self.streams():
            with self.subTest(topic=topic):
                message = make_message(
                    header={"stamp": {"sec": 42, "nanosec": 7}},
                    extra={"values": [1]},
                )
                self.topic(topic).callback(message)
                first = query(timeout=0, raw=True)
                message["header"]["stamp"]["sec"] = 0
                first["header"]["stamp"]["sec"] = 1
                first["extra"]["values"][0] = 99
                if "ranges" in first:
                    message["ranges"][0] = 0
                    first["ranges"][1] = 0
                    first["intensities"][0] = 0
                else:
                    message["data"] = "changed"
                    first["data"] = "changed again"
                second = query(timeout=0, raw=True)
                self.assertEqual(second["header"]["stamp"]["sec"], 42)
                self.assertEqual(second["extra"]["values"], [1])
                if "ranges" in second:
                    self.assertEqual(second["ranges"], [1.0, 2.0, 3.0])
                    self.assertEqual(second["intensities"], [10.0, 20.0, 30.0])
                else:
                    self.assertEqual(second["data"], make_message()["data"])

    def test_raw_and_decoded_queries_share_wait_for_new_cursor(self):
        for topic, make_message, query in self.streams():
            with self.subTest(topic=topic):
                message = make_message(header={"stamp": {"sec": 1, "nanosec": 0}})
                self.topic(topic).callback(message)
                self.assertEqual(query(timeout=0, raw=True, wait_for_new=True), message)
                self.assertIsNone(query(timeout=0, wait_for_new=True))
                # A repeated source stamp can still arrive in a new message.
                self.topic(topic).callback(message)
                self.assertIsNotNone(query(timeout=0, wait_for_new=True))
                self.assertIsNone(query(timeout=0, raw=True, wait_for_new=True))
                self.assertEqual(query(timeout=0, raw=True), message)

    def test_raw_mode_preserves_missing_and_unusual_headers(self):
        for topic, make_message, query in self.streams():
            for header in (None, {}, {"stamp": {"sec": 0, "nanosec": 0}},
                           {"stamp": "invalid", "frame_id": 123}):
                with self.subTest(topic=topic, header=header):
                    message = make_message(header=header)
                    self.topic(topic).callback(message)
                    self.assertEqual(query(timeout=0, raw=True), message)
                    self.assertIsNotNone(query(timeout=0))
            self.topic(topic).callback(make_message())
            self.assertNotIn("header", query(timeout=0, raw=True))

    def test_raw_mode_timeout_and_connection_errors(self):
        for _, _, query in self.streams():
            self.assertIsNone(query(timeout=0.01, raw=True))
            with self.assertRaises(ValueError):
                query(timeout=-1, raw=True)
        self.robot.cleanup()
        for _, _, query in self.streams():
            with self.assertRaisesRegex(RuntimeError, "setup"):
                query(timeout=0, raw=True)

    def test_cleanup_wakes_raw_queries(self):
        for topic, _, query in self.streams():
            with self.subTest(topic=topic):
                self.robot.setup()
                condition = (self.robot._image_condition if topic.startswith("/camera")
                             else self.robot._laser_scan_condition)
                waiting = threading.Event()
                wait = condition.wait
                result = []

                def wait_and_signal(timeout=None, *, waiting=waiting, wait=wait):
                    waiting.set()
                    return wait(timeout)

                with patch.object(condition, "wait", side_effect=wait_and_signal):
                    thread = threading.Thread(
                        target=lambda result=result, query=query: result.append(
                            query(raw=True)
                        ),
                        daemon=True,
                    )
                    thread.start()
                    try:
                        self.assertTrue(waiting.wait(timeout=1))
                    finally:
                        self.robot.cleanup()
                        thread.join(timeout=1)
                self.assertFalse(thread.is_alive())
                self.assertEqual(result, [None])

    def test_reconnect_clears_raw_and_decoded_caches(self):
        for topic, make_message, query in self.streams():
            self.topic(topic).callback(make_message())
            self.assertIsNotNone(query(timeout=0, raw=True))
        self.robot.cleanup()
        FakeTopic.instances.clear()
        self.robot.setup()
        for topic, make_message, query in self.streams():
            self.assertIsNone(query(timeout=0, raw=True))
            self.assertIsNone(query(timeout=0))
            message = make_message(header={"frame_id": "new_session"})
            self.topic(topic).callback(message)
            self.assertEqual(query(timeout=0, raw=True, wait_for_new=True), message)

    def test_invalid_payload_does_not_replace_raw_or_decoded_cache(self):
        for topic, make_message, query in self.streams():
            with self.subTest(topic=topic):
                message = make_message(header={"frame_id": "valid"})
                self.topic(topic).callback(message)
                query(timeout=0)
                with patch("builtins.print"):
                    self.topic(topic).callback({"header": {"frame_id": "invalid"}})
                self.assertIsNone(query(timeout=0, raw=True, wait_for_new=True))
                self.assertEqual(query(timeout=0, raw=True), message)
                self.assertIsNotNone(query(timeout=0))


if __name__ == "__main__":
    unittest.main()
