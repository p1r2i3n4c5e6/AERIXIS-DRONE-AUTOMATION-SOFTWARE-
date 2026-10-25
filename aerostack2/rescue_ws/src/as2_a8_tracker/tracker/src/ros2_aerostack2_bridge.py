#!/usr/bin/env python3
"""
ros2_aerostack2_bridge.py
=========================
Subscribes to the UDP JSON Telemetry broadcasted by hybrid_tracker.py (port 9090)
and publishes target track coordinates, distances, and normalized errors
to Aerostac2 / ROS 2 topics.
"""

import json
import socket
import sys

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PointStamped, PoseStamped
    from std_msgs.msg import String, Bool
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


TELEMETRY_PORT = 9090


class Aerostac2VisionBridge(Node):
    def __init__(self):
        super().__init__("aerostac2_vision_bridge")
        self.pub_telemetry = self.create_publisher(String, "drone/vision/target_telemetry", 10)
        self.pub_target_detected = self.create_publisher(Bool, "drone/vision/target_found", 10)
        self.pub_target_point = self.create_publisher(PointStamped, "drone/vision/target_offset", 10)

        # Setup non-blocking UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", TELEMETRY_PORT))
        self.sock.setblocking(False)

        # 30Hz polling timer
        self.timer = self.create_timer(1.0 / 30.0, self.poll_telemetry)
        self.get_logger().info(f"Aerostac2 Vision Bridge active. Listening on UDP 127.0.0.1:{TELEMETRY_PORT}")

    def poll_telemetry(self):
        try:
            data, _ = self.sock.recvfrom(4096)
            packet = json.loads(data.decode("utf-8"))

            # Publish raw JSON string
            str_msg = String()
            str_msg.data = data.decode("utf-8")
            self.pub_telemetry.publish(str_msg)

            targets = packet.get("targets", [])
            found_msg = Bool()
            found_msg.data = len(targets) > 0
            self.pub_target_detected.publish(found_msg)

            if targets:
                primary = targets[0]
                pt_msg = PointStamped()
                pt_msg.header.stamp = self.get_clock().now().to_msg()
                pt_msg.header.frame_id = "camera_optical_link"
                pt_msg.point.x = float(primary["err_x"])
                pt_msg.point.y = float(primary["err_y"])
                pt_msg.point.z = float(primary["dist_m"])
                self.pub_target_point.publish(pt_msg)

        except BlockingIOError:
            pass
        except Exception as e:
            self.get_logger().warn(f"Bridge parse error: {e}")


def main():
    if not HAS_ROS2:
        print("⚠️ rclpy not installed or ROS 2 environment not sourced.")
        print("Run within your Aerostac2 ROS 2 workspace (e.g. source /opt/ros/humble/setup.bash)")
        sys.exit(1)

    rclpy.init()
    node = Aerostac2VisionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
