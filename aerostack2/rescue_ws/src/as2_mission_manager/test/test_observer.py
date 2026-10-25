"""ROS graph isolation and fresh ground-state observation; no flight commands."""
import time
import unittest
import rclpy
from rclpy.node import Node
from as2_msgs.msg import PlatformInfo
from as2_mission_manager.observer import RosObserver

class ObserverTests(unittest.TestCase):
    def test_namespace_and_stale_platform(self):
        rclpy.init(args=[])
        observer=RosObserver('observer_bench')
        own=Node('camera_bench',namespace='observer_bench')
        other=Node('other_camera',namespace='different_bench')
        publisher=own.create_publisher(PlatformInfo,'platform/info',10)
        try:
            message=PlatformInfo();message.connected=True;message.armed=False;message.status.state=1
            end=time.monotonic()+5
            while time.monotonic()<end:
                publisher.publish(message);time.sleep(.1)
                snapshot=observer.snapshot()
                if observer.safe_on_ground() and '/observer_bench/camera_bench' in (snapshot['nodes'] or []):break
            self.assertTrue(observer.safe_on_ground())
            self.assertIn('/observer_bench/camera_bench',snapshot['nodes'])
            self.assertNotIn('/different_bench/other_camera',snapshot['nodes'])
            time.sleep(2.1)
            self.assertFalse(observer.safe_on_ground())
            message.armed=True;publisher.publish(message);time.sleep(.2)
            self.assertFalse(observer.safe_on_ground())
        finally:observer.close();own.destroy_node();other.destroy_node();rclpy.shutdown()

if __name__=='__main__':unittest.main()
