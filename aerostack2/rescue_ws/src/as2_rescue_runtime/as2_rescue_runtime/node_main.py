"""One start/stop path for the rescue nodes.

ROS 2 Humble's own SIGINT handler shuts the ROS context down before Python's ``finally`` blocks
run. On every link restart the rescue nodes then crashed on the way out ("rcl_shutdown already
called", "publisher's context is invalid", destroy_node errors), and the runtime's last "stop and
hover" command was never sent (Pi 5 log, 2026-10-25). Here SIGINT/SIGTERM only stop the spin loop;
ROS stays up for the node's own shutdown step, then everything is closed once.
"""
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions


def _stop(_signum, _frame):
    raise KeyboardInterrupt


def run(make_node, args=None, graceful=None, cleanup=None, poll_s=0.2):
    """Spin ``make_node()`` until SIGINT/SIGTERM.

    graceful(node): runs while ROS is still up (may publish and spin), e.g. stop and hover.
    cleanup(node): closes the node's own files, sockets and locks.
    poll_s: the spin wakes at least this often, so a node with no traffic still sees the signal.
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    node = None
    try:
        node = make_node()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=poll_s)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)     # a second Ctrl+C must not cut the shutdown step
        try:
            if node is not None and graceful is not None and rclpy.ok():
                graceful(node)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                if node is not None and cleanup is not None:
                    cleanup(node)
            finally:
                if node is not None and rclpy.ok():
                    node.destroy_node()
                rclpy.try_shutdown()
