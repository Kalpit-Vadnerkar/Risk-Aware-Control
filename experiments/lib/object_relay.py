#!/usr/bin/env python3
"""Relay /perception/object_recognition/objects -> objects_filtered.

Autoware's planning launch is patched to read objects_filtered, so this has
to run during collection (run_experiments.py starts it). It copies the
serialized message unchanged: the hook where a perception fault would be
injected later (the old obstacle/delay/dropout interceptor lived here).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from autoware_perception_msgs.msg import PredictedObjects

IN_TOPIC = '/perception/object_recognition/objects'
OUT_TOPIC = '/perception/object_recognition/objects_filtered'
QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=5)


class ObjectRelay(Node):
    def __init__(self):
        super().__init__('object_relay')
        self._pub = self.create_publisher(PredictedObjects, OUT_TOPIC, QOS)
        self._sub = self.create_subscription(PredictedObjects, IN_TOPIC, self._pub.publish, QOS, raw=True)
        self.get_logger().info(f'relaying {IN_TOPIC} -> {OUT_TOPIC}')


def main():
    rclpy.init()
    node = ObjectRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
