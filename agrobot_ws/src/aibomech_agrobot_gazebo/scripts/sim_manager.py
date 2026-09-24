#!/usr/bin/env python3
"""Prepares a scenario world after the robot has been spawned, then starts physics.

Gazebo creates every DetachableJoint in the attached state, so each graspable
object starts out glued to the gripper. The world is therefore loaded paused;
this node releases all gripper joints and then unpauses the simulation.
"""
import subprocess
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Empty

GRASPABLE = ('strawberry_ripe', 'strawberry_unripe', 'seedling', 'weed')


class SimManager(Node):
    def __init__(self):
        super().__init__('sim_manager')
        self.world = self.declare_parameter('world', 'strawberry_greenhouse').value
        objects_file = self.declare_parameter('objects_file', '').value
        with open(objects_file) as fh:
            objects = yaml.safe_load(fh)['objects']
        self.graspable = [o['name'] for o in objects if o['type'] in GRASPABLE]
        self.pubs = [self.create_publisher(Empty, f'/agrobot/sim/{name}/detach_gripper', 10)
                     for name in self.graspable]

    def wait_for_bridge(self, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if all(p.get_subscription_count() > 0 for p in self.pubs):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def run(self):
        if not self.wait_for_bridge():
            self.get_logger().error('ros_gz_bridge is not subscribed to the grasp topics.')
            return False
        # gz-transport needs a moment to connect the bridge to the plugins.
        time.sleep(1.0)
        for _ in range(3):
            for pub in self.pubs:
                pub.publish(Empty())
            time.sleep(0.3)
        self.get_logger().info(f'Released {len(self.pubs)} gripper joints, starting physics.')
        cmd = ['ign', 'service', '-s', f'/world/{self.world}/control',
               '--reqtype', 'ignition.msgs.WorldControl', '--reptype', 'ignition.msgs.Boolean',
               '--timeout', '5000', '--req', 'pause: false']
        for _ in range(5):
            if 'data: true' in subprocess.run(cmd, capture_output=True, text=True).stdout:
                return True
            time.sleep(1.0)
        self.get_logger().error('Could not unpause the simulation.')
        return False


def main():
    rclpy.init()
    node = SimManager()
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
