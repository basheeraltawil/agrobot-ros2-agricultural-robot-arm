"""Software emergency stop for the task layer.

  ros2 run aibomech_agrobot_tasks estop on     # cancel motion, abort the task
  ros2 run aibomech_agrobot_tasks estop off    # reset

This complements, and never replaces, the hardware e-stop circuit that cuts
actuator power on the real robot.
"""
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ('on', 'off'):
        print(__doc__)
        raise SystemExit(1)
    rclpy.init()
    node = rclpy.create_node('estop_cli')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(Bool, '/agrobot/estop', qos)
    pub.publish(Bool(data=sys.argv[1] == 'on'))
    time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
