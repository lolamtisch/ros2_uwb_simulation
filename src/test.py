#!/usr/bin/env python3
from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from ros2_uwb_simulation.srv import UwbPosition, RealPosition

class Test(Node):

    def __init__(self, node_name = 'test'):
        super().__init__(node_name)

        cb_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.uwb1 = self.create_client(RealPosition, 'device_position/set_real_position', callback_group=cb_group)
        self.uwb2 = self.create_client(RealPosition, 'device_position2/set_real_position', callback_group=cb_group)

        self.timer = self.create_timer(0.1, self.real_position_publish)

        self.uwb1Y = {
            "v": 0.0,
            "d": 1,
        }

        self.uwb2X = {
            "v": -3.4,
            "d": 1,
        }

        self.uwb2Y = {
            "v": 0.0,
            "d": 1,
        }

    def real_position_publish(self):
        t = RealPosition.Request()

        self.uwb1Y["v"] += 0.005 * self.uwb1Y["d"]

        if self.uwb1Y["v"] > 1.0:
            self.uwb1Y["d"] = -1
        elif self.uwb1Y["v"] < -1.0:
            self.uwb1Y["d"] = 1

        t.x = -1.4
        t.y = self.uwb1Y["v"]
        t.z = 0.0

        self.uwb1.call_async(t)

        t = RealPosition.Request()

        self.uwb2X["v"] += 0.025 * self.uwb2X["d"]

        if self.uwb2X["v"] > -2.2:
            self.uwb2X["d"] = -1
        elif self.uwb2X["v"] < -5.2:
            self.uwb2X["d"] = 1

        self.uwb2Y["v"] += 0.025 * self.uwb2Y["d"]

        if self.uwb2Y["v"] > 1.0:
            self.uwb2Y["d"] = -1
        elif self.uwb2Y["v"] < -1.0:
            self.uwb2Y["d"] = 1

        t.x = self.uwb2X["v"]
        t.y = self.uwb2Y["v"]
        t.z = 0.0

        self.uwb2.call_async(t)


if __name__ == "__main__":
    rclpy.init()
    node = Test()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()