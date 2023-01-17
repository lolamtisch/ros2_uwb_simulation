#!/usr/bin/env python3
from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from ros2_uwb_simulation.srv import UwbPosition, RealPosition

class UwbBroadcaster(Node):

    def __init__(self, node_name = 'uwb_broadcaster'):
        super().__init__(node_name)
        self.declare_parameter('translation', [0.0, 0.0, 0.0])
        self.real_position = self.get_parameter('translation').get_parameter_value().double_array_value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.position_srv = self.create_service(UwbPosition, self.get_name()+'/get_position', self.position_callback, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        self.set_real_position_srv = self.create_service(RealPosition, self.get_name()+'/set_real_position', self.set_real_position_callback, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())

        self.timer = self.create_timer(0.1, self.real_position_publish)
        self.uwb_frame_id = 'uwb_'+self.get_name()

    def real_position_publish(self):
        translation = self.real_position
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'

        t.child_frame_id = self.uwb_frame_id
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    def position_callback(self, request, response):
        translation = self.real_position

        response.x = translation[0]
        response.y = translation[1]
        response.z = translation[2]
        response.type = 0

        return response

    def set_real_position_callback(self, request, response):

        self.real_position[0] = request.x
        self.real_position[1] = request.y
        self.real_position[2] = request.z

        return response

if __name__ == "__main__":
    rclpy.init()
    node = UwbBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()