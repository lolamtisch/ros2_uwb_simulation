#!/usr/bin/env python3
import math
import numpy as np
import yaml
import rclpy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from uwb_broadcaster import UwbBroadcaster
from ros2_uwb_simulation.msg import UwbData

class UwbSimulation(UwbBroadcaster):

    def __init__(self, node_name = 'uwb_simulation'):
        super().__init__(node_name)

        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('range', 30.0)

        self.distance_publisher = self.create_publisher(UwbData, self.get_name()+'/distance', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(self.get_parameter('frequency').get_parameter_value().double_value, self.distance_measurement)

    def distance_measurement(self):

        # Get all frames and filter out the uwb frames
        frames_dict = yaml.safe_load(self.tf_buffer.all_frames_as_yaml())
        frames_list = list(frames_dict.keys())
        frames_list = [frame for frame in frames_list if frame != self.uwb_frame_id and frame.startswith('uwb_')]

        sensor_frames = []
        sensor_distance = []

        for frame in frames_list:
            try:
                # Get the distance between the uwb frame and the sensor frame
                t = self.tf_buffer.lookup_transform(frame, self.uwb_frame_id, rclpy.time.Time())
                distance = math.sqrt(t.transform.translation.x**2 + t.transform.translation.y**2 + t.transform.translation.z**2)

                # Only continue with frames that are in range
                if distance < self.get_parameter('range').get_parameter_value().double_value:
                    sensor_frames.append(frame)

                    # Add noise to the distance measurement
                    noise_distance = distance + np.random.normal(0, distance*0.015,1)[0]

                    sensor_distance.append(noise_distance)

            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {frame} to {self.uwb_frame_id}: {ex}')

        data = UwbData()
        data.frames = sensor_frames
        data.distance = sensor_distance
        data.stamp = self.get_clock().now().to_msg()

        self.distance_publisher.publish(data)

    def position_callback(self, request, response):
       response.type = 1
       return response


if __name__ == "__main__":
    rclpy.init()
    node = UwbSimulation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()