#!/usr/bin/env python3
import math
import numpy as np
import yaml
import rclpy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from uwb_broadcaster import UwbBroadcaster
from pozyx_simulation.msg import UwbData

class UwbSimulation(UwbBroadcaster):

    def __init__(self):
        super().__init__('uwb_simulation')

        self.uwb_config = {
            'range': 2.5,
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.distance_measurement)

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
                if distance < self.uwb_config['range']:
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

        self.get_logger().info(str(data))


if __name__ == "__main__":
    rclpy.init()
    node = UwbSimulation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()