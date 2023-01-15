#!/usr/bin/env python3
import numpy as np
import rclpy

from uwb_simulation import UwbSimulation
from ros2_uwb_simulation.srv import UwbPosition as UwbPositionSrv
from ros2_uwb_simulation.msg import UwbData

class UwbPosition(UwbSimulation):

    def __init__(self):
        super().__init__('uwb_position')

        self.mu = [0, 0, 0.0]
        self.sigma = np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]])

        self.subscription = self.create_subscription(
            UwbData,
            self.get_name()+'/distance',
            self.listener_uwb_data,
            10)

        self.client_map = {}

    async def listener_uwb_data(self, uwb_data):
        sensor_frames = uwb_data.frames
        sensor_distance = uwb_data.distance

        position = []

        for frame in sensor_frames:
            service_name = frame[4:] + '/get_position'

            if frame not in self.client_map:
                cb_group = rclpy.callback_groups.ReentrantCallbackGroup()
                self.client_map[frame] = self.create_client(UwbPositionSrv, service_name, callback_group=cb_group)
                while not self.client_map[frame].wait_for_service(timeout_sec=1.0):
                    self.get_logger().info(f'/{ service_name } service not available, waiting again...')

            request = UwbPositionSrv.Request()
            try:
                result = await self.client_map[frame].call_async(request)
            except Exception as e:
                node.get_logger().info('Service call failed %r' % (e,))
            else:
                position.append([result.x, result.y, result.z, result.type])

        self.correction_step(sensor_frames, sensor_distance, position)

    def correction_step(self, sensor_frames, sensor_distance, sensor_pos):
        # updates the belief, i.e., mu and sigma, according to the
        # sensor model
        #
        # The employed sensor model is range-only
        #
        # mu: 3x1 vector representing the mean of the
        #     belief distribution
        # sigma: 3x3 covariance matrix of belief distribution

        x = self.mu[0]
        y = self.mu[1]
        z = self.mu[2]

        #measured landmark ids and ranges
        ids = sensor_frames
        ranges = sensor_distance
        # Compute the expected range measurements for each landmark.
        # This corresponds to the function h
        H = []
        Z = []
        expected_ranges = []

        el_length = 0

        for i in range(len(ids)):
            if sensor_pos[i][3] == 1:
                continue
            lm_id = ids[i]
            meas_range = ranges[i]
            lx = sensor_pos[i][0]
            ly = sensor_pos[i][1]
            lz = sensor_pos[i][2]
            #calculate expected range measurement
            range_exp = np.sqrt( (lx - x)**2 + (ly - y)**2+(lz - z)**2 )
            #compute a row of H for each measurement
            H_i = [(x - lx)/range_exp, (y - ly)/range_exp, 0]
            H.append(H_i)
            Z.append(ranges[i])
            expected_ranges.append(range_exp)
            el_length += 1

        # noise covariance for the measurements
        R = 0.5 * np.eye(el_length)
        # Kalman gain
        K_help = np.linalg.inv(np.dot(np.dot(H, self.sigma), np.transpose(H)) + R)
        K = np.dot(np.dot(self.sigma, np.transpose(H)), K_help)
        # Kalman correction of mean and covariance
        self.mu = self.mu + np.dot(K, (np.array(Z) - np.array(expected_ranges)))
        self.sigma = np.dot(np.eye(len(self.sigma)) - np.dot(K, H), self.sigma)

        return self.mu, self.sigma

    def position_callback(self, request, response):
       response.x = self.mu[0]
       response.y = self.mu[1]
       response.z = self.mu[2]
       response.type = 2

       return response

if __name__ == "__main__":
    rclpy.init()
    node = UwbPosition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()