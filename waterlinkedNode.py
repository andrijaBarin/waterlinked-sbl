#!/usr/bin/env python3

import requests
import rclpy

from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String


class WaterlinkedNode(Node):

    def __init__(self):
        super().__init__('waterlinked_node')

        self.declare_parameter('base_url', 'http://192.168.2.94')
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('frame_id', 'waterlinked')

        self.base_url = self.get_parameter(
            'base_url').get_parameter_value().string_value

        self.update_rate = self.get_parameter(
            'update_rate').get_parameter_value().double_value

        self.frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value

        self.raw_pub = self.create_publisher(
            PointStamped,
            '/waterlinked/acoustic_position/raw',
            10
        )

        self.filtered_pub = self.create_publisher(
            PointStamped,
            '/waterlinked/acoustic_position/filtered',
            10
        )

        self.global_pub = self.create_publisher(
            NavSatFix,
            '/waterlinked/global_position',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/waterlinked/status',
            10
        )

        self.last_raw = None
        self.last_filtered = None

        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.timer_callback
        )

        self.get_logger().info('Waterlinked ROS2 node started')

    def get_data(self, endpoint):

        url = f'{self.base_url}{endpoint}'

        try:
            response = requests.get(url, timeout=0.5)

        except requests.exceptions.RequestException as exc:
            self.publish_status(f'HTTP exception: {exc}')
            return None

        if response.status_code != 200:
            self.publish_status(
                f'HTTP error {response.status_code}: {response.text}'
            )
            return None

        try:
            return response.json()

        except ValueError as exc:
            self.publish_status(f'JSON decode error: {exc}')
            return None

    def get_acoustic_position_raw(self):
        return self.get_data('/api/v1/position/acoustic/raw')

    def get_acoustic_position_filtered(self):
        return self.get_data('/api/v1/position/acoustic/filtered')

    def get_global_position(self):
        return self.get_data('/api/v1/position/global')

    def publish_point(self, publisher, data):

        if data is None:
            return

        if 'x' not in data:
            return

        msg = PointStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.point.x = float(data.get('x', 0.0))
        msg.point.y = float(data.get('y', 0.0))
        msg.point.z = float(data.get('z', 0.0))

        publisher.publish(msg)

    def publish_global(self, data):

        if data is None:
            return

        if data.get('lat') is None or \
           data.get('lon') is None or \
           data.get('z') is None:
            return

        msg = NavSatFix()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.latitude = float(data['lat'])
        msg.longitude = float(data['lon'])
        msg.altitude = float(data['z'])

        self.global_pub.publish(msg)

    def publish_status(self, text):

        msg = String()
        msg.data = text

        self.status_pub.publish(msg)

        self.get_logger().warn(text)

    def position_changed(self, current, previous):

        if current is None:
            return False

        if 'x' not in current or 'y' not in current or 'z' not in current:
            return False

        if previous is None:
            return True

        dist = (
            (current['x'] - previous['x']) ** 2 +
            (current['y'] - previous['y']) ** 2 +
            (current['z'] - previous['z']) ** 2
        )

        return dist > 1e-10

    def timer_callback(self):

        raw_data = self.get_acoustic_position_raw()

        if self.position_changed(raw_data, self.last_raw):
            self.publish_point(self.raw_pub, raw_data)
            self.last_raw = raw_data

        filtered_data = self.get_acoustic_position_filtered()

        if self.position_changed(filtered_data, self.last_filtered):
            self.publish_point(self.filtered_pub, filtered_data)
            self.last_filtered = filtered_data

        global_data = self.get_global_position()

        self.publish_global(global_data)


def main(args=None):

    rclpy.init(args=args)

    node = WaterlinkedNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
