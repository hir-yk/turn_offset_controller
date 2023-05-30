#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from autoware_auto_vehicle_msgs.msg import TurnIndicatorsReport, SteeringReport
from tier4_planning_msgs.msg import LateralOffset

class TurnOffsetController(Node):

    def __init__(self):
        super().__init__('turn_report_subscriber')
        self.subscription = self.create_subscription(
            TurnIndicatorsReport,
            '/vehicle/status/turn_indicators_status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.steering_status_subscription = self.create_subscription(  # ステアリング状態のサブスクリプションを作成
            SteeringReport,
            '/vehicle/status/steering_status',
            self.steering_status_callback,
            10)        

        self.publisher_ = self.create_publisher(LateralOffset, '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/input/lateral_offset', 10)

        self.prev_state = 1
        self.in_intersection = False  # 交差点内であるかを保存する変数
        
    def listener_callback(self, msg):
        self.get_logger().debug('I heard: "%d"' % msg.report)

        offset_msg = LateralOffset()

        if msg.report == self.prev_state:
            return

        if msg.report == 1 or self.in_intersection:  # 交差点にいる場合も横方向オフセットをゼロに設定
            offset_msg.lateral_offset = 0.0
        elif msg.report == 2:
            offset_msg.lateral_offset = 0.3
        elif msg.report == 3:
            offset_msg.lateral_offset = -0.3

        self.prev_state = msg.report

        self.publisher_.publish(offset_msg)
        self.get_logger().info('turn_signal_report: "%d"' % msg.report)
        self.get_logger().info('publishing_offset: "%f"' % offset_msg.lateral_offset)

    def steering_status_callback(self, msg):
        self.get_logger().debug('Steering tire angle: "%f"' % msg.steering_tire_angle)
        self.new_intersection_status = abs(msg.steering_tire_angle) >= 0.04  # ステアリング角度が2.0度以上であるかどうか
        if( self.new_intersection_status != self.in_intersection ):
            self.in_intersection = self.new_intersection_status
            if self.in_intersection:
                offset_msg = LateralOffset()
                offset_msg.lateral_offset = 0.0
                self.publisher_.publish(offset_msg)
                self.get_logger().info('In intersection, setting offset to 0.0')
            
def main(args=None):
    rclpy.init(args=args)

    my_subscriber = TurnOffsetController()

    rclpy.spin(my_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
