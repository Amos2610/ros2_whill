#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRelayNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay_node')

        # サブスクライバー: /cmd_vel_nav を購読
        self.subscriber = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.cmd_vel_callback,
            10
        )

        # パブリッシャー: /whill/controller/cmd_vel に転送
        self.publisher = self.create_publisher(
            Twist,
            '/whill/controller/cmd_vel',
            10
        )

    def cmd_vel_callback(self, msg):
        # メッセージをそのまま転送
        self.publisher.publish(msg)
        self.get_logger().info(f"Relayed Twist: linear.x={msg.linear.x}, angular.z={msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
