#! /usr/bin/env python3

# 모듈 불러오기
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist



# CmdVelPublisher
class CmdVelNode(Node):
        # 생성자
        def __init__(self):
                # 부모 Node 생성자 호출하여 Node 이름 설정하기
                super().__init__("cmd_vel_node")

                # publisher 객체 생성하기
                # 메시지 데이터형 : Twist
                # topic 이름 : skidbot/cmd_vel
                # queue size : 10
                self.publisher = self.create_publisher(
                        Twist, "skidbot/cmd_vel", 10
                )

                # 타이머 생성하기
                self.timer = self.create_timer(0.5, self.publish_callback)

        # publish callback
        def publish_callback(self):
                twist_msg = Twist()
                twist_msg.linear.x = 0.5
                twist_msg.angular.z = 1.0
                self.publisher.publish(twist_msg)
                
                self.get_logger().info("publish_callback")

        # stop robot
        def stop_robot(self):
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.linear.y = 0.0
                twist_msg.linear.z = 0.0


                twist_msg.angular.x = 0.0
                twist_msg.angular.y = 0.0
                twist_msg.angular.z = 0.0

                self.publisher.publish(twist_msg)

# main 함수
def main():
        # rclpy 초기화하기
        rclpy.init()

        # Node 생성하기
        cmd_vel_node = CmdVelNode()

        # publish 호출하기
        start_time = cmd_vel_node.get_clock().now().to_msg().sec
        end_time = cmd_vel_node.get_clock().now().to_msg().sec

        while (end_time - start_time < 5):
                rclpy.spin_once(cmd_vel_node)
                end_time = cmd_vel_node.get_clock().now().to_msg().sec

                cmd_vel_node.get_logger().info("cmd_vel_node spin_once\n")

        # 종료하기
        cmd_vel_node.get_logger().info("cmd_vel_node destroy_node\m")
        cmd_vel_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
        main()

