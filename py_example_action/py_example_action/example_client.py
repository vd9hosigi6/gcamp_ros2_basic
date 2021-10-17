# !/usr/bin/env/ python3

# 모듈 불러오기
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from custom_interfaces.action import Fibonacci



# 노드 정의하기
class ExampleActionClientNode(Node):
        # 생성자
        def __init__(self):
                # 노드 이름
                super().__init__("example_action_client_node")

                self.get_logger().info("ExampleActionClient constructor")

                # action client 객체 생성하기
                self.action_client = ActionClient(self, Fibonacci, "fibonacci_action")
        
        # send goal request
        def sendGoalRequest(self, order):
        
                # goal request message
                goal_msg = Fibonacci.Goal()
                goal_msg.order = order

                # action server 가 대기 상태가 될 때까지 대기하기
                while not self.action_client.wait_for_server(1):
                        self.get_logger().info("server is busy")
                        
                # goal 메시지 전송하고 feedback 콜백함수 설정하기
                self.goal_future = self.action_client.send_goal_async(
                        goal_msg, feedback_callback=self.feedback_callback
                )
                
                # goal response callback 설정하기
                self.goal_future.add_done_callback(self.goal_response_callback)
                
        # goal response callback
        def goal_response_callback(self, future):
                # goal response
                goal_handle = future.result()

                # goal response 검사하기
                if not goal_handle.accepted:
                        self.get_logger().info("goal rejected")
                        return
                else:
                        self.get_logger().info("goal accepted")

                # result callback 설정하기
                self.result_future = goal_handle.get_result_async()
                self.result_future.add_done_callback(self.get_result_callback)

        # feedback callback
        def feedback_callback(self, feedback_msg):
                feedback = feedback_msg.feedback
                self.get_logger().info(f"action feedback callback : {feedback.partial_sequence}")

        # result callback
        def get_result_callback(self, future):
                result = future.result().result
                self.get_logger().info(f"action result callback :{result.sequence}")

                # 종료하기
                rclpy.shutdown()


def main(args=None):
        # 초기화하기
        rclpy.init(args=args)

        # Action Client Node 생성하기
        example_action_client_node = ExampleActionClientNode()
        
        # send goal request
        future = example_action_client_node.sendGoalRequest(5)

        # 대기하기
        rclpy.spin(example_action_client_node)


if __name__ == "__main__":
        main()

