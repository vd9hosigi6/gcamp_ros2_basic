#!/usr/bin/env/ python3

# 모듈 불러오기
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node

from custom_interfaces.action import Fibonacci
import time



# 노드 정의하기
class ExampleActionServerNode(Node):
        # 생성자
        def __init__(self):
                # 노드 이름
                super().__init__('example_action_node')

                self.get_logger().info("Action Node constructor")
                
                # ActionServer 객체 생성하기
                self.action_server = ActionServer(
                        self,
                        Fibonacci,
                        "fibonacci_action",
                        self.execute_callback,
                        goal_callback=self.goal_callback,
                )
                
        # goal_callback
        def goal_callback(self, goal_request):
                self.get_logger().info("goal callback")
                return GoalResponse.ACCEPT

        # callback 함수
        async def execute_callback(self, goal_handle):
                self.get_logger().info("execute callback.")

                # feedback action message
                feedback_msg = Fibonacci.Feedback()
                feedback_msg.partial_sequence = [0, 1]

                
                # goal action message
                # goal_handle.request.order
                for i in range(1, goal_handle.request.order):

                        # cancel request 검사하기
                        if goal_handle.is_cancel_requested:
                                # cancel 처리하기
                                self.get_logger().info("goal cancel")
                                
                                goal_handle.canceled()
                                return Fibonacci.Result()

                        # feedback message 에 데이터 추가하기
                        feedback_msg.partial_sequence.append(
                                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i - 1]
                        )

                        # feedback 메시지 보내기
                        goal_handle.publish_feedback(feedback_msg)
                        time.sleep(1)

                # 
                goal_handle.succeed()

                # action result message
                self.get_logger().info("result message")
                result = Fibonacci.Result()
                result.sequence = feedback_msg.partial_sequence
                
                return result

# main 함수
def main(args=None):
        # 초기화하기
        rclpy.init(args=args)

        # Action Server 노드 생성하기
        example_action_server_node = ExampleActionServerNode()
        
        # 대기하기
        rclpy.spin(example_action_server_node)

        # 종료하기
        example_action_server_node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
        main()
