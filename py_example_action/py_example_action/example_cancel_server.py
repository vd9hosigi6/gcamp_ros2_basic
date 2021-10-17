# !/usr/bin/env/ python3

# 모듈 불러오기
import rclpy
from rclpy.node import Node

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from custom_interfaces.action import Fibonacci
import time

# 노드 클래스 정의하기
class ExampleActionCancelServerNode(Node):
        # 생성자
        def __init__(self):
                # 노드 이름
                super().__init__("example_action_cacnel_server_node")
                
                self.get_logger().info("ExampleActionCancelServerNode")
                
                # ActionServer 객체 생성하기
                self.action_server = ActionServer(
                        self,
                        Fibonacci,
                        'fibonacci_cancel_action',
                        callback_group=ReentrantCallbackGroup(),
                        execute_callback=self.execute_callback,
                        goal_callback=self.goal_callback,
                        cancel_callback=self.cancel_callback)

        # execute callback
        async def execute_callback(self, goal_handle):
                self.get_logger().info("execute callback")

                # action feedback message
                feedback_msg = Fibonacci.Feedback()
                feedback_msg.partial_sequence = [0, 1]

                # feedback message
                for i in range(1, goal_handle.request.order):
                
                        # cacnel request 검사하기
                        if goal_handle.is_cancel_requested:
                                # 취소하기
                                goal_handle.canceled()
                                self.get_logger().info("goal cancel")
                                return Fibonacci.Result()

                        # feedback msg
                        feedback_msg.partial_sequence.append(
                                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i - 1]
                        )

                        # publish feedback message
                        goal_handle.publish_feedback(feedback_msg)
                        
                        time.sleep(1)

                # send result message
                goal_handle.succeed()
                
                self.get_logger().warn("action send result")

                result = Fibonacci.Result()
                result.sequence = feedback_msg.partial_sequence
                return result

        # goal callback
        def goal_callback(self, goal_request):
                self.get_logger().info("goal callback")
                return GoalResponse.ACCEPT

        # cancel callback               
        def cancel_callback(self, goal_handle):
                self.get_logger().info("cancel callback")
                return CancelResponse.ACCEPT


# main()
def main(args=None):
        # 초기화하기
        rclpy.init(args=args)

        example_action_cancel_server_node = ExampleActionCancelServerNode()

        # MultiThreadExecutor 생성하기
        executor = MultiThreadedExecutor()
        
        # 대기하기
        rclpy.spin(example_action_cancel_server_node, executor=executor)

        # 종료하기
        example_action_cancel_server_node.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
        main()

