# !/usr/bin/env/ python3

# 모듈 불러오기
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from custom_interfaces.action import Fibonacci

# 노드 정의하기
class ExampleActionCancelClientNode(Node):
        # 생성자
        def __init__(self):
                # 노드 이름 정하기
                super().__init__("example_action_cancel_client_node")
                
                self.get_logger().info("ExampleActionCancelClientNode constructor")
                
                # ActionClient 객체 생성하기
                self.action_client = ActionClient(self, Fibonacci, "fibonacci_cancel_action")

                # goal_handle 멤버 변수 초기화하기
                self.goal_handle = None

        # send goal request
        def sendGoalRequest(self, order):
                # action goal message
                goal_msg = Fibonacci.Goal()
                goal_msg.order = order

                # action server 가 실행주이면 대기하기
                while not self.action_client.wait_for_server(1):
                        self.get_logger().info("action server is busy")

                # send goal request
                self.goal_future = self.action_client.send_goal_async(
                        goal_msg, feedback_callback=self.feedback_callback
                )

                # goal response callback 설정하기
                self.goal_future.add_done_callback(self.goal_response_callback)
        
        # goal response callback
        def goal_response_callback(self, future):
                # response 결과 얻기
                self.goal_handle = future.result()

                # response 결과 검사하기
                if not self.goal_handle.accepted:
                        self.get_logger().info("goal response rejected")
                        return
                else:           
                        self.get_logger().info("goal response accepted")

                # 2초 후 cancel을 요청하는 타이머 콜백함수 호출하기
                # 여기에서는 feeedback.partial_sequence 개수를 검사하여 cancel 요청함
                # self.timer= self.create_timer(2.0, self.timer_callback)

                # result response callback
                self.result_future = self.goal_handle.get_result_async()
                self.result_future.add_done_callback(self.get_result_callback)

        # feedback callback
        def feedback_callback(self, feedback_msg):
                feedback = feedback_msg.feedback
                self.get_logger().info(f"action feedback message: {feedback.partial_sequence}")

                if len(feedback.partial_sequence) > 4:
                        self.get_logger().info("send cancel request")
                        self.sendCancelRequest()

        # timer callback
        def timer_callback(self):
                self.get_logger().info("timer callback")

                # cancel 요청하기
                self.sendCancelRequest()

                # 타이머 중지하기
                self.timer.cancel()

        # cancel 요청하기
        def sendCancelRequest(self):
                self.get_logger().info("sendCancelRequest")
                
                if self.goal_handle:
                        future = self.goal_handle.cancel_goal_async()
                        future.add_done_callback(self.cancel_callback)

        # cancel_callback
        def cancel_callback(self, future):
                # canel 결과 검사하기
                cancel_response = future.result()
                
                if len(cancel_response.goals_canceling) > 0:
                        self.get_logger().info("successfully canceled")
                else:
                        self.get_logger().info("failed to cancel")

                # 종료하기
                rclpy.shutdown()

        # result callback
        def get_result_callback(self, future):
                result = future.result().result
                self.get_logger().info(f"action result callback :{result.sequence}")

                # 종료하기
                rclpy.shutdown()


# main()
def main(args=None):
        # 초기화하기
        rclpy.init(args=args)

        # 노드 객체 생성하기
        example_action_cancel_client_node = ExampleActionCancelClientNode()

        future = example_action_cancel_client_node.sendGoalRequest(10)

        # 대기하기
        rclpy.spin(example_action_cancel_client_node)


if __name__ == "__main__":
        main()

