#! /usr/bin/env python3

# 모듈 불러오기
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import TurningControl



# Node 클래스 정의하기
class ExampleClientNode(Node):
        # 생성자
        def __init__(self):
                super().__init__("example_client_node")

                # Service 클라이언트 생성하기
                self.client = self.create_client(TurningControl, "example_service")

                # 기존 실행 중인 서비스 대기하기
                while not self.client.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info("example_service is busy")

        # request 보내기
        def send_request(self):
                self.get_logger().info("send request")
                # time_duration, linear_vel_x, angular_vel_z 항목
                request = TurningControl.Request()

                request.time_duration = 5
                request.linear_vel_x = 0.5
                request.angular_vel_z = 1.0

                # 비동기로 호출하기
                self.future = self.client.call_async(request)

                return self.future

# main 함수
def main(args=None):
        # 초기화하기
        rclpy.init(args=args)

        # 노드 생성하기
        example_client_node = ExampleClientNode()

        # Service Server 노드로 보내기
        future = example_client_node.send_request()

        # 대기하기
        rclpy.spin_until_future_complete(example_client_node, future)

        if future.done():
                try:
                        response = future.result()
                except Exception:
                        raise RuntimeError("exception service : %r" %future.exception())
                else:
                        example_client_node.get_logger().info(f"result : {response}")
                finally:
                        example_client_node.destroy_node()
                        rclpy.shutdown()

if __name__ == "__main__":
        main()

