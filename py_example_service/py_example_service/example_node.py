#! /usr/bin/env python3

# 모듈불러오기
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity

from ament_index_python.packages import get_package_share_directory
import os



# Service Client Node 정의하기
class ExampleNode(Node):
        # 생성자
        def __init__(self) : 
                # Node 생성자 호출하기 (Node 이름)
                super().__init__("service_example_node")

                # client service
                self.client = self.create_client(SpawnEntity, "/spawn_entity")

                # service 가 유휴 상태가 될 때까지 기다리기
                while not self.client.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info("/spawn_entity service is busy")
                self.get_logger().info("/spwan_enttity is not busy")

        # request 보내기
        def sendRequest(self):
                # read skidbot2 urdf
                xml_file = os.path.join(
                        get_package_share_directory("gcamp_gazebo"),
                        "urdf",
                        "skidbot2.urdf",
                )
                self.get_logger().info("xml path : %s" %xml_file)
                xml_data = open(xml_file, "r").read()

                # request
                request = SpawnEntity.Request()

                request.name = "skidbot2"
                request.xml = xml_data
                request.robot_namespace = "skidbot2"
                request.initial_pose.position.x = 1.0
                request.initial_pose.position.y = 1.0
                request.initial_pose.position.z = 0.3

                # service call
                self.future = self.client.call_async(request)

                return self.future

# main
def main(args=None):
        # 초기화하기
        rclpy.init(args=args)

        # 노드 생성하기
        example_node = ExampleNode()

        # request 보내기
        future = example_node.sendRequest()
        
        # 대기하기
        rclpy.spin_until_future_complete(example_node, future)

        if future.done():
                try:
                        response = future.result()
        
                except Exception:
                        raise RuntimeError("exception service call: %r" %future.exeeption())
        
                else:
                        example_node.get_logger().info(f"result message {response.status_message}")
        
                finally:
                        example_node.get_logger().info("exit example-node")
        
                        example_node.destroy_node()
                        rclpy.shutdown()

if __name__ == "__main__":
        main()

