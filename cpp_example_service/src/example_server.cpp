#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "cpp_example_service/srv/turning_control.hpp"
#include "geometry_msgs/msg/twist.hpp"



// Node 클래스 정의하기
class ExampleServerNode : public rclcpp::Node
{
private:
        rclcpp::Service<cpp_example_service::srv::TurningControl>::SharedPtr m_service;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;

public:
        // 생성자
        ExampleServerNode() : rclcpp::Node("example_server_node")
        {
                RCLCPP_INFO(get_logger(), "ExampleServerNode start");

                // create publisher : skidbot/cmd_vel
                m_publisher = create_publisher<geometry_msgs::msg::Twist>("skidbot/cmd_vel", 10);

                // create service : turn_robot
                m_service = create_service<cpp_example_service::srv::TurningControl>(
                        "turn_robot", std::bind(&ExampleServerNode::response_callback, this, std::placeholders::_1, std::placeholders::_2));
        }

        // response callback
        void response_callback(std::shared_ptr<cpp_example_service::srv::TurningControl::Request> request, std::shared_ptr<cpp_example_service::srv::TurningControl::Response> response)
        {
                RCLCPP_INFO(get_logger(), "response_callback");

                auto start_time = now();
                auto end_time = now();
                auto duration_time = request->time_duration * 1e9;

                while ((end_time - start_time).nanoseconds() < duration_time)
                {
                        move_robot(request->linear_vel_x, request->angular_vel_z);
                        end_time = now();
                        usleep(100000);
                }
                
                stop_robot();

                response->success = true;
        }

        // move robot
        void move_robot(const float linear_vel_x, const float angular_vel_z)
        {
                RCLCPP_INFO(get_logger(), "move_robot");
                geometry_msgs::msg::Twist twist_msg;

                twist_msg.linear.x = linear_vel_x;
                twist_msg.angular.z = angular_vel_z;

                m_publisher->publish(twist_msg);
        }

        // stop robot
        void stop_robot(void)
        {
                RCLCPP_INFO(get_logger(), "stop robot");
                geometry_msgs::msg::Twist twist_msg;

                twist_msg.linear.x = 0.0;
                twist_msg.angular.z = 0.0;

                m_publisher->publish(twist_msg);
        }
};



/*
 *
 */
int main(int argc, char **argv)
{
        // 초기화하기
        rclcpp::init(argc, argv);

        // 노드 생성하기
        auto example_server_node = std::make_shared<ExampleServerNode>();

        // 대기하기
        rclcpp::spin(example_server_node);

        // 종료하기
        rclcpp::shutdown();

        return 0;
}

