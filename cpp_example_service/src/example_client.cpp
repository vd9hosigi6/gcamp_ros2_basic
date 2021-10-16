#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cpp_example_service/srv/turning_control.hpp"



using namespace std::chrono_literals;

// 노드 생성하기
class ExampleClientNode : public rclcpp::Node
{
private:
        rclcpp::Client<cpp_example_service::srv::TurningControl>::SharedPtr m_client;

public:
        // 생성자
        ExampleClientNode() : rclcpp::Node("example_client_node")
        {

                // Service Client 생성하기
                m_client = create_client<cpp_example_service::srv::TurningControl>("turn_robot");

                //Service Server가 실행중인 경우 대기하기
                while (!m_client->wait_for_service(1s))
                {
                        RCLCPP_INFO(get_logger(), "service server ('turn_robot') is busy");
                }
        }

        // request 보내기
        auto get_result_future(unsigned int time_duration, float linear_vel_x, float angular_vel_z)
        {
                std::shared_ptr<cpp_example_service::srv::TurningControl::Request> request = std::make_shared<cpp_example_service::srv::TurningControl::Request>();

                request->time_duration = time_duration;
                request->linear_vel_x = linear_vel_x;
                request->angular_vel_z = angular_vel_z;

                return m_client->async_send_request(request);    
        }
};



int main(int argc, char **argv)
{
        // 초기화하기
        rclcpp::init(argc, argv);

        // 노드 생성하기
        auto example_client_node = std::make_shared<ExampleClientNode>();

        // request 보내기
        auto future = example_client_node->get_result_future(5, 0.5, 1.0);

        // 대기하기
        auto result = rclcpp::spin_until_future_complete(example_client_node, future);

        if( result == rclcpp::executor::FutureReturnCode::SUCCESS )
        {
                RCLCPP_INFO(example_client_node->get_logger(), "Result : %d", future.get()->success);
        }
        else
        {
                RCLCPP_INFO(example_client_node->get_logger(), "Fail service client");
        }

        // 종료하기
        rclcpp::shutdown();

        return 0;
}

