#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"



// 노드 정의하기
class ExampleNode : public rclcpp::Node
{
private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_subscriber;

public:
        ExampleNode() : Node("example4_node")
        {
                m_subscriber = create_subscription<sensor_msgs::msg::LaserScan>("skidbot/scan", 10, std::bind(&ExampleNode::subscribe_callback, this, std::placeholders::_1));
        }

        void subscribe_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
                RCLCPP_INFO(get_logger(), "distance from front side : %f", (msg->ranges)[360]);
        }
};

int main(int argc, char **argv)
{
        // 초기화하기
        rclcpp::init(argc, argv);

        // ExampleNode 객체 생성하기
        auto example_node = std::make_shared<ExampleNode>();

        // 실행하기
        rclcpp::spin(example_node);

        // 종료하기
        rclcpp::shutdown();

        return 0;
}

