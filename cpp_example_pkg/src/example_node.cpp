#include "rclcpp/rclcpp.hpp"



int main(int argc, char **argv)
{
        // 초기화하기
        rclcpp::init(argc, argv);

        // 노드 생성하기
        auto example_node = rclcpp::Node::make_shared("example_node");

        // 주기 설정하기
        rclcpp::WallRate rate(2);

        //
        while (rclcpp::ok())
        {
                rclcpp::spin_some(example_node);
                rate.sleep();

                RCLCPP_INFO(example_node->get_logger(), "example node log");
        }

        rclcpp::shutdown();

        return 0;
}

