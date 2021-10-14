#include "rclcpp/rclcpp.hpp"



/*
 *
 */
class ExampleNode : public rclcpp::Node
{
private:
        rclcpp::TimerBase::SharedPtr m_timer;

        void timer_callback(void)
        {
                static size_t count = 0;



                RCLCPP_INFO(this->get_logger(), "timer : %u", count);
                count = count + 1;
        }

public:
        ExampleNode() : Node("example2_node")
        {
                m_timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ExampleNode::timer_callback, this));
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
        auto example_node = std::make_shared<ExampleNode>();

        // 실행하기
        rclcpp::spin(example_node);

        // 종료하기
        rclcpp::shutdown();

        return 0;
}

