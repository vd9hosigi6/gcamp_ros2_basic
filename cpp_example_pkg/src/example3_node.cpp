#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"



// Node 클래스 정의하기
class ExamplePublishNode : public rclcpp::Node
{
private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;
        rclcpp::TimerBase::SharedPtr m_timer;




        // timer callback 멤버 함수 정의하기
        void timer_callback(void)
        {
                RCLCPP_INFO(this->get_logger(), "timer callback");
                move_robot();
        }

public:
        ExamplePublishNode() : Node("example_publish_node")
        {
                // publisher 객체 생성하기
                m_publisher = create_publisher<geometry_msgs::msg::Twist>("skidbot/cmd_vel", 10);

                // 타이머 생성하기
                m_timer = create_wall_timer(std::chrono::milliseconds(400), std::bind(&ExamplePublishNode::timer_callback, this));
        }

        void move_robot(void)
        {
                geometry_msgs::msg::Twist twist_msg;



                twist_msg.linear.x = 0.5;
                twist_msg.angular.z = 2.0;

                m_publisher->publish(twist_msg);
        }

        void stop_robot(void)
        {
                geometry_msgs::msg::Twist twist_msg;
                


                twist_msg.linear.x = 0;
                twist_msg.angular.z = 0;

                m_publisher->publish(twist_msg);
        }
};




int main(int argc, char **argv)
{
        // 초기화하기
        rclcpp::init(argc, argv);


        // 노드 생성하기
        auto example_node = std::make_shared<ExamplePublishNode>();

        // 지연 시간 설정하기 (2 Hz)
        rclcpp::WallRate rate(2);

        // 시간 계산하기
        auto start_time = example_node->now();
        auto end_time = example_node->now();

        // 메인 루프
        while ( (end_time - start_time).seconds() < 5 )
        {
                rclcpp::spin_some(example_node);
                rate.sleep();

                end_time = example_node->now();

                RCLCPP_INFO(example_node->get_logger(), "main loop");
        }

        example_node->stop_robot();

        // 종료하기
        rclcpp::shutdown();

        return 0;
} 
