#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <follower_msgs/msg/input.hpp>

using Float64 = std_msgs::msg::Float64;
using Input = follower_msgs::msg::Input;
// using std::placeholders::_1;

class FollowerFollow : public rclcpp::Node
{
public:
    FollowerFollow(std::string node_name, rclcpp::NodeOptions options)
        : Node(node_name, options)
    {
        // timer
        timer = this->create_wall_timer(std::chrono::milliseconds(loop_ms),
                                        std::bind(&FollowerFollow::TimerCallback, this));

        // subscriber
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        input_sub = this->create_subscription<Input>(
            "/sub_input", qos, [this](const Input::SharedPtr msg)
            { input = *msg; });
        // publisher
        input_pub = this->create_publisher<Input>("/input", qos);
    }

private:
    void TimerCallback()
    {
        input_pub->publish(input);
    }

    int loop_ms{1000};
    Input input;

    // timer
    rclcpp::TimerBase::SharedPtr timer;
    // subscriber
    rclcpp::Subscription<Input>::SharedPtr input_sub;
    // publisher
    rclcpp::Publisher<Input>::SharedPtr input_pub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node_options = rclcpp::NodeOptions();
    auto node = std::make_shared<FollowerFollow>("follower_follow_node", node_options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}