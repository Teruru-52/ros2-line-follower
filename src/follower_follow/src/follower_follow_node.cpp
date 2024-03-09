#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

using Float64 = std_msgs::msg::Float64;
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
        data_sub = this->create_subscription<Float64>(
            "/sub_data", qos, [this](const Float64::SharedPtr msg)
            { data = *msg; });
        // publisher
        data_pub = this->create_publisher<Float64>("/pub_data", qos);
    }

private:
    void TimerCallback()
    {
        data_pub->publish(data);
    }

    int loop_ms{1000};
    Float64 data;

    // timer
    rclcpp::TimerBase::SharedPtr timer;
    // subscriber
    rclcpp::Subscription<Float64>::SharedPtr data_sub;
    // publisher
    rclcpp::Publisher<Float64>::SharedPtr data_pub;
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