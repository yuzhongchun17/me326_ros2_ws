#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStateRelay : public rclcpp::Node
{
public:
    JointStateRelay()
    : Node("joint_state_relay")
    {
        // Initialize subscriber to the /locobot/joint_states topic
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/locobot/joint_states", 10,
            std::bind(&JointStateRelay::joint_state_callback, this, std::placeholders::_1));

        // Initialize publisher to the /joint_states topic
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const
    {
        // Simply publish the received message to /joint_states
        publisher_->publish(*msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStateRelay>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
