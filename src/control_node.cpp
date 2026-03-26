#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TargetFollower : public rclcpp::Node
{
public:
    TargetFollower() : Node("target_follower")
    {
        // 1. Listen to the coordinates from your perception node
        subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/target_centroid", 10,
            std::bind(&TargetFollower::centroid_callback, this, std::placeholders::_1));

        // 2. Publish movement commands to the simulated robot
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Control Node Started. Waiting for target...");
    }

private:
    void centroid_callback(const geometry_msgs::msg::Point::SharedPtr msg) const
    {
        geometry_msgs::msg::Twist twist_msg;

        // Assuming a standard 640x480 webcam resolution, the center X is 320.
        double center_x = 320.0;
        double error_x = center_x - msg->x;

        // Calculate rotation speed (Proportional Control)
        twist_msg.angular.z = error_x * 0.005;

        // Drive forward at a constant, slow speed as long as we see the target
        twist_msg.linear.x = 0.5;

        // Send the movement command to the robot
        publisher_->publish(twist_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetFollower>());
    rclcpp::shutdown();
    return 0;
}