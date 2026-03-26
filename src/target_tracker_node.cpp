#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/point.hpp" // [NEW] Include the Point message type

class TargetTracker : public rclcpp::Node
{
public:
    TargetTracker() : Node("target_tracker")
    {
        // Subscribe to the camera
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, 
            std::bind(&TargetTracker::image_callback, this, std::placeholders::_1));
            
        // [NEW] Create a publisher that broadcasts Point messages to the "/target_centroid" topic
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/target_centroid", 10);
            
        RCLCPP_INFO(this->get_logger(), "Tracking Node Started. Broadcasting to /target_centroid");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;
            cv::Mat hsv_frame, mask;

            cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

            // Filter for Green
            cv::Scalar lower_green(40, 100, 100);
            cv::Scalar upper_green(80, 255, 255);
            cv::inRange(hsv_frame, lower_green, upper_green, mask);

            cv::Moments m = cv::moments(mask);

            if (m.m00 > 1000) 
            {
                int cx = int(m.m10 / m.m00);
                int cy = int(m.m01 / m.m00);

                cv::circle(frame, cv::Point(cx, cy), 10, cv::Scalar(0, 0, 255), -1);

                // [NEW] Package the coordinates into a ROS message and publish it
                geometry_msgs::msg::Point centroid_msg;
                centroid_msg.x = cx;
                centroid_msg.y = cy;
                centroid_msg.z = 0.0; // We are in 2D space on the screen, so Z is 0
                publisher_->publish(centroid_msg);
            }

            cv::imshow("Perception Node: Live Feed", frame);
            cv::waitKey(1); 
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_; // [NEW] Publisher variable
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetTracker>());
    rclcpp::shutdown();
    return 0;
}