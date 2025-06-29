#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class ColorMapNode : public rclcpp::Node
{
public:
    ColorMapNode() : Node("color_map_node")
    {
        // Subscriber to the black and white image topic
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "input_image", 10, std::bind(&ColorMapNode::image_callback, this, std::placeholders::_1));

        // Publisher for the colored image topic
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("output_image", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS image to OpenCV format
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Apply OpenCV color map
        cv::Mat colored_image;
        cv::applyColorMap(cv_ptr->image, colored_image, cv::COLORMAP_JET); // You can change the colormap here

        // Convert back to ROS image format
        cv_bridge::CvImage output_msg;
        output_msg.header = msg->header; // Use the same header as the input
        output_msg.encoding = sensor_msgs::image_encodings::BGR8; // Change encoding to BGR
        output_msg.image = colored_image;

        // Publish the colored image
        image_publisher_->publish(output_msg.toImageMsg());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorMapNode>());
    rclcpp::shutdown();
    return 0;
}
