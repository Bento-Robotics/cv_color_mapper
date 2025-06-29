#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"

class ColorMapNode : public rclcpp::Node
{
  public:
    ColorMapNode() : Node("color_map_node"){};
    ~ColorMapNode() {
      sub_->shutdown();
      pub_->shutdown();
    }

  public:
    void InitializeImageTransport() {
      it_ = std::make_shared<image_transport::ImageTransport>(
          this->shared_from_this()); // https://answers.ros.org/question/353828/getting-a-nodesharedptr-from-this/
      sub_ = std::make_shared<image_transport::Subscriber>(
          it_->subscribe("input_image", 1,
            std::bind(&ColorMapNode::image_callback, this,
            std::placeholders::_1)));
      pub_ = std::make_shared<image_transport::Publisher>(
          it_->advertise("output_image", 1));
    }
 

    void image_callback (const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
      // Convert ROS image to OpenCV format
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
      }
      catch (cv_bridge::Exception& e)
      {
        auto logger = rclcpp::get_logger("color_map_node");
        RCLCPP_ERROR(logger, "cv_bridge exception: %s", e.what());
        return;
      }

      // Apply OpenCV color map
      cv::Mat color_mapped_image;
      cv::convertScaleAbs(cv_ptr->image, color_mapped_image, 40, -270); // alpha=contrast, beta=brightness
      cv::resize(color_mapped_image, color_mapped_image, cv::Size(320, 240), 0, 0, CV_INTER_LANCZOS4);
      cv::applyColorMap(color_mapped_image, color_mapped_image, cv::COLORMAP_INFERNO);

      // Convert back to ROS image format
      sensor_msgs::msg::Image::SharedPtr output_msg = cv_bridge::CvImage(
          msg->header, // Use the same header as the input
          sensor_msgs::image_encodings::BGR8, // Change encoding to BGR
          color_mapped_image
          ).toImageMsg();

      // Publish the resulting image
      pub_->publish(output_msg);
    }

  private:
    std::shared_ptr<image_transport::ImageTransport> it_;
    std::shared_ptr<image_transport::Subscriber> sub_;
    std::shared_ptr<image_transport::Publisher> pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ColorMapNode>();
  node->InitializeImageTransport();

  rclcpp::spin(node);
  rclcpp::shutdown();
  rclcpp::spin_some(node); // better crash than segfault
  return 0;
}
