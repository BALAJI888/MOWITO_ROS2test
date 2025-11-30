#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class ImageConversionNode : public rclcpp::Node
{
public:
    ImageConversionNode() : Node("image_conversion_node"), mode_(false) // false = Color, true = Grayscale
    {
        // Declare parameters
        this->declare_parameter<std::string>("input_topic", "/image_raw");
        this->declare_parameter<std::string>("output_topic", "/converted_image");

        // Get parameter values
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();

        // Create subscription to input image topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic, 10,
            std::bind(&ImageConversionNode::image_callback, this, std::placeholders::_1));

        // Create publisher for output image
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic, 10);

        // Create service for mode change
        service_ = this->create_service<std_srvs::srv::SetBool>(
            "set_mode",
            std::bind(&ImageConversionNode::set_mode_callback, this,
                     std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Image conversion node started");
        RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Current mode: %s", mode_ ? "Grayscale" : "Color");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // Convert ROS image message to OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            
            cv::Mat output_image;
            
            if (mode_) {
                // Convert to grayscale
                if (msg->encoding == "bgr8" || msg->encoding == "rgb8") {
                    cv::cvtColor(cv_ptr->image, output_image, cv::COLOR_BGR2GRAY);
                } else {
                    // If already grayscale or different format, use as is
                    output_image = cv_ptr->image;
                }
            } else {
                // Keep as color
                output_image = cv_ptr->image;
            }

            // Convert back to ROS image message
            std::string output_encoding = mode_ ? "mono8" : msg->encoding;
            cv_bridge::CvImage output_cv_image(msg->header, output_encoding, output_image);
            
            // Publish the converted image
            publisher_->publish(*output_cv_image.toImageMsg());

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        } catch (cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
            return;
        }
    }

    void set_mode_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        mode_ = request->data;
        
        if (mode_) {
            RCLCPP_INFO(this->get_logger(), "Mode changed to: Grayscale");
        } else {
            RCLCPP_INFO(this->get_logger(), "Mode changed to: Color");
        }
        
        response->success = true;
        response->message = mode_ ? "Mode set to Grayscale" : "Mode set to Color";
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    bool mode_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageConversionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
