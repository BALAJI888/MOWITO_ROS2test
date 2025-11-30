#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class DummyUsbCamNode : public rclcpp::Node
{
public:
    DummyUsbCamNode() : Node("dummy_usb_cam_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("video_path", "");
        this->declare_parameter<std::string>("output_topic", "/image_raw");
        this->declare_parameter<int>("frame_rate", 30);

        // Get parameter values
        std::string video_path = this->get_parameter("video_path").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        int frame_rate = this->get_parameter("frame_rate").as_int();

        // Create publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic, 10);

        // Initialize video capture
        if (!video_path.empty()) {
            cap_.open(video_path);
            RCLCPP_INFO(this->get_logger(), "Using video file: %s", video_path.c_str());
        } else {
            cap_.open(0); // Default camera
            RCLCPP_INFO(this->get_logger(), "Using default camera");
        }

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open video source");
            return;
        }

        // Create timer for publishing frames
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / frame_rate),
            std::bind(&DummyUsbCamNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Dummy USB camera node started");
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Frame rate: %d Hz", frame_rate);
    }

private:
    void timer_callback()
    {
        if (!cap_.isOpened()) {
            return;
        }

        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            // If video file ended, restart from beginning
            if (!video_path_.empty()) {
                cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
                cap_ >> frame;
            }
            if (frame.empty()) {
                return;
            }
        }

        // Convert OpenCV image to ROS message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_frame";

        // Publish the image
        publisher_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    std::string video_path_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DummyUsbCamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
