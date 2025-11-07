#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CamToImageNode : public rclcpp::Node {
   public:
    CamToImageNode() : Node("cam_two_image_node") {
        // define a publisher
        publisher_ =
            this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

        // open camera by cv
        cap_.open("/dev/video2", cv::CAP_V4L2);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the camera");
            rclcpp::shutdown();
            return;
        }

        // Set capture properties (optional)
        // cap_.set(cv::CAP_PROP_FRAME_WIDTH, 320);
        // cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&CamToImageNode::timer_callback, this));
    }

   private:
    // main callback func
    void timer_callback() {
        cv::Mat frame;
        if (!cap_.read(frame)) {
            RCLCPP_WARN(this->get_logger(), "Failed to capture frame");
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
                       .toImageMsg();
        publisher_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CamToImageNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
