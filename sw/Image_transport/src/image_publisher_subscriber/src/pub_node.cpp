/* maked by Mohamad Hamdi Alhaji Shommo*/
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImagePublisher : public rclcpp::Node {
public:
    ImagePublisher() : Node("image_publisher") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&ImagePublisher::publish_image, this));
    }

private:
    void publish_image() {
        cv::Mat img = cv::imread("/root/momosit.jpg");   // Path to your image
        if (img.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open or find the image");
            return;
        }

        // Förväntad storlek från YOLO (t.ex. 480x640)
        int target_width = 480;
        int target_height = 640;

        // Hämta originalstorleken på bilden
        int original_width = img.cols;
        int original_height = img.rows;

        // Beräkna skalfaktorn för att bibehålla proportionerna
        double scale = std::min(static_cast<double>(target_width) / original_width,
                                static_cast<double>(target_height) / original_height);

        // Resiza proportionellt
        int new_width = static_cast<int>(original_width * scale);
        int new_height = static_cast<int>(original_height * scale);
        cv::Mat resized_img;
        cv::resize(img, resized_img, cv::Size(new_width, new_height));

        // Lägg till utfyllnad om nödvändigt för att matcha YOLO:s förväntade storlek
        int top = (target_height - new_height) / 2;
        int bottom = target_height - new_height - top;
        int left = (target_width - new_width) / 2;
        int right = target_width - new_width - left;
        cv::Mat padded_img;
        cv::copyMakeBorder(resized_img, padded_img, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        // Konvertera och publicera bilden
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", padded_img).toImageMsg();
        publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Image published with size: %d x %d", padded_img.cols, padded_img.rows);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}

