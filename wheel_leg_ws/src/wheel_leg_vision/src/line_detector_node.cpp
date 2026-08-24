/**
 * @file line_detector_node.cpp
 * @brief Detects colored tape lines on the floor using HSV thresholding.
 *
 * Subscribes: /image_left_raw (sensor_msgs/Image, BGR8)
 * Publishes:  /wheel_leg/line_state (wheel_leg_msgs/LineState)
 *
 * Strategy:
 *   1. Convert BGR -> HSV
 *   2. Apply threshold for the tape color
 *   3. Find contours in the lower half of the image (ROI)
 *   4. Compute line center offset and angle
 *   5. Publish normalized lateral/angular errors
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <wheel_leg_msgs/msg/line_state.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class LineDetectorNode : public rclcpp::Node
{
public:
  LineDetectorNode()
    : Node("line_detector")
  {
    /* Parameters */
    this->declare_parameter("color", "red");
    this->declare_parameter("hsv_lower", std::vector<int64_t>({0, 100, 100}));
    this->declare_parameter("hsv_upper", std::vector<int64_t>({10, 255, 255}));
    this->declare_parameter("roi_top", 0.4);
    this->declare_parameter("roi_bottom", 1.0);
    this->declare_parameter("min_contour_area", 200);
    this->declare_parameter("publish_debug_img", true);

    color_ = this->get_parameter("color").as_string();
    auto lower = this->get_parameter("hsv_lower").as_integer_array();
    auto upper = this->get_parameter("hsv_upper").as_integer_array();
    hsv_lower_ = cv::Scalar(lower[0], lower[1], lower[2]);
    hsv_upper_ = cv::Scalar(upper[0], upper[1], upper[2]);
    roi_top_ = this->get_parameter("roi_top").as_double();
    roi_bottom_ = this->get_parameter("roi_bottom").as_double();
    min_area_ = this->get_parameter("min_contour_area").as_int();
    publish_debug_ = this->get_parameter("publish_debug_img").as_bool();

    /* Subscribers */
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_left_raw", 10,
      std::bind(&LineDetectorNode::image_callback, this, std::placeholders::_1));

    /* Publishers */
    line_pub_ = this->create_publisher<wheel_leg_msgs::msg::LineState>(
      "/wheel_leg/line_state", 10);

    if (publish_debug_) {
      debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/line_detector/debug_img", 10);
    }

    RCLCPP_INFO(this->get_logger(), "LineDetector started (color=%s)", color_.c_str());
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat frame = cv_ptr->image;
    int h = frame.rows, w = frame.cols;

    /* Define ROI: bottom portion of image (where the floor/line is visible) */
    int roi_y_start = static_cast<int>(h * roi_top_);
    int roi_y_end   = static_cast<int>(h * roi_bottom_);
    cv::Rect roi(0, roi_y_start, w, roi_y_end - roi_y_start);
    cv::Mat roi_frame = frame(roi);

    /* HSV threshold */
    cv::Mat hsv, mask;
    cv::cvtColor(roi_frame, hsv, cv::COLOR_BGR2HSV);

    if (color_ == "red") {
      /* Red wraps around in HSV, need two ranges */
      cv::Mat mask1, mask2;
      cv::inRange(hsv, cv::Scalar(0, 100, 100),   cv::Scalar(10, 255, 255), mask1);
      cv::inRange(hsv, cv::Scalar(156, 100, 100), cv::Scalar(180, 255, 255), mask2);
      cv::bitwise_or(mask1, mask2, mask);
    } else {
      cv::inRange(hsv, hsv_lower_, hsv_upper_, mask);
    }

    /* Morphological cleanup */
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    /* Find contours */
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    wheel_leg_msgs::msg::LineState line_state;
    line_state.line_detected = false;
    line_state.lateral_error = 0.0f;
    line_state.angular_error = 0.0f;
    line_state.confidence = 0.0f;

    if (!contours.empty()) {
      /* Select largest contour */
      auto best = std::max_element(contours.begin(), contours.end(),
        [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
          return cv::contourArea(a) < cv::contourArea(b);
        });

      double area = cv::contourArea(*best);
      if (area > min_area_) {
        /* Compute centroid of the contour */
        cv::Moments m = cv::moments(*best);
        if (m.m00 > 0) {
          float cx = m.m10 / m.m00;  /* centroid x in ROI coordinates */
          float cy = m.m01 / m.m00;  /* centroid y in ROI coordinates */

          /* Lateral error: normalized (-1 to 1), 0 = center */
          float roi_center_x = w / 2.0f;
          line_state.lateral_error = (cx - roi_center_x) / roi_center_x;

          /* Fit line to estimate angle */
          cv::Vec4f line_param;
          cv::fitLine(*best, line_param, cv::DIST_L2, 0, 0.01, 0.01);
          float vx = line_param[0], vy = line_param[1];
          float line_angle = std::atan2(vy, vx);  /* angle from horizontal */

          /* Angular error: deviation from vertical (PI/2) */
          float desired_angle = CV_PI / 2.0f;  /* vertical line = PI/2 */
          line_state.angular_error = static_cast<float>(line_angle - desired_angle);

          line_state.line_detected = true;
          line_state.confidence = std::min(area / 5000.0, 1.0);  /* normalize area */

          /* Debug: draw on image */
          if (publish_debug_) {
            cv::Point pt1(cx - vx * 50, cy - vy * 50);
            cv::Point pt2(cx + vx * 50, cy + vy * 50);
            cv::line(roi_frame, pt1, pt2, cv::Scalar(0, 255, 0), 2);
            cv::circle(roi_frame, cv::Point(cx, cy), 5, cv::Scalar(0, 0, 255), -1);
            cv::putText(roi_frame, "LINE", cv::Point(cx, cy - 10),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
          }
        }
      }
    }

    line_pub_->publish(line_state);

    /* Publish debug image */
    if (publish_debug_) {
      /* Copy ROI back for display */
      roi_frame.copyTo(frame(roi));
      cv::rectangle(frame, roi, cv::Scalar(255, 255, 0), 1);
      auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
      debug_pub_->publish(*debug_msg);
    }
  }

  /* Parameters */
  std::string color_;
  cv::Scalar hsv_lower_, hsv_upper_;
  double roi_top_, roi_bottom_;
  int min_area_;
  bool publish_debug_;

  /* ROS2 */
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::LineState>::SharedPtr line_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
