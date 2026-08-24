/**
 * @file box_detector_node.cpp
 * @brief Detects a cardboard box wrapped with colored tape using HSV thresholding.
 *
 * Subscribes: /image_left_raw  (sensor_msgs/Image, BGR8)
 *             /image_right_raw (sensor_msgs/Image, BGR8) [optional, for depth]
 * Publishes:  /wheel_leg/box_state (wheel_leg_msgs/BoxState)
 *
 * Strategy:
 *   1. BGR -> HSV threshold for tape color
 *   2. Find largest connected region -> bounding box
 *   3. (Optional) Use stereo disparity at box center for rough depth estimate
 *   4. Apply EMA smoothing for temporal stability
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <wheel_leg_msgs/msg/box_state.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class BoxDetectorNode : public rclcpp::Node
{
public:
  BoxDetectorNode()
    : Node("box_detector")
  {
    /* Parameters */
    this->declare_parameter("color", "red");
    this->declare_parameter("box_hsv_lower", std::vector<int64_t>({0, 100, 100}));
    this->declare_parameter("box_hsv_upper", std::vector<int64_t>({10, 255, 255}));
    this->declare_parameter("min_box_area", 500);
    this->declare_parameter("ema_alpha", 0.7f);
    this->declare_parameter("publish_debug_img", true);

    color_ = this->get_parameter("color").as_string();
    auto lower = this->get_parameter("box_hsv_lower").as_integer_array();
    auto upper = this->get_parameter("box_hsv_upper").as_integer_array();
    hsv_lower_ = cv::Scalar(lower[0], lower[1], lower[2]);
    hsv_upper_ = cv::Scalar(upper[0], upper[1], upper[2]);
    min_area_ = this->get_parameter("min_box_area").as_int();
    ema_alpha_ = this->get_parameter("ema_alpha").as_double();
    publish_debug_ = this->get_parameter("publish_debug_img").as_bool();

    /* Subscriber: left (and optionally right) camera images */
    left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_left_raw", 10,
      std::bind(&BoxDetectorNode::left_callback, this, std::placeholders::_1));

    right_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_right_raw", 10,
      std::bind(&BoxDetectorNode::right_callback, this, std::placeholders::_1));

    /* Publisher */
    box_pub_ = this->create_publisher<wheel_leg_msgs::msg::BoxState>(
      "/wheel_leg/box_state", 10);

    if (publish_debug_) {
      debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/box_detector/debug_img", 10);
    }

    RCLCPP_INFO(this->get_logger(), "BoxDetector started (color=%s)", color_.c_str());
  }

private:
  void left_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      return;
    }
    left_frame_ = cv_ptr->image.clone();
    left_header_ = msg->header;

    process_frame();
  }

  void right_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      return;
    }
    right_frame_ = cv_ptr->image.clone();
  }

  void process_frame()
  {
    if (left_frame_.empty()) return;

    cv::Mat hsv, mask;
    cv::cvtColor(left_frame_, hsv, cv::COLOR_BGR2HSV);

    if (color_ == "red") {
      cv::Mat mask1, mask2;
      cv::inRange(hsv, cv::Scalar(0, 100, 100),   cv::Scalar(10, 255, 255), mask1);
      cv::inRange(hsv, cv::Scalar(156, 100, 100), cv::Scalar(180, 255, 255), mask2);
      cv::bitwise_or(mask1, mask2, mask);
    } else {
      cv::inRange(hsv, hsv_lower_, hsv_upper_, mask);
    }

    /* Morphological cleanup */
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    /* Find contours */
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    wheel_leg_msgs::msg::BoxState box_state;
    box_state.box_detected = false;

    if (!contours.empty()) {
      auto best = std::max_element(contours.begin(), contours.end(),
        [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
          return cv::contourArea(a) < cv::contourArea(b);
        });

      double area = cv::contourArea(*best);
      if (area > min_area_) {
        cv::Rect bbox = cv::boundingRect(*best);
        int w = left_frame_.cols, h = left_frame_.rows;

        /* Raw measurements */
        float raw_cx = (bbox.x + bbox.width / 2.0f) / w;
        float raw_cy = (bbox.y + bbox.height / 2.0f) / h;
        float raw_bw = bbox.width / static_cast<float>(w);
        float raw_bh = bbox.height / static_cast<float>(h);

        /* EMA smoothing */
        if (first_detection_) {
          smoothed_cx_ = raw_cx;
          smoothed_cy_ = raw_cy;
          smoothed_bw_ = raw_bw;
          smoothed_bh_ = raw_bh;
          first_detection_ = false;
        } else {
          smoothed_cx_ = ema_alpha_ * smoothed_cx_ + (1.0f - ema_alpha_) * raw_cx;
          smoothed_cy_ = ema_alpha_ * smoothed_cy_ + (1.0f - ema_alpha_) * raw_cy;
          smoothed_bw_ = ema_alpha_ * smoothed_bw_ + (1.0f - ema_alpha_) * raw_bw;
          smoothed_bh_ = ema_alpha_ * smoothed_bh_ + (1.0f - ema_alpha_) * raw_bh;
        }

        box_state.box_detected = true;
        box_state.bbox_center_x = smoothed_cx_;
        box_state.bbox_center_y = smoothed_cy_;
        box_state.bbox_width    = smoothed_bw_;
        box_state.bbox_height   = smoothed_bh_;

        /* Rough depth estimation from stereo */
        /* If we have right frame, compute disparity at box center */
        float distance = estimate_depth(bbox);
        box_state.distance = distance;

        box_state.confidence = std::min(area / 5000.0, 1.0);

        /* Debug visualization */
        if (publish_debug_) {
          cv::rectangle(left_frame_, bbox, cv::Scalar(0, 255, 0), 2);
          int cx = static_cast<int>(box_state.bbox_center_x * left_frame_.cols);
          int cy = static_cast<int>(box_state.bbox_center_y * left_frame_.rows);
          cv::circle(left_frame_, cv::Point(cx, cy), 5, cv::Scalar(0, 0, 255), -1);
          std::string label = "BOX d=" + std::to_string(distance).substr(0, 4) + "m";
          cv::putText(left_frame_, label, cv::Point(bbox.x, bbox.y - 5),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }
      }
    } else {
      first_detection_ = true;  /* reset EMA on loss */
    }

    box_pub_->publish(box_state);

    /* Debug image */
    if (publish_debug_) {
      cv::putText(left_frame_, box_state.box_detected ? "BOX FOUND" : "NO BOX",
        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
        box_state.box_detected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
      auto debug_msg = cv_bridge::CvImage(left_header_, "bgr8", left_frame_).toImageMsg();
      debug_pub_->publish(*debug_msg);
    }
  }

  /** Simple stereo disparity-based distance estimate.
   *  Falls back to bounding-box-size heuristic if no right frame available. */
  float estimate_depth(const cv::Rect& bbox)
  {
    if (!right_frame_.empty() && left_frame_.rows == right_frame_.rows) {
      /* Use template matching or block matching at box center */
      int cx = bbox.x + bbox.width / 2;
      int cy = bbox.y + bbox.height / 2;
      int patch_size = 30;

      int x1 = std::max(cx - patch_size / 2, 0);
      int y1 = std::max(cy - patch_size / 2, 0);
      int pw = std::min(patch_size, left_frame_.cols - x1);
      int ph = std::min(patch_size, left_frame_.rows - y1);

      if (pw > 10 && ph > 10) {
        cv::Rect patch_rect(x1, y1, pw, ph);

        /* Search in right image on epipolar line (horizontal only) */
        int search_range = std::min(80, right_frame_.cols - x1 - pw);
        if (search_range > 0) {
          cv::Mat left_patch  = left_frame_(patch_rect);
          cv::Mat search_strip = right_frame_(cv::Rect(x1, y1, pw + search_range, ph));

          cv::Mat result;
          cv::matchTemplate(search_strip, left_patch, result, cv::TM_CCOEFF_NORMED);

          double min_val, max_val;
          cv::Point min_loc, max_loc;
          cv::minMaxLoc(result, &min_val, &max_val, &min_loc, &max_loc);

          float disparity_px = static_cast<float>(max_loc.x);
          if (disparity_px > 1.0f && max_val > 0.6f) {
            /* Z = f * B / d
             *   f ≈ 395 px (from GS130W spec: 1280px horizontal, 115.6° FOV)
             *   B = 0.08 m (baseline)
             */
            float focal_px = 395.0f;
            float baseline_m = 0.08f;
            return (focal_px * baseline_m) / disparity_px;
          }
        }
      }
    }

    /* Fallback: distance heuristic from bounding box size */
    /* Assuming the box is ~0.3m wide when viewed at 0.5m */
    if (bbox.width > 0) {
      float bbox_ratio = bbox.width / static_cast<float>(left_frame_.cols);
      if (bbox_ratio > 0.01f) {
        return 1.0f / (bbox_ratio * 5.0f);  /* rough heuristic */
      }
    }

    return -1.0f;  /* unknown */
  }

  /* Parameters */
  std::string color_;
  cv::Scalar hsv_lower_, hsv_upper_;
  int min_area_;
  double ema_alpha_;
  bool publish_debug_;

  /* State */
  cv::Mat left_frame_, right_frame_;
  std_msgs::msg::Header left_header_;
  float smoothed_cx_ = 0.0f, smoothed_cy_ = 0.0f;
  float smoothed_bw_ = 0.0f, smoothed_bh_ = 0.0f;
  bool first_detection_ = true;

  /* ROS2 */
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::BoxState>::SharedPtr box_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BoxDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
