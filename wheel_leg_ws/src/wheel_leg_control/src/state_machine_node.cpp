/**
 * @file state_machine_node.cpp
 * @brief Main state machine controller for the wheel-legged robot.
 *
 * States: IDLE -> LINE_FOLLOW -> RETURN_HOME -> STOPPED (loop back to IDLE)
 *
 * Subscribes:
 *   /joy                     (sensor_msgs/Joy) -- gamepad/button input
 *   /wheel_leg/line_state    (wheel_leg_msgs/LineState)
 *   /wheel_leg/box_state     (wheel_leg_msgs/BoxState)
 *   /odom                     (nav_msgs/Odometry)
 *
 * Publishes:
 *   /cmd_vel                  (geometry_msgs/Twist)
 *   /wheel_leg/cmd_start      (std_msgs/Bool) -- start/stop the robot
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <wheel_leg_msgs/msg/line_state.hpp>
#include <wheel_leg_msgs/msg/box_state.hpp>
#include <cmath>
#include <string>

enum class RobotState {
  IDLE = 0,
  LINE_FOLLOW,
  RETURN_HOME,
  STOPPED
};

class StateMachineNode : public rclcpp::Node
{
public:
  StateMachineNode()
    : Node("state_machine")
  {
    /* Parameters */
    this->declare_parameter("base_speed", 0.15);
    this->declare_parameter("max_speed", 0.3);
    this->declare_parameter("kp_lateral", 3.0);
    this->declare_parameter("kd_lateral", 0.5);
    this->declare_parameter("max_home_speed", 0.2);
    this->declare_parameter("kp_distance", 0.8);
    this->declare_parameter("kp_yaw", 1.5);
    this->declare_parameter("target_tolerance", 0.05);
    this->declare_parameter("yaw_tolerance", 0.05);
    this->declare_parameter("stop_delay", 5.0);

    base_speed_         = this->get_parameter("base_speed").as_double();
    max_speed_          = this->get_parameter("max_speed").as_double();
    kp_lateral_         = this->get_parameter("kp_lateral").as_double();
    kd_lateral_         = this->get_parameter("kd_lateral").as_double();
    max_home_speed_     = this->get_parameter("max_home_speed").as_double();
    kp_distance_        = this->get_parameter("kp_distance").as_double();
    kp_yaw_             = this->get_parameter("kp_yaw").as_double();
    target_tolerance_   = this->get_parameter("target_tolerance").as_double();
    yaw_tolerance_      = this->get_parameter("yaw_tolerance").as_double();
    stop_delay_         = this->get_parameter("stop_delay").as_double();

    /* Subscribers */
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&StateMachineNode::joy_callback, this, std::placeholders::_1));

    line_sub_ = this->create_subscription<wheel_leg_msgs::msg::LineState>(
      "/wheel_leg/line_state", 10,
      std::bind(&StateMachineNode::line_callback, this, std::placeholders::_1));

    box_sub_ = this->create_subscription<wheel_leg_msgs::msg::BoxState>(
      "/wheel_leg/box_state", 10,
      std::bind(&StateMachineNode::box_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&StateMachineNode::odom_callback, this, std::placeholders::_1));

    /* Publishers */
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    cmd_start_pub_ = this->create_publisher<std_msgs::msg::Bool>("/wheel_leg/cmd_start", 10);

    /* Control loop timer: 50Hz (20ms) */
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&StateMachineNode::control_loop, this));

    /* Status reporting timer: 1Hz */
    status_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&StateMachineNode::status_report, this));

    RCLCPP_INFO(this->get_logger(), "StateMachine started. Press B/Circle to start line follow.");
    RCLCPP_INFO(this->get_logger(), "Press A/Cross to return home. Press Y/Triangle for emergency stop.");
  }

private:
  /* ====== Callbacks ====== */

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    /* Button mapping (PS4 controller):
     *   buttons[0] = Cross/A     -> RETURN_HOME
     *   buttons[1] = Circle/B    -> LINE_FOLLOW
     *   buttons[2] = Triangle/Y  -> EMERGENCY STOP
     *   buttons[3] = Square/X    -> MARK HOME POSITION
     */

    bool cross   = (msg->buttons.size() > 0) ? msg->buttons[0] : 0;
    bool circle  = (msg->buttons.size() > 1) ? msg->buttons[1] : 0;
    bool triangle = (msg->buttons.size() > 2) ? msg->buttons[2] : 0;
    bool square  = (msg->buttons.size() > 3) ? msg->buttons[3] : 0;

    /* Rising edge detection */
    if (circle && !last_circle_) {
      /* Start line following */
      if (state_ == RobotState::IDLE || state_ == RobotState::STOPPED) {
        state_ = RobotState::LINE_FOLLOW;
        home_x_ = odom_x_;
        home_y_ = odom_y_;
        home_yaw_ = odom_yaw_;
        RCLCPP_INFO(this->get_logger(), "STATE: IDLE -> LINE_FOLLOW. Home saved: (%.2f, %.2f, %.2f)",
          home_x_, home_y_, home_yaw_);
      }
    }

    if (cross && !last_cross_) {
      /* Return to home */
      if (state_ == RobotState::LINE_FOLLOW || state_ == RobotState::IDLE) {
        state_ = RobotState::RETURN_HOME;
        RCLCPP_INFO(this->get_logger(), "STATE: -> RETURN_HOME. Target: (%.2f, %.2f, %.2f)",
          home_x_, home_y_, home_yaw_);
      }
    }

    if (triangle && !last_triangle_) {
      /* Emergency stop */
      state_ = RobotState::IDLE;
      publish_stop();
      RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP");
    }

    if (square && !last_square_) {
      /* Mark current position as home */
      home_x_ = odom_x_;
      home_y_ = odom_y_;
      home_yaw_ = odom_yaw_;
      RCLCPP_INFO(this->get_logger(), "Home position marked: (%.2f, %.2f, %.2f)",
        home_x_, home_y_, home_yaw_);
    }

    last_cross_   = cross;
    last_circle_  = circle;
    last_triangle_ = triangle;
    last_square_  = square;
  }

  void line_callback(const wheel_leg_msgs::msg::LineState::SharedPtr msg)
  {
    last_line_ = *msg;
  }

  void box_callback(const wheel_leg_msgs::msg::BoxState::SharedPtr msg)
  {
    last_box_ = *msg;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_x_ = msg->pose.pose.position.x;
    odom_y_ = msg->pose.pose.position.y;

    /* Extract yaw from quaternion */
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    odom_yaw_ = 2.0 * std::atan2(qz, qw);
  }

  /* ====== Control Loop (50Hz) ====== */

  void control_loop()
  {
    switch (state_) {
      case RobotState::IDLE:
        publish_zero();
        break;

      case RobotState::LINE_FOLLOW:
        control_line_follow();
        break;

      case RobotState::RETURN_HOME:
        control_return_home();
        break;

      case RobotState::STOPPED:
        publish_zero();
        /* Auto-return to IDLE after stop_delay */
        if ((this->now() - stop_time_).seconds() > stop_delay_) {
          state_ = RobotState::IDLE;
          RCLCPP_INFO(this->get_logger(), "STATE: STOPPED -> IDLE");
        }
        break;
    }
  }

  void control_line_follow()
  {
    geometry_msgs::msg::Twist cmd;

    if (last_line_.line_detected) {
      /* Pure pursuit: maintain forward speed, steer based on lateral error */
      double speed = base_speed_ * (1.0 - std::abs(last_line_.lateral_error));
      speed = std::max(speed, 0.03);  /* minimum forward speed */
      speed = std::min(speed, max_speed_);

      cmd.linear.x = speed;

      /* Lateral error: negative = line is left, need to turn left (positive omega)
       * Convert lateral_error (-1 to 1) to angular velocity */
      cmd.angular.z = kp_lateral_ * last_line_.lateral_error
                    + kd_lateral_ * (last_line_.lateral_error - last_lateral_error_);
      last_lateral_error_ = last_line_.lateral_error;
    } else {
      /* Line lost: slow forward (may reacquire) */
      cmd.linear.x = 0.05;
      cmd.angular.z = 0.0;
    }

    cmd_vel_pub_->publish(cmd);
  }

  void control_return_home()
  {
    geometry_msgs::msg::Twist cmd;

    /* Compute errors */
    double dx = home_x_ - odom_x_;
    double dy = home_y_ - odom_y_;
    double distance = std::sqrt(dx * dx + dy * dy);

    /* Check if we've arrived */
    if (distance < target_tolerance_) {
      state_ = RobotState::STOPPED;
      stop_time_ = this->now();
      publish_stop();
      RCLCPP_INFO(this->get_logger(), "STATE: RETURN_HOME -> STOPPED (arrived at home)");
      return;
    }

    /* Desired heading: angle to home */
    double desired_yaw = std::atan2(dy, dx);
    double yaw_error = desired_yaw - odom_yaw_;

    /* Normalize to [-PI, PI] */
    while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

    /* Two-phase approach:
     *   Phase 1: Turn in place until aligned (yaw_error > 15 degrees)
     *   Phase 2: Move forward while maintaining heading
     */
    const double TURN_THRESH = 0.26;  /* ~15 degrees */
    if (std::abs(yaw_error) > TURN_THRESH) {
      /* Phase 1: Pure rotation */
      cmd.linear.x = 0.0;
      cmd.angular.z = std::clamp(kp_yaw_ * yaw_error, -0.8, 0.8);
    } else {
      /* Phase 2: Forward + heading correction */
      double speed = std::clamp(kp_distance_ * distance, 0.05, max_home_speed_);
      cmd.linear.x = speed;
      cmd.angular.z = std::clamp(kp_yaw_ * yaw_error, -0.5, 0.5);
    }

    cmd_vel_pub_->publish(cmd);
  }

  void publish_zero()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);
  }

  void publish_stop()
  {
    publish_zero();
    auto msg = std_msgs::msg::Bool();
    msg.data = false;
    cmd_start_pub_->publish(msg);
  }

  void status_report()
  {
    const char* state_names[] = {"IDLE", "LINE_FOLLOW", "RETURN_HOME", "STOPPED"};
    int idx = static_cast<int>(state_);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "State: %s | Odometry: (%.2f, %.2f, %.2f) | Home: (%.2f, %.2f, %.2f)",
      state_names[idx], odom_x_, odom_y_, odom_yaw_, home_x_, home_y_, home_yaw_);
  }

  /* ====== State ====== */

  RobotState state_ = RobotState::IDLE;

  /* Home position (set on LINE_FOLLOW start) */
  double home_x_ = 0.0, home_y_ = 0.0, home_yaw_ = 0.0;

  /* Current odometry */
  double odom_x_ = 0.0, odom_y_ = 0.0, odom_yaw_ = 0.0;

  /* Latest perception data */
  wheel_leg_msgs::msg::LineState last_line_;
  wheel_leg_msgs::msg::BoxState  last_box_;
  double last_lateral_error_ = 0.0;

  /* Button state for edge detection */
  bool last_cross_ = false, last_circle_ = false;
  bool last_triangle_ = false, last_square_ = false;

  /* Timers */
  rclcpp::Time stop_time_;

  /* Parameters */
  double base_speed_, max_speed_;
  double kp_lateral_, kd_lateral_;
  double max_home_speed_, kp_distance_, kp_yaw_;
  double target_tolerance_, yaw_tolerance_, stop_delay_;

  /* ROS2 */
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<wheel_leg_msgs::msg::LineState>::SharedPtr line_sub_;
  rclcpp::Subscription<wheel_leg_msgs::msg::BoxState>::SharedPtr box_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cmd_start_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateMachineNode>());
  rclcpp::shutdown();
  return 0;
}
