#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/msg/float64.hpp>
#include <cmath>

class OdometryPublisher : public rclcpp::Node
{
public:
  OdometryPublisher()
  : Node("odometry_publisher")
  {
    this->declare_parameter("wheel_radius", 0.0473);
    this->declare_parameter("wheel_separation", 0.1796);
    this->declare_parameter("publish_rate", 50.0);

    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    double publish_rate = this->get_parameter("publish_rate").as_double();

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    pos_l_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/esp/position_l_rad", 10,
      std::bind(&OdometryPublisher::pos_l_callback, this, std::placeholders::_1));

    pos_r_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/esp/position_r_rad", 10,
      std::bind(&OdometryPublisher::pos_r_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      std::bind(&OdometryPublisher::publish_odometry, this));

    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;

    left_pos_prev_ = 0.0;
    right_pos_prev_ = 0.0;

    first_left_ = true;
    first_right_ = true;

    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Odometry Publisher initialized");
    RCLCPP_INFO(this->get_logger(), "Wheel radius: %.4f m", wheel_radius_);
    RCLCPP_INFO(this->get_logger(), "Wheel separation: %.4f m", wheel_separation_);
  }

private:
  void pos_l_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (first_left_) {
      left_pos_prev_ = msg->data;
      first_left_ = false;
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    left_pos_ = msg->data;
    left_received_ = true;
  }

  void pos_r_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (first_right_) {
      right_pos_prev_ = msg->data;
      first_right_ = false;
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    right_pos_ = msg->data;
    right_received_ = true;
  }

  void publish_odometry()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!left_received_ || !right_received_) {
      return;
    }

    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    double delta_left = left_pos_ - left_pos_prev_;
    double delta_right = right_pos_ - right_pos_prev_;

    left_pos_prev_ = left_pos_;
    right_pos_prev_ = right_pos_;

    double left_dist = delta_left * wheel_radius_;
    double right_dist = delta_right * wheel_radius_;

    double dist = (left_dist + right_dist) / 2.0;
    double delta_theta = (right_dist - left_dist) / wheel_separation_;

    double delta_x = dist * std::cos(theta_);
    double delta_y = dist * std::sin(theta_);

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[14] = 0.0;
    odom.pose.covariance[21] = 0.0;
    odom.pose.covariance[28] = 0.0;
    odom.pose.covariance[35] = 0.001;

    double vx = dist / dt;
    double vtheta = delta_theta / dt;

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = vtheta;

    odom.twist.covariance[0] = 0.001;
    odom.twist.covariance[7] = 0.001;
    odom.twist.covariance[14] = 0.0;
    odom.twist.covariance[21] = 0.0;
    odom.twist.covariance[28] = 0.0;
    odom.twist.covariance[35] = 0.001;

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = current_time;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";

    odom_tf.transform.translation.x = x_;
    odom_tf.transform.translation.y = y_;
    odom_tf.transform.translation.z = 0.0;

    odom_tf.transform.rotation.x = q.x();
    odom_tf.transform.rotation.y = q.y();
    odom_tf.transform.rotation.z = q.z();
    odom_tf.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(odom_tf);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pos_l_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pos_r_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double wheel_radius_;
  double wheel_separation_;
  double x_, y_, theta_;
  double left_pos_, right_pos_;
  double left_pos_prev_, right_pos_prev_;
  bool first_left_, first_right_;
  bool left_received_{false};
  bool right_received_{false};
  rclcpp::Time last_time_;

  std::mutex mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryPublisher>());
  rclcpp::shutdown();
  return 0;
}
