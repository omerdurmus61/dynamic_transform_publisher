#include <memory>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;

static inline bool is_auto(const std::string &s) {
  return s == "auto" || s == "-";
}

enum class MsgKind {
  ODOM,
  POSE_STAMPED,
  POSE_COV_STAMPED,
  TF_STAMPED,
  POSE,
  POSE_COV
};

static MsgKind parse_msg_kind(std::string msg_type) {
  std::transform(msg_type.begin(), msg_type.end(), msg_type.begin(), ::tolower);

  if (msg_type == "nav_msgs/msg/odometry") return MsgKind::ODOM;
  if (msg_type == "geometry_msgs/msg/posestamped") return MsgKind::POSE_STAMPED;
  if (msg_type == "geometry_msgs/msg/posewithcovariancestamped") return MsgKind::POSE_COV_STAMPED;
  if (msg_type == "geometry_msgs/msg/transformstamped") return MsgKind::TF_STAMPED;
  if (msg_type == "geometry_msgs/msg/pose") return MsgKind::POSE;
  if (msg_type == "geometry_msgs/msg/posewithcovariance") return MsgKind::POSE_COV;

  throw std::runtime_error("Unsupported msg_type: " + msg_type);
}

class DynamicTransformPublisher : public rclcpp::Node
{
public:
  DynamicTransformPublisher(const std::string &topic,
                            const std::string &msg_type,
                            const std::string &parent_frame_arg,
                            const std::string &child_frame_arg)
      : Node("dynamic_transform_publisher"),
        parent_frame_arg_(parent_frame_arg),
        child_frame_arg_(child_frame_arg)
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    try {
      kind_ = parse_msg_kind(msg_type);
    } catch (const std::exception &e) {
      RCLCPP_FATAL(get_logger(), "%s", e.what());
      throw;
    }

    // ✅ Senin istediğin loglar
    RCLCPP_INFO(get_logger(), "Subscribing topic: %s", topic.c_str());
    RCLCPP_INFO(get_logger(), "Msg type: %s", msg_type.c_str());
    RCLCPP_INFO(get_logger(), "Parent frame: %s", parent_frame_arg_.c_str());
    RCLCPP_INFO(get_logger(), "Child  frame: %s", child_frame_arg_.c_str());

    switch (kind_) {
      case MsgKind::ODOM:
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            topic, 10, std::bind(&DynamicTransformPublisher::odom_cb, this, _1));
        break;
      case MsgKind::POSE_STAMPED:
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            topic, 10, std::bind(&DynamicTransformPublisher::pose_cb, this, _1));
        break;
      case MsgKind::POSE_COV_STAMPED:
        pose_cov_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            topic, 10, std::bind(&DynamicTransformPublisher::pose_cov_cb, this, _1));
        break;
      case MsgKind::TF_STAMPED:
        tf_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
            topic, 10, std::bind(&DynamicTransformPublisher::tf_cb, this, _1));
        break;
      case MsgKind::POSE:
        pose2_sub_ = create_subscription<geometry_msgs::msg::Pose>(
            topic, 10, std::bind(&DynamicTransformPublisher::pose2_cb, this, _1));
        break;
      case MsgKind::POSE_COV:
        pose_cov2_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovariance>(
            topic, 10, std::bind(&DynamicTransformPublisher::pose_cov2_cb, this, _1));
        break;
    }
  }

private:
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = is_auto(parent_frame_arg_) ? msg->header.frame_id : parent_frame_arg_;
    std::string child = is_auto(child_frame_arg_) ? msg->child_frame_id : child_frame_arg_;
    if (child.empty()) child = "base_link";
    t.child_frame_id = child;
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;
    t.transform.rotation = msg->pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);

    RCLCPP_INFO(this->get_logger(),
        "TF published [Odometry] %s -> %s (x=%.2f, y=%.2f, z=%.2f)",
        t.header.frame_id.c_str(), t.child_frame_id.c_str(),
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
  }

  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = is_auto(parent_frame_arg_) ? msg->header.frame_id : parent_frame_arg_;
    t.child_frame_id = is_auto(child_frame_arg_) ? "base_link" : child_frame_arg_;
    t.transform.translation.x = msg->pose.position.x;
    t.transform.translation.y = msg->pose.position.y;
    t.transform.translation.z = msg->pose.position.z;
    t.transform.rotation = msg->pose.orientation;
    tf_broadcaster_->sendTransform(t);

    RCLCPP_INFO(this->get_logger(),
        "TF published [PoseStamped] %s -> %s (x=%.2f, y=%.2f, z=%.2f)",
        t.header.frame_id.c_str(), t.child_frame_id.c_str(),
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
  }

  void pose_cov_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = is_auto(parent_frame_arg_) ? msg->header.frame_id : parent_frame_arg_;
    t.child_frame_id = is_auto(child_frame_arg_) ? "base_link" : child_frame_arg_;
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;
    t.transform.rotation = msg->pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);

    RCLCPP_INFO(this->get_logger(),
        "TF published [PoseWithCovStamped] %s -> %s (x=%.2f, y=%.2f, z=%.2f)",
        t.header.frame_id.c_str(), t.child_frame_id.c_str(),
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
  }

  void tf_cb(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t = *msg;
    if (!is_auto(parent_frame_arg_)) t.header.frame_id = parent_frame_arg_;
    if (!is_auto(child_frame_arg_)) t.child_frame_id   = child_frame_arg_;
    tf_broadcaster_->sendTransform(t);

    RCLCPP_INFO(this->get_logger(),
        "TF published [TransformStamped] %s -> %s",
        t.header.frame_id.c_str(), t.child_frame_id.c_str());
  }

  void pose2_cb(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = is_auto(parent_frame_arg_) ? "map" : parent_frame_arg_;
    t.child_frame_id  = is_auto(child_frame_arg_) ? "base_link" : child_frame_arg_;
    t.transform.translation.x = msg->position.x;
    t.transform.translation.y = msg->position.y;
    t.transform.translation.z = msg->position.z;
    t.transform.rotation = msg->orientation;
    tf_broadcaster_->sendTransform(t);

    RCLCPP_INFO(this->get_logger(),
        "TF published [Pose] %s -> %s (x=%.2f, y=%.2f, z=%.2f)",
        t.header.frame_id.c_str(), t.child_frame_id.c_str(),
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
  }

  void pose_cov2_cb(const geometry_msgs::msg::PoseWithCovariance::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = is_auto(parent_frame_arg_) ? "map" : parent_frame_arg_;
    t.child_frame_id  = is_auto(child_frame_arg_) ? "base_link" : child_frame_arg_;
    t.transform.translation.x = msg->pose.position.x;
    t.transform.translation.y = msg->pose.position.y;
    t.transform.translation.z = msg->pose.position.z;
    t.transform.rotation = msg->pose.orientation;
    tf_broadcaster_->sendTransform(t);

    RCLCPP_INFO(this->get_logger(),
        "TF published [PoseWithCovariance] %s -> %s (x=%.2f, y=%.2f, z=%.2f)",
        t.header.frame_id.c_str(), t.child_frame_id.c_str(),
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
  }

  MsgKind kind_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr tf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose2_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovariance>::SharedPtr pose_cov2_sub_;

  std::string parent_frame_arg_;
  std::string child_frame_arg_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 5) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
      "Usage:\n  ros2 run dynamic_transform_publisher dynamic_transform_publisher_node "
      "<topic> <msg_type> <parent_frame|auto> <child_frame|auto>\n\n"
      "Examples:\n"
      "  ros2 run dynamic_transform_publisher dynamic_transform_publisher_node "
      "/encoder_odom nav_msgs/msg/Odometry odom base_link\n"
      "  ros2 run dynamic_transform_publisher dynamic_transform_publisher_node "
      "/pose geometry_msgs/msg/PoseStamped map base_link\n"
      "  ros2 run dynamic_transform_publisher dynamic_transform_publisher_node "
      "/tf_in geometry_msgs/msg/TransformStamped auto auto");
    rclcpp::shutdown();
    return 1;
  }

  try {
    auto node = std::make_shared<DynamicTransformPublisher>(argv[1], argv[2], argv[3], argv[4]);
    rclcpp::spin(node);
  } catch (...) {}

  rclcpp::shutdown();
  return 0;
}
