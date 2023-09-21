#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "lbr_fri_ros2/app.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/exceptions.h"
#include "end_controller.hpp"

class EndControlNode : public rclcpp::Node {
public:
  EndControlNode(const std::string &node_name, const rclcpp::NodeOptions &options)
  : rclcpp::Node(node_name, options) {
    this->declare_parameter<std::string>("robot_description");
    this->declare_parameter<std::string>("base_link", "link_0");
    this->declare_parameter<std::string>("end_effector_link", "link_ee");
    this->declare_parameter<std::string>("end_effector_link_expect", "link_ee_expect");

    base_link_= this->get_parameter("base_link").as_string();
    end_effector_link_= this->get_parameter("end_effector_link").as_string();
    end_effector_link_expect_= this->get_parameter("end_effector_link_expect").as_string();

    end_controller_ = std::make_unique<EndController>(
        this->get_parameter("robot_description").as_string(),
        base_link_,
        end_effector_link_);

    lbr_command_pub_ = create_publisher<lbr_fri_msgs::msg::LBRCommand>(
        "/lbr/command", rclcpp::QoS(1)
                            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                            .deadline(std::chrono::milliseconds(10)));
    lbr_state_sub_ = create_subscription<lbr_fri_msgs::msg::LBRState>(
        "/lbr/state",
        rclcpp::QoS(1)
            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            .deadline(std::chrono::milliseconds(10)),
        std::bind(&EndControlNode::on_lbr_state, this, std::placeholders::_1));

    expect_tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock()); // default 10s
    tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*expect_tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  }

protected:
  void on_lbr_state(const lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state) {
    if (!lbr_state) {
      return;
    }

    smooth_lbr_state_(lbr_state, 0.0);// open loop, no noise problem

    KDL::Frame end_frame_ = end_controller_->get_end_frame(lbr_state_);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = base_link_;
    t.child_frame_id = end_effector_link_;
    t.transform.translation.x = end_frame_.p.x();
    t.transform.translation.y = end_frame_.p.y();
    t.transform.translation.z = end_frame_.p.z();
    end_frame_.M.GetQuaternion(t.transform.rotation.x,t.transform.rotation.y,
                               t.transform.rotation.z,t.transform.rotation.w);
    tf_broadcaster_->sendTransform(t);

    try{
      command_tf_ = expect_tf_buffer_->lookupTransform(
              base_link_, end_effector_link_expect_, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_INFO(
              this->get_logger(), "Could not transform %s to %s: %s",
              end_effector_link_expect_.c_str(),base_link_.c_str(), ex.what());
      return;
    }



    auto lbr_command = end_controller_->update(lbr_state_, command_tf_);
    lbr_command_pub_->publish(lbr_command);

  };

  void smooth_lbr_state_(const lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state, double alpha) {
    if (!init_) {
      lbr_state_ = *lbr_state;
      init_ = true;
      return;
    }

    for (int i = 0; i < 7; i++) {
      lbr_state_.measured_joint_position[i] = lbr_state->measured_joint_position[i] * (1 - alpha) +
                                              lbr_state_.measured_joint_position[i] * alpha;
      lbr_state_.external_torque[i] =
          lbr_state->external_torque[i] * (1 - alpha) + lbr_state_.external_torque[i] * alpha;
    }
  }

  bool init_{false};
  lbr_fri_msgs::msg::LBRState lbr_state_;
  std::string base_link_;
  std::string end_effector_link_;
  std::string end_effector_link_expect_;
  rclcpp::Publisher<lbr_fri_msgs::msg::LBRCommand>::SharedPtr lbr_command_pub_;
  rclcpp::Subscription<lbr_fri_msgs::msg::LBRState>::SharedPtr lbr_state_sub_;
  std::unique_ptr<tf2_ros::Buffer> expect_tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped command_tf_;


  std::unique_ptr<EndController> end_controller_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();

  auto app_node = std::make_shared<rclcpp::Node>(
      "app", "lbr", rclcpp::NodeOptions().use_intra_process_comms(true));

  auto app = lbr_fri_ros2::App(app_node);

  auto end_control_node = std::make_shared<EndControlNode>(
      "end_control_node", rclcpp::NodeOptions().use_intra_process_comms(true));

  executor->add_node(app_node);
  executor->add_node(end_control_node);
  executor->spin();

  return 0;
}
