#ifndef LBR_DEMOS_FRI_ROS2_ADVANCED_CPP__END_CONTROLLER_HPP_
#define LBR_DEMOS_FRI_ROS2_ADVANCED_CPP__END_CONTROLLER_HPP_

#include <cstring>
#include <string>

#include "kdl/chain.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/frames.hpp"

#include "friLBRState.h"

#include "lbr_fri_msgs/msg/lbr_command.hpp"
#include "lbr_fri_msgs/msg/lbr_state.hpp"

#include "damped_least_squares.hpp"

class EndController {
  using JointVector = Eigen::Vector<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS>;
  using CartesianVector = Eigen::Vector<double, 6>;

public:
  EndController(const std::string &robot_description,
                       const std::string &base_link = "link_0",
                       const std::string &end_effector_link = "link_ee",
                       const JointVector &dq_gains = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
                       const CartesianVector &dx_gains = {200, 200, 200, 200, 200, 200})
      : dq_gains_(dq_gains), dx_gains_(dx_gains) {
    if (!kdl_parser::treeFromString(robot_description, tree_)) {
      throw std::runtime_error("Failed to construct kdl tree from robot description.");
    }
    if (!tree_.getChain(base_link, end_effector_link, chain_)) {
      throw std::runtime_error("Failed to construct kdl chain from robot description.");
    }

    jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    jacobian_.resize(chain_.getNrOfJoints());
    q_.resize(chain_.getNrOfJoints());
  };

  const lbr_fri_msgs::msg::LBRCommand &update(const lbr_fri_msgs::msg::LBRState &lbr_state,
                                              const geometry_msgs::msg::TransformStamped &command_tf) {
    std::memcpy(q_.data.data(), lbr_state.measured_joint_position.data(),
                sizeof(double) * KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

    fk_solver_->JntToCart(q_, ee_frame_);
    ee_frame_expect_ = KDL::Frame(KDL::Rotation::Quaternion(command_tf.transform.rotation.x,
                                                           command_tf.transform.rotation.y,
                                                           command_tf.transform.rotation.z,
                                                           command_tf.transform.rotation.w),
                                  KDL::Vector(command_tf.transform.translation.x,
                                              command_tf.transform.translation.y,
                                              command_tf.transform.translation.z));
    auto pos_error= ee_frame_expect_.p - ee_frame_.p;
    auto rotation_error = ee_frame_.M.Inverse()* ee_frame_expect_.M;
    auto rotation_error_vec = ee_frame_.M * rotation_error.GetRot();
    end_error_ << pos_error.x(), pos_error.y(), pos_error.z(),
        rotation_error_vec.x(), rotation_error_vec.y(), rotation_error_vec.z();

    jacobian_solver_->JntToJac(q_, jacobian_);
    jacobian_inv_ = damped_least_squares(jacobian_.data);

    f_drive_= dx_gains_.asDiagonal() * end_error_;

    dq_ = dq_gains_.asDiagonal() * jacobian_inv_ * f_drive_;

    for (int i = 0; i < 7; i++) {
      dq_[i]=std::clamp(dq_[i],-dq_abs_max_[i],dq_abs_max_[i]);
      lbr_command_.joint_position[i] =
          lbr_state.measured_joint_position[i] + dq_[i] * lbr_state.sample_time;
    }

    LBRCommand_safe_check();
    return lbr_command_;
  };

  const KDL::Frame &get_end_frame(){
    return ee_frame_;
  }

protected:
  lbr_fri_msgs::msg::LBRCommand lbr_command_;
  KDL::Tree tree_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  KDL::Frame ee_frame_;
  KDL::Frame ee_frame_expect_;
  KDL::Jacobian jacobian_;
  Eigen::Matrix<double, KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 6> jacobian_inv_;
  KDL::JntArray q_;
  JointVector dq_;
  JointVector dq_abs_max_={5,5,5,5,10,10,10};
  JointVector dq_gains_;
  JointVector q_range_low_={-M_PI/180*165,-M_PI/180*115,-M_PI/180*165,-M_PI/180*115,-M_PI/180*165,-M_PI/180*115,-M_PI/180*170};
  JointVector q_range_up_={M_PI/180*165,M_PI/180*115,M_PI/180*165,M_PI/180*115,M_PI/180*165,M_PI/180*115,M_PI/180*170};
  CartesianVector dx_gains_;
  CartesianVector f_drive_;
  CartesianVector end_error_;

private:
  void LBRCommand_safe_check()
  {
    for (int i = 0; i < 7; i++) {
      if (lbr_command_.joint_position[i] < q_range_low_[i]) {
        lbr_command_.joint_position[i] = q_range_low_[i];
      } else if (lbr_command_.joint_position[i] > q_range_up_[i]) {
        lbr_command_.joint_position[i] = q_range_up_[i];
      }
    }
  }

};
#endif // LBR_DEMOS_FRI_ROS2_ADVANCED_CPP__END_CONTROLLER_HPP_
