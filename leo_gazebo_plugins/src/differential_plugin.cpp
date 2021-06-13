#include <functional>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

class DifferentialPlugin : public gazebo::ModelPlugin {

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  /// Joint names
  std::string joint_a_name_, joint_b_name_;

  /// Force multiplier constant
  double force_constant_;

  /// Pointers to joints
  gazebo::physics::JointPtr joint_a_, joint_b_;

  /// Pointer to the model
  gazebo::physics::ModelPtr model_;

  /// Connects to physics update event
  gazebo::event::ConnectionPtr update_connection_;

public:
  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    // Create ros_node configured from sdf
    ros_node_ = gazebo_ros::Node::Get(_sdf);

    model_ = _parent;

    if (!_sdf->HasElement("jointA")) {
      RCLCPP_ERROR(
          ros_node_->get_logger(),
          "No jointA element present. DifferentialPlugin could not be loaded.");
      return;
    }
    joint_a_name_ = _sdf->GetElement("jointA")->Get<std::string>();

    if (!_sdf->HasElement("jointB")) {
      RCLCPP_ERROR(
          ros_node_->get_logger(),
          "No jointB element present. DifferentialPlugin could not be loaded.");
      return;
    }
    joint_b_name_ = _sdf->GetElement("jointB")->Get<std::string>();

    if (!_sdf->HasElement("forceConstant")) {
      RCLCPP_ERROR(ros_node_->get_logger(),
                   "No forceConstant element present. DifferentialPlugin could "
                   "not be loaded.");
      return;
    }
    force_constant_ = _sdf->GetElement("forceConstant")->Get<double>();

    joint_a_ = model_->GetJoint(joint_a_name_);
    if (!joint_a_) {
      RCLCPP_ERROR_STREAM(ros_node_->get_logger(),
                          "No joint named \""
                              << joint_a_name_
                              << "\". DifferentialPlugin could not be loaded.");
      return;
    }

    joint_b_ = model_->GetJoint(joint_b_name_);
    if (!joint_b_) {
      RCLCPP_ERROR_STREAM(ros_node_->get_logger(),
                          "No joint named \""
                              << joint_b_name_
                              << "\". DifferentialPlugin could not be loaded.");
      return;
    }

    update_connection_ = gazebo::event::Events::ConnectBeforePhysicsUpdate(
        std::bind(&DifferentialPlugin::OnUpdate, this));

    RCLCPP_INFO_STREAM(ros_node_->get_logger(),
                       "DifferentialPlugin loaded! Joint A: \""
                           << joint_a_name_ << "\", Joint B: \""
                           << joint_b_name_
                           << "\", Force Constant: " << force_constant_);
  }

  void OnUpdate() {
    double angle_diff = joint_a_->Position() - joint_b_->Position();
    joint_a_->SetForce(0, -angle_diff * force_constant_);
    joint_b_->SetForce(0, angle_diff * force_constant_);
  }
};

GZ_REGISTER_MODEL_PLUGIN(DifferentialPlugin)
