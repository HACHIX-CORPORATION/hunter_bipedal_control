#include "modified_rl_controllers/RLControllerBase.h"
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>

namespace legged {

bool RLControllerBase::init(hardware_interface::RobotHW *robotHw, ros::NodeHandle &controllerNH) {
	if (!loadModel(controllerNH)) {
		ROS_ERROR_STREAM("[RLControllerBase] Failed to load the model. Ensure the path is correct and accessible.");
		return false;
	}
	ROS_INFO_STREAM("[RLControllerBase] Model loaded successfully.");
	if (!loadRLCfg(controllerNH)) { 
		
		ROS_ERROR_STREAM("[RLControllerBase] Failed to load the rl config. Ensure the yaml is correct and accessible.");
		return false;
	}
	ROS_INFO_STREAM("[RLControllerBase] RL config loaded successfully.");

	// Get default stand joint angles
	ROS_INFO_STREAM("[RLControllerBase] actuatedDofNum: " << actuatedDofNum_);
	standJointAngles_.resize(actuatedDofNum_);
	rbdState_ = vector_t::Zero(2*(actuatedDofNum_ + 6));

	auto& initState = robotCfg_.initState; 

	standJointAngles_ << initState.leg_l1_joint, 
	initState.leg_l2_joint, 
	initState.leg_l3_joint, 
	initState.leg_l4_joint, 
	initState.leg_l5_joint,
	initState.leg_r1_joint,
	initState.leg_r2_joint,
	initState.leg_r3_joint,
	initState.leg_r4_joint,
	initState.leg_r5_joint;
	ROS_INFO_STREAM("[RLControllerBase] Stand joint angles: " << standJointAngles_.transpose());

	// Hardware interface
	auto* hybridJointInterface = robotHw->get<HybridJointInterface>();
	const std::vector<std::string> jointNames = {"leg_l1_joint", "leg_l2_joint", "leg_l3_joint", "leg_l4_joint", "leg_l5_joint", "leg_r1_joint", "leg_r2_joint", "leg_r3_joint", "leg_r4_joint", "leg_r5_joint"};
	std::string jointNamesStr;
	for (const auto& jointName : jointNames) {
		hybridJointHandles_.push_back(hybridJointInterface->getHandle(jointName)); 
		jointNamesStr += "\n"+ jointName;
	}
	ROS_INFO_STREAM("[RLControllerBase] Joint names: " << jointNamesStr);
;
	imuSensorHandles_ = robotHw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");

	// Register callbacks
	cmdVelSub_ = controllerNH.subscribe("/cmd_vel", 1, &RLControllerBase::cmdVelCallback, this);
	joyInfoSub_ = controllerNH.subscribe("/joy", 1, &RLControllerBase::joyInfoCallback, this);
	ROS_INFO_STREAM("[RLControllerBase] successfully initialized.");
	return true;
}

void RLControllerBase::starting(const ros::Time &time) {
	updateStateEstimation(time, ros::Duration(0.002));

	for (auto& hybridJointHandle : hybridJointHandles_) {
		currentJointAngles_.push_back(hybridJointHandle.getPosition());
	}

	scalar_t durationSecs = 2.0;
	standDuration_ = durationSecs * 1000.0;
	standPercent_ += 1 / standDuration_;

	mode_ = Mode::LIE;
	loopCount_ = 0;
}

void RLControllerBase::update(const ros::Time &time, const ros::Duration &period) {
	updateStateEstimation(time, period);

	switch (mode_) {
		case Mode::LIE:
		  handleLieMode();
		  break;
		case Mode::STAND:
		  handleStandMode();
		  break;
		case Mode::WALK:
		  handleWalkMode();
		  break;
		case Mode::DEFAULT:
		  handleDefautMode();
		  break;
		default:
		  ROS_ERROR_STREAM("Unexpected mode encountered: " << static_cast<int>(mode_));
		  break;
	  }
	
	loopCount_++;
}

void RLControllerBase::handleLieMode() {
	if (standPercent_ < 1) {
		for (int j = 0; j < hybridJointHandles_.size(); j++) {
		  scalar_t pos_des = currentJointAngles_[j] * (1 - standPercent_) + standJointAngles_(j) * standPercent_;
		//   hybridJointHandles_[j].setCommand(pos_des, 0, robotCfg_.controlCfg.stiffness[j], robotCfg_.controlCfg.damping[j], 0);
		}
		standPercent_ += 1 / standDuration_;
	  } else {
		mode_ = Mode::WALK;
	  }
}

void RLControllerBase::handleStandMode() {
	if (loopCount_ > 5000) {
		mode_ = Mode::WALK;
	  }
}

void RLControllerBase::handleDefautMode() {
	//  TODO: Implement default mode behavior
	ROS_INFO_STREAM("[RLControllerBase] Default mode is active. No specific behavior implemented.");
}

// void RLControllerBase::dynamicParamCallback(legged_debugger::TutorialsConfig &config, uint32_t level) {
// 	// TODO: Implement
// }

void RLControllerBase::updateStateEstimation(const ros::Time &time, const ros::Duration &period) {
	int generalizedCoordinatesNum = actuatedDofNum_ + 6;
	vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
	Eigen::Quaternion<scalar_t> quat;
	vector3_t angularVel;

	for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
		jointPos(i) = hybridJointHandles_[i].getPosition();
		jointVel(i) = hybridJointHandles_[i].getVelocity();
	}

	for (size_t i = 0; i < 4; ++i) {
	quat.coeffs()(i) = imuSensorHandles_.getOrientation()[i];
	}

	for (size_t i = 0; i < 3; ++i) {
		angularVel(i) = imuSensorHandles_.getAngularVelocity()[i];
	}
	
    rbdState_.segment(0, 3) = quatToZyx(quat);
	rbdState_.segment(3, 3) = vector3_t::Zero(); // TODO: estimate base position
	rbdState_.segment(6, actuatedDofNum_) = jointPos;

	rbdState_.segment(generalizedCoordinatesNum, 3) = vector3_t::Zero(); // TODO: estimate base linear velocity
	rbdState_.segment(generalizedCoordinatesNum + 3, 3) = angularVel;
	rbdState_.segment(generalizedCoordinatesNum + 6, actuatedDofNum_) = jointVel;
}

void RLControllerBase::cmdVelCallback(const geometry_msgs::Twist &msg) {
	command_.x = msg.linear.x;
	command_.y = msg.linear.y;
	command_.yaw = msg.angular.z;
}

void RLControllerBase::joyInfoCallback(const sensor_msgs::Joy &msg) {
	//TODO: Implement
}

} // namespace legged

// Export the controller class as a ROS plugin
PLUGINLIB_EXPORT_CLASS(legged::RLControllerBase, controller_interface::ControllerBase)
