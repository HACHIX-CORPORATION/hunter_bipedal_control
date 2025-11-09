#include "modified_rl_controllers/RLControllerBase.h"
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>

namespace legged {

bool RLControllerBase::init(hardware_interface::RobotHW *robotHw, ros::NodeHandle &controllerNH) {
	std::string taskFile;
	std::string urdfFile;
	std::string referenceFile;
	controllerNH.getParam("/urdfFile", urdfFile);
	controllerNH.getParam("/taskFile", taskFile);
	controllerNH.getParam("/referenceFile", referenceFile);	

	bool verbose = false;

	loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);
	setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
	CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());

	eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
	leggedInterface_->modelSettings().contactNames3DoF); 

	rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
	leggedInterface_->getCentroidalModelInfo()); 

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
	// rbdState_ = vector_t::Zero(2*(actuatedDofNum_ + 6));

	auto& initState = robotCfg_.initState; 

	standJointAngles_ << initState.L_CROTCH_R,
		initState.L_CROTCH_P,
		initState.L_KNEE_P,
		initState.L_TOE_P,
		initState.R_CROTCH_R,
		initState.R_CROTCH_P,
		initState.R_KNEE_P,
		initState.R_TOE_P;

	ROS_INFO_STREAM("[RLControllerBase] Stand joint angles: " << standJointAngles_.transpose());

	// Hardware interface
	auto* hybridJointInterface = robotHw->get<HybridJointInterface>();
	const std::vector<std::string> jointNames = {"L_CROTCH_R", "L_CROTCH_P", "L_KNEE_P", "L_TOE_P", "R_CROTCH_R", "R_CROTCH_P", "R_KNEE_P", "R_TOE_P"};
	std::string jointNamesStr;
	for (const auto& jointName : jointNames) {
		hybridJointHandles_.push_back(hybridJointInterface->getHandle(jointName)); 
		jointNamesStr += "\n"+ jointName;
	}
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

	scalar_t durationSecs = 4.0;
	standDuration_ = durationSecs * 500.0;
	standPercent_ += 1 / standDuration_;

	mode_ = Mode::LIE;
	loopCount_ = 0;
}

void RLControllerBase::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
	bool verbose) {
		leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
		leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void RLControllerBase::setupStateEstimate(const std::string& taskFile, bool verbose) {
	stateEstimate_ = std::make_shared<KalmanFilterEstimate>(leggedInterface_->getPinocchioInterface(),
	leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);

	dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
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
		  handleDefaultMode();
		  break;
		default:
		  ROS_ERROR_STREAM("Unexpected mode encountered: " << static_cast<int>(mode_));
		  break;
	  }
	
	loopCount_++;
}

void RLControllerBase::handleDefaultMode() {
	for (int j = 0; j < hybridJointHandles_.size(); j++) {
		scalar_t pos_des = currentJointAngles_[j] * (1 - standPercent_) + standJointAngles_(j) * standPercent_;
		hybridJointHandles_[j].setCommand(pos_des, 0, robotCfg_.controlCfg.stiffness[j], robotCfg_.controlCfg.damping[j], 0);
	}
	if (standPercent_ < 1) {
		standPercent_ += 1 / standDuration_;
	}
}

// void RLControllerBase::dynamicParamCallback(legged_debugger::TutorialsConfig &config, uint32_t level) {
// 	// TODO: Implement
// }

void RLControllerBase::updateStateEstimation(const ros::Time &time, const ros::Duration &period) {
	vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
	Eigen::Quaternion<scalar_t> quat;
	vector3_t angularVel, linearAccel;
	matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

	for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
		jointPos(i) = hybridJointHandles_[i].getPosition();
		jointVel(i) = hybridJointHandles_[i].getVelocity();
	}

	for (size_t i = 0; i < 4; ++i) {
		quat.coeffs()(i) = imuSensorHandles_.getOrientation()[i];
	}

	for (size_t i = 0; i < 3; ++i) {
		angularVel(i) = imuSensorHandles_.getAngularVelocity()[i];
		linearAccel(i) = imuSensorHandles_.getLinearAcceleration()[i];
	}

	for (size_t i = 0; i < 9; ++i) {
		orientationCovariance(i) = imuSensorHandles_.getOrientationCovariance()[i];
		angularVelCovariance(i) = imuSensorHandles_.getAngularVelocityCovariance()[i];
		linearAccelCovariance(i) = imuSensorHandles_.getLinearAccelerationCovariance()[i];
	}

	stateEstimate_->updateJointStates(jointPos, jointVel);
	stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
	rbdState_ = stateEstimate_->update(time, period);

}

void RLControllerBase::cmdVelCallback(const geometry_msgs::Twist &msg) {
	// command_.x = msg.linear.x;
	// command_.y = msg.linear.y;
	// command_.yaw = msg.angular.z;
}

void RLControllerBase::joyInfoCallback(const sensor_msgs::Joy &msg) {
	if (msg.buttons[7] == 1) { // Start button
		mode_ = Mode::LIE;
		ROS_INFO_STREAM("[RLControllerBase] Switching to LIE mode.");
	} else if (msg.buttons[4] == 1 && msg.buttons[0] == 1) { // LB + A button
		mode_ = Mode::STAND;
		ROS_INFO_STREAM("[RLControllerBase] Switching to STAND mode.");
	} else if (msg.buttons[4] == 1 && msg.buttons[1] == 1) { // LB + B button
		mode_ = Mode::WALK;
		ROS_INFO_STREAM("[RLControllerBase] Switching to WALK mode.");
	} else if (msg.buttons[4] == 1 && msg.buttons[2] == 1) { // LB + X button
		standPercent_ = 0.0;
		mode_ = Mode::DEFAULT;
		ROS_INFO_STREAM("[RLControllerBase] Switching to DEFAULT mode.");
	}
}

} // namespace legged

// Export the controller class as a ROS plugin
PLUGINLIB_EXPORT_CLASS(legged::RLControllerBase, controller_interface::ControllerBase)
