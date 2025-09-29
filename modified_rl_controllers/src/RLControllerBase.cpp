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
	ROS_WARN_STREAM("[RLControllerBase] verbose: " << verbose);
	setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
	ROS_WARN_STREAM("[RLControllerBase] Legged interface setup complete.");
	CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
	ROS_WARN_STREAM("[RLControllerBase] Pinocchio mapping setup complete.");
	eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
	leggedInterface_->modelSettings().contactNames3DoF); 
	ROS_WARN_STREAM("[RLControllerBase] End-effector kinematics setup complete.");
	rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
	leggedInterface_->getCentroidalModelInfo()); 
	ROS_WARN_STREAM("[RLControllerBase] RBD conversions setup complete.");
	if (!loadModel(controllerNH)) {
		ROS_ERROR_STREAM("[RLControllerBase] Failed to load the model. Ensure the path is correct and accessible.");
		return false;
	}
	ROS_WARN_STREAM("[RLControllerBase] Model loaded successfully.");
	if (!loadRLCfg(controllerNH)) { 
		
		ROS_ERROR_STREAM("[RLControllerBase] Failed to load the rl config. Ensure the yaml is correct and accessible.");
		return false;
	}
	ROS_WARN_STREAM("[RLControllerBase] RL config loaded successfully.");

	// Get default stand joint angles
	ROS_INFO_STREAM("[RLControllerBase] actuatedDofNum: " << leggedInterface_->getCentroidalModelInfo().actuatedDofNum);
	standJointAngles_.resize(leggedInterface_->getCentroidalModelInfo().actuatedDofNum);
	
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
	ROS_INFO_STREAM("[RLControllerBase] Default stand joint angles: " << standJointAngles_.transpose());

	// Hardware interface
	auto* hybridJointInterface = robotHw->get<HybridJointInterface>();
	ROS_WARN_STREAM("[RLControllerBase] Hybrid joint interface obtained.");
	const auto& jointNames = leggedInterface_->modelSettings().jointNames;
	
	std::string jointNamesStr;
	for (const auto& jointName : jointNames) {
		hybridJointHandles_.push_back(hybridJointInterface->getHandle(jointName)); 
		jointNamesStr += "\n"+ jointName;
	}
	ROS_INFO_STREAM("[RLControllerBase] Joint names: " << jointNamesStr);

	ROS_WARN_STREAM("[RLControllerBase] Hybrid joint handles obtained.");
	imuSensorHandles_ = robotHw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");
	ROS_WARN_STREAM("[RLControllerBase] IMU sensor handle obtained.");
	// State estimate 
	setupStateEstimate(taskFile, verbose);

	// Register callbacks
	cmdVelSub_ = controllerNH.subscribe("/cmd_vel", 1, &RLControllerBase::cmdVelCallback, this);
	joyInfoSub_ = controllerNH.subscribe("/joy", 1, &RLControllerBase::joyInfoCallback, this);
	ROS_WARN_STREAM("[RLControllerBase] Subscribers set up successfully.");
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
		  handleDefautMode();
		  break;
		default:
		  ROS_ERROR_STREAM("Unexpected mode encountered: " << static_cast<int>(mode_));
		  break;
	  }
	
	loopCount_++;
}

void RLControllerBase::handleLieMode() {
	// if (standPercent_ < 1) {
	// 	for (int j = 0; j < hybridJointHandles_.size(); j++) {
	// 	  scalar_t pos_des = currentJointAngles_[j] * (1 - standPercent_) + standJointAngles_(j) * standPercent_;
	// 	  hybridJointHandles_[j].setCommand(pos_des, 0, robotCfg_.controlCfg.stiffness[j], robotCfg_.controlCfg.damping[j], 0);
	// 	}
	// 	standPercent_ += 1 / standDuration_;
	//   } else {
	// 	for (int j = 0; j < hybridJointHandles_.size(); j++) {
	// 		scalar_t pos_des = standJointAngles_(j);
	// 		hybridJointHandles_[j].setCommand(pos_des, 0, robotCfg_.controlCfg.stiffness[j], robotCfg_.controlCfg.damping[j], 0);		
	// 	}
	//   }
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
