#include "modified_rl_controllers/CustomAcController.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <pluginlib/class_list_macros.hpp>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>

namespace legged {

bool CustomAcController::loadModel(ros::NodeHandle &nh) {
    std::string policyFilePath;

    if (!nh.getParam("/policyFile", policyFilePath)) {
        ROS_ERROR_STREAM("Get policy path fail from param server, some error occur!");
        return false;
      }
    
    if (!nh.getParam("/debug", debug_)) {
        ROS_ERROR_STREAM("Get debug flag fail from param server, set to false!");
        debug_ = false;
      }

    policyFilePath_ = policyFilePath;
    ROS_INFO_STREAM("Load Onnx model from path : " << policyFilePath);
    
    onnxEnvPrt_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController"));
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetInterOpNumThreads(1);
    sessionPtr_ = std::make_unique<Ort::Session>(*onnxEnvPrt_, policyFilePath.c_str(), sessionOptions);
    inputNames_.clear();
    outputNames_.clear();
    inputShapes_.clear();
    outputShapes_.clear();

    Ort::AllocatorWithDefaultOptions allocator;
    
    for (int i = 0; i < sessionPtr_->GetInputCount(); i++) {
        inputNodeNameAllocatedStrings_.push_back(sessionPtr_->GetInputNameAllocated(i, allocator));
        inputNames_.push_back(inputNodeNameAllocatedStrings_[i].get());
        inputShapes_.push_back(sessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
      }
    for (int i = 0; i < sessionPtr_->GetOutputCount(); i++) {
    outputNodeNameAllocatedStrings_.push_back(sessionPtr_->GetOutputNameAllocated(i, allocator));
    outputNames_.push_back(outputNodeNameAllocatedStrings_[i].get());
    outputShapes_.push_back(sessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    }
    
    ROS_INFO_STREAM("Load Onnx model from successfully !!!");
    ROS_INFO_STREAM("Input node num: " << inputNames_.size());
    ROS_INFO_STREAM("Input node names: " << inputNames_[0]);
    ROS_INFO_STREAM("Output node num: " << outputNames_.size());
    ROS_INFO_STREAM("Output node names: " << outputNames_[0]);
    ROS_INFO_STREAM("Input shape: " << inputShapes_[0][0] << "x" << inputShapes_[0][1]);
    ROS_INFO_STREAM("Output shape: " << outputShapes_[0][0] << "x" << outputShapes_[0][1]);
    return true;
}

bool CustomAcController::loadRLCfg(ros::NodeHandle &nh) {
    RLRobotCfg::InitState& initState = robotCfg_.initState;
    RLRobotCfg::ControlCfg& controlCfg = robotCfg_.controlCfg;
    RLRobotCfg::ObsScales& obsScales = robotCfg_.obsScales;
    int error = 0;

    controlCfg.stiffness.resize(actuatedDofNum_, 0.0);
    controlCfg.damping.resize(actuatedDofNum_, 0.0);
    // Load leg init state
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_l1_joint", initState.leg_l1_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_l2_joint", initState.leg_l2_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_l3_joint", initState.leg_l3_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_l4_joint", initState.leg_l4_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_l5_joint", initState.leg_l5_joint));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_r1_joint", initState.leg_r1_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_r2_joint", initState.leg_r2_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_r3_joint", initState.leg_r3_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_r4_joint", initState.leg_r4_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_r5_joint", initState.leg_r5_joint));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/stiffness/leg_l1_joint", controlCfg.stiffness[0]));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/stiffness/leg_l2_joint", controlCfg.stiffness[1]));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/stiffness/leg_l3_joint", controlCfg.stiffness[2]));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/stiffness/leg_l4_joint", controlCfg.stiffness[3]));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/stiffness/leg_l5_joint", controlCfg.stiffness[4]));
    
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/stiffness/leg_r1_joint", controlCfg.stiffness[5]));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/stiffness/leg_r2_joint", controlCfg.stiffness[6]));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/stiffness/leg_r3_joint", controlCfg.stiffness[7]));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/stiffness/leg_r4_joint", controlCfg.stiffness[8]));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/stiffness/leg_r5_joint", controlCfg.stiffness[9]));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/damping/leg_l1_joint", controlCfg.damping[0]));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/damping/leg_l2_joint", controlCfg.damping[1]));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/damping/leg_l3_joint", controlCfg.damping[2]));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/damping/leg_l4_joint", controlCfg.damping[3]));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/damping/leg_l5_joint", controlCfg.damping[4]));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/damping/leg_r1_joint", controlCfg.damping[5]));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/damping/leg_r2_joint", controlCfg.damping[6]));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/damping/leg_r3_joint", controlCfg.damping[7]));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/damping/leg_r4_joint", controlCfg.damping[8]));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/damping/leg_r5_joint", controlCfg.damping[9]));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/action_scale", controlCfg.actionScale));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/decimation", controlCfg.decimation));

    // Load clip scales
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/clip_scales/clip_observations", robotCfg_.clipObs));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/clip_scales/clip_actions", robotCfg_.clipActions));

    // Load obs scales
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/lin_vel", obsScales.linVel));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/ang_vel", obsScales.angVel));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/dof_pos", obsScales.dofPos));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/dof_vel", obsScales.dofVel));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/quat", obsScales.quat));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/height_measurements", obsScales.heightMeasurements));

    // load size 
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/actions_size", actionsSize_));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/observations_size", observationSize_));
    actions_.resize(actionsSize_);
    observations_.resize(observationSize_);
    phase_.resize(2);

    phase_ << 0.0, M_PI;

    command_.x = 0.0;
    command_.y = 0.0;
    command_.yaw = 0.0;

    std::vector<scalar_t> defaultJointAngles{
        initState.leg_l1_joint,
        initState.leg_l2_joint,
        initState.leg_l3_joint,
        initState.leg_l4_joint,
        initState.leg_l5_joint,
        initState.leg_r1_joint,
        initState.leg_r2_joint,
        initState.leg_r3_joint,
        initState.leg_r4_joint,
        initState.leg_r5_joint
    };

    lastActions_.resize(actuatedDofNum_);
    defaultJointAngles_.resize(actuatedDofNum_);
    
    for (int i = 0; i < actuatedDofNum_; i++) {
        defaultJointAngles_(i, 0) = defaultJointAngles[i];
      }

    ROS_INFO_STREAM("Default joint angles: " << defaultJointAngles_.transpose());
    
    if (debug_){
        // Initialize CSV logging
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream oss;

        oss << logDir_ << std::put_time(&tm, "%Y%m%d_%H%M%S");
        std::string timestampedFolder = oss.str();
        
        mkdir(timestampedFolder.c_str(), 0755);
        ROS_INFO_STREAM("Created log folder: " << timestampedFolder);

        // Initialize observation CSV file
        std::string observationCsvFilePath_ = timestampedFolder + "/observation_data.csv";
        observationCsvFile_.open(observationCsvFilePath_, std::ios::out);
        if (observationCsvFile_.is_open()) {
            observationCsvFile_ << "loop_count,";
            observationCsvFile_ << "baseAngVel_x,baseAngVel_y,baseAngVel_z,";
            observationCsvFile_ << "IMUzaxis_x,IMUzaxis_y,IMUzaxis_z,";
            observationCsvFile_ << "deltaJointPos_0,deltaJointPos_1,deltaJointPos_2,deltaJointPos_3,deltaJointPos_4,deltaJointPos_5,deltaJointPos_6,deltaJointPos_7,deltaJointPos_8,deltaJointPos_9,";
            observationCsvFile_ << "jointVel_0,jointVel_1,jointVel_2,jointVel_3,jointVel_4,jointVel_5,jointVel_6,jointVel_7,jointVel_8,jointVel_9,";
            observationCsvFile_ << "lastActions_0,lastActions_1,lastActions_2,lastActions_3,lastActions_4,lastActions_5,lastActions_6,lastActions_7,lastActions_8,lastActions_9,";
            observationCsvFile_ << "command_x,command_y,command_yaw,";
            observationCsvFile_ << "gaitFrequency,gait";
            observationCsvFile_ << std::endl;
            ROS_INFO_STREAM("Observation CSV file opened for logging: " << observationCsvFilePath_);
        } else {
            ROS_ERROR_STREAM("Failed to open observation CSV file for logging: " << observationCsvFilePath_);
        }
        
        // Initialize action CSV file
        std::string actionCsvFilePath_ = timestampedFolder + "/action_data.csv";
        actionCsvFile_.open(actionCsvFilePath_, std::ios::out);
        if (actionCsvFile_.is_open()) {
            actionCsvFile_ << "loop_count";
            for (int i = 0; i < actions_.size(); i++) {
                actionCsvFile_ << "," << "action_" << i;
            }
            actionCsvFile_ << std::endl;
            ROS_INFO_STREAM("Action CSV file opened for logging: " << actionCsvFilePath_);
        } else {
            ROS_ERROR_STREAM("Failed to open action CSV file for logging: " << actionCsvFilePath_);
        }
    }

    return (error == 0);
}

void CustomAcController::computeActions() {
    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    std::vector<Ort::Value> inputValues;

    inputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, observations_.data(), observations_.size(),
                                inputShapes_[0].data(), inputShapes_[0].size()));

    Ort::RunOptions runOptions;
    std::vector<Ort::Value> outputValues = sessionPtr_->Run(runOptions, inputNames_.data(), inputValues.data(), 1, outputNames_.data(), 1);
    
    for (int i = 0; i < actionsSize_; i++) {
        actions_[i] = *(outputValues[0].GetTensorMutableData<tensor_element_t>() + i);
    }
}

void CustomAcController::computeObservation() {
    int generalizedCoordinatesNum = actuatedDofNum_ + 6;

    vector3_t zyx = rbdState_.segment(0, 3);
    matrix_t rot = getRotationMatrixFromZyxEulerAngles(zyx);
    vector3_t IMUzaxis(rot * vector3_t(0, 0, 1));
    matrix_t inverseRot = getRotationMatrixFromZyxEulerAngles(zyx).inverse();

    vector3_t baseLinVel = inverseRot * rbdState_.segment(generalizedCoordinatesNum, 3);
    vector3_t baseAngVel = rbdState_.segment(generalizedCoordinatesNum + 3, 3);
    std::cout << "baseLinVel x: " << baseLinVel << std::endl;
    vector3_t command(command_.x, command_.y, command_.yaw);

    vector_t jointPos = rbdState_.segment(6, actuatedDofNum_);

    vector_t jointVel = rbdState_.segment(generalizedCoordinatesNum + 6, actuatedDofNum_);

    vector_t lastActions(lastActions_);

    // Note: This values are hardcoded for walking mode
    scalar_t gait = 1.0;
    scalar_t gait_frequency = 1.75;
    scalar_t foot_height = 0.16;
    scalar_t phase_dt = 2 * M_PI * gait_frequency * (0.002 * robotCfg_.controlCfg.decimation);

    for (int i = 0; i < phase_.size(); ++i) {
        phase_[i] = std::fmod(phase_[i] + phase_dt + M_PI, 2 * M_PI) - M_PI;
    }
    
    vector_t observed_phase(4);
    observed_phase << cos(phase_[0]), cos(phase_[1]), sin(phase_[0]), sin(phase_[1]);

    // normalize data
    // RLRobotCfg::ObsScales& obsScales = robotCfg_.obsScales;
    // matrix_t commandScaler = Eigen::DiagonalMatrix<scalar_t, 3>(obsScales.linVel, obsScales.linVel, obsScales.angVel);
    
    // get observation
    vector_t obs(observationSize_); // 43

    obs << 
        baseLinVel,                    // 3
        baseAngVel,                    // 3
        IMUzaxis,                      // 3
        (jointPos - defaultJointAngles_) * robotCfg_.obsScales.dofPos, // 10 
        jointVel * robotCfg_.obsScales.dofVel,    // 10
        lastActions,                    // 10
        command,                        // 3
        observed_phase,                 // 4
        gait,                           // 1
        gait_frequency,                 // 1
        foot_height                     // 1
        ;

    // clip observation
    for (size_t i = 0; i < obs.size(); i++) {
        observations_[i] = static_cast<tensor_element_t>(obs(i));
        }

    scalar_t obsMin = -robotCfg_.clipObs;
    scalar_t obsMax = robotCfg_.clipObs;

    std::transform(observations_.begin(), observations_.end(), observations_.begin(),
                    [obsMin, obsMax](scalar_t x) { return std::max(obsMin, std::min(obsMax, x)); });
}

void CustomAcController::handleStandMode() {
    if (loopCount_ % robotCfg_.controlCfg.decimation == 0) {
        computeObservation();
        computeActions();
        scalar_t actionMin = -robotCfg_.clipActions;
        scalar_t actionMax = robotCfg_.clipActions;

        std::transform(actions_.begin(), actions_.end(), actions_.begin(),
        [actionMin, actionMax](scalar_t x) { return std::max(actionMin, std::min(actionMax, x)); });

        if (debug_) {
            // Log observation data to CSV
            if (observationCsvFile_.is_open()) {
                observationCsvFile_ << loopCount_;
                for (int i = 0; i < observations_.size(); i++) {
                    observationCsvFile_ << "," << observations_[i];
                }
                observationCsvFile_ << std::endl;
            }

            //log action data to CSV
            if (actionCsvFile_.is_open()) {
                actionCsvFile_ << loopCount_;
                for (int i = 0; i < actions_.size(); i++) {
                    actionCsvFile_ << "," << actions_[i];
                }
                actionCsvFile_ << std::endl;
            }
        }
    }

    for (int i = 0; i < hybridJointHandles_.size(); i++) {
        scalar_t pos_des = actions_[i] * robotCfg_.controlCfg.actionScale + defaultJointAngles_(i, 0);
        hybridJointHandles_[i].setCommand(pos_des, 0, robotCfg_.controlCfg.stiffness[i], robotCfg_.controlCfg.damping[i], 0);
        lastActions_(i, 0) = actions_[i];
    }
}

// void CustomAcController::move2standDetect() {
//     // Default implementation, override in derived classes
// }

// void CustomAcController::pushDetect() {
//     // Default implementation, override in derived classes
// }

// bool CustomAcController::push2standDetect() {
//     // Default implementation, override in derived classes
//     return false;
// }

// void CustomAcController::quickStop() {
//     // Default implementation, override in derived classes
// }

// bool CustomAcController::kinematic() {
//     // Default implementation, override in derived classes
//     return false;
// }

// bool CustomAcController::computeLegForwardKinematics(const std::string &leg_name, const vector_t &joint_positions, KDL::Frame &end_effector_frame) {
//     // Default implementation, override in derived classes
//     return false;
// }

} // namespace legged

// Export the controller class as a ROS plugin for dynamic loading
PLUGINLIB_EXPORT_CLASS(legged::CustomAcController, controller_interface::ControllerBase)
