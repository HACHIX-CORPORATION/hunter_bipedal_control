#pragma once

// #include "modified_rl_controllers/utilities.h"
#include <robot_state_publisher/robot_state_publisher.h>

#include <controller_interface/multi_interface_controller.h>
#include <gazebo_msgs/ModelStates.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>

#include <controller_manager_msgs/SwitchController.h>
#include <sensor_msgs/Joy.h>

#include <onnxruntime_cxx_api.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

// #include "TutorialsConfig.h"
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/ParamDescription.h>

#include <atomic>
#include <map>
#include <memory>

namespace legged
{
  using namespace ocs2;
  using namespace legged_robot; 
  struct RLRobotCfg
  {
    struct ControlCfg
    {
      std::vector<float> stiffness;
      std::vector<float> damping;
      float actionScale;
      int decimation;
      float user_torque_limit;
      float user_power_limit;
    };

    struct InitState
    {
      // default joint angles
      scalar_t L_CROTCH_R;
      scalar_t L_CROTCH_P;
      scalar_t L_KNEE_P;
      scalar_t L_TOE_P;

      scalar_t R_CROTCH_R;
      scalar_t R_CROTCH_P;
      scalar_t R_KNEE_P;
      scalar_t R_TOE_P;
    };

    struct ObsScales
    {
      scalar_t linVel;
      scalar_t angVel;
      scalar_t dofPos;
      scalar_t dofVel;
      scalar_t quat;
    };

    scalar_t clipActions;
    scalar_t clipObs;

    InitState initState;
    ObsScales obsScales;
    ControlCfg controlCfg;
  };

  struct JoyInfo
  {
    float axes[8];
    int buttons[11];
  };

  struct Command
  {
    std::atomic<scalar_t> x;
    std::atomic<scalar_t> y;
    std::atomic<scalar_t> yaw;
  };

  class RLControllerBase : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                                                 ContactSensorInterface>
  {
  public:
    enum class Mode : uint8_t
    {
      LIE,
      STAND,
      WALK,
      DEFAULT
    };

    RLControllerBase() = default;
    virtual ~RLControllerBase() = default;
    virtual bool init(hardware_interface::RobotHW *robotHw, ros::NodeHandle &controllerNH);
    virtual void starting(const ros::Time &time);
    virtual void update(const ros::Time &time, const ros::Duration &period);

    virtual bool loadModel(ros::NodeHandle &nh) { return false; };
    virtual bool loadRLCfg(ros::NodeHandle &nh) { return false; };
    virtual void computeActions() {};
    virtual void computeObservation() {};

    virtual void handleLieMode();
    virtual void handleStandMode();
    virtual void handleDefaultMode();
    virtual void handleWalkMode();

    // std::unique_ptr<dynamic_reconfigure::Server<legged_debugger::TutorialsConfig>> server_ptr_;
    // void dynamicParamCallback(legged_debugger::TutorialsConfig &config, uint32_t level);

  protected:
    virtual void updateStateEstimation(const ros::Time &time, const ros::Duration &period);
    virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                      bool verbose);
    virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

    void cmdVelCallback(const geometry_msgs::Twist &msg);
    void joyInfoCallback(const sensor_msgs::Joy &msg);

    Mode mode_;
    int64_t loopCount_;
    Command command_;
    RLRobotCfg robotCfg_{};

    std::shared_ptr<StateEstimateBase> stateEstimate_;
    std::shared_ptr<LeggedInterface> leggedInterface_;
    std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;

    JoyInfo joyInfo;

    vector_t rbdState_;
    vector_t estimatedRbdState_;
    std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;
    
    // hardware interface
    std::vector<HybridJointHandle> hybridJointHandles_;
    hardware_interface::ImuSensorHandle imuSensorHandles_;

    ros::Subscriber cmdVelSub_;
    ros::Subscriber joyInfoSub_;

    int actuatedDofNum_ = 8;

  private:
    std::vector<scalar_t> currentJointAngles_;
    vector_t standJointAngles_;
    // vector_t lieJointAngles_;

    scalar_t standPercent_;
    scalar_t standDuration_;
  };
} // namespace legged