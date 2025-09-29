#pragma once // 确保文件在编译过程中只被包含一次

#include "modified_rl_controllers/RLControllerBase.h"
#include "modified_rl_controllers/utilities.h"

namespace legged
{
  using namespace ocs2;
  using namespace ocs2::legged_robot;

  class CustomAcController : public RLControllerBase
  {
    using tensor_element_t = float;

  public:
    CustomAcController() = default;

    ~CustomAcController() override = default;

  protected:
    bool loadModel(ros::NodeHandle &nh) override;
    bool loadRLCfg(ros::NodeHandle &nh) override;
    void computeActions() override;
    void computeObservation() override;
    void handleWalkMode() override;
    // void move2standDetect();
    // void pushDetect();
    // bool push2standDetect();
    // void quickStop();
    // bool kinematic();
    // bool computeLegForwardKinematics(const std::string &leg_name, const vector_t &joint_positions, KDL::Frame &end_effector_frame);

  private:
    // onnx policy model
    std::string policyFilePath_;
    std::shared_ptr<Ort::Env> onnxEnvPrt_;
    std::unique_ptr<Ort::Session> sessionPtr_;
    std::vector<const char *> inputNames_;
    std::vector<const char *> outputNames_;
    std::vector<Ort::AllocatedStringPtr> inputNodeNameAllocatedStrings_;
    std::vector<Ort::AllocatedStringPtr> outputNodeNameAllocatedStrings_;
    std::vector<std::vector<int64_t>> inputShapes_;
    std::vector<std::vector<int64_t>> outputShapes_;
    // int numHist_;
    // bool sw_mode_;
    // double cmd_threshold_;
    // bool quickStop_ = false;
    // int quickStop_counter_ = 0;
    // int est_cmd_counter_ = 0;
    // vector_t pushed_lin_vel_ = vector_t::Zero(4);
    // matrix_t pushed_angle_ = matrix_t::Zero(4, 2);
    // matrix_t stand_angle_ = matrix_t::Zero(2, 1);
    // matrix_t stand_pos_ = matrix_t::Zero(2, 2);
    // vector_t feet_delta_x_ = vector_t::Ones(50);
    // /*--------------------------------------------*/
    // vector_t feet_delta_y_ = vector_t::Ones(50);
    // vector_t feet_delta_z_ = vector_t::Ones(50);
    // /*--------------------------------------------*/
    // vector_t pushed_ang_vel_ = vector_t::Ones(4);
    // vector_t command_vel_ = vector_t::Zero(10);
    // vector_t joint_vel_ = vector_t::Ones(20);
    // bool move2standDetector_ = true;
    // int move2standDetector_counter_ = 0;
    // int move2standDetector_counter_thre_ = 0;
    // bool pushDetector_ = false;
    // int pushDetector_counter_ = 0;
    // int pushDetector_counter_thre_ = 0;
    // int joint_vel_thre_ = 0;
    // double ang_vel_threshold_;
    // double angle_threshold_;
    // double feet_delta_x_threshold_;
    // /*--------------------------------*/
    // double feet_delta_y_threshold_;
    // double feet_delta_z_threshold_;
    // /*--------------------------------*/
    // double pos_x_threshold_;
    // double pos_y_threshold_;
    // /*--------------------------------*/
    // double pos_z_threshold_;
    // /*--------------------------------*/

    vector3_t baseLinVel_;
    vector3_t basePosition_;
    vector_t lastActions_;
    vector_t defaultJointAngles_;

    // bool isfirstRecObs_{true};
    int actionsSize_;
    int observationSize_;
    std::vector<tensor_element_t> actions_;
    std::vector<tensor_element_t> observations_;

    // Ort::MemoryInfo memoryInfo;
    // Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> proprioHistoryBuffer_;
    // bool isfirstCompAct_{true};

    // bool filt_action_;   // 是否滤波
    // double cutoff_freq_; // 截止频率
    // double sample_rate_; // 采样率
    // double alpha_;       // 滤波器系数
    // vector_t filted_action_;
  };

} // namespace legged