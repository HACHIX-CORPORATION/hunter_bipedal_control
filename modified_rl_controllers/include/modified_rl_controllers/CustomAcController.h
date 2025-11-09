#pragma once // 确保文件在编译过程中只被包含一次

#include "modified_rl_controllers/RLControllerBase.h"
// #include "modified_rl_controllers/utilities.h"
#include <fstream>

namespace legged
{
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
    void handleStandMode() override;

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
   
    vector_t lastActions_;
    vector_t defaultJointAngles_;
    vector_t phase_;

    int actionsSize_;
    int observationSize_;
    std::vector<tensor_element_t> actions_;
    std::vector<tensor_element_t> observations_;

    // CSV logging
    bool debug_ = false;
    std::ofstream observationCsvFile_;
    std::ofstream actionCsvFile_;
    std::string logDir_ = "/home/ubuntu/Work/hunter_bidepal_control/src/hunter_bipedal_control/modified_rl_controllers/logs/";
  };

} // namespace legged