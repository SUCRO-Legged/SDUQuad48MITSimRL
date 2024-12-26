#ifndef SDUOG_FSM_STATE_RL_H
#define SDUOG_FSM_STATE_RL_H
#include "FSM_State.h"
#include <cstring>
#include <experimental/filesystem>
#include "./common/include/Controllers/StateEstimatorContainer.h"
#include "./common/include/Controllers/LegController.h"

#include <cpu_provider_factory.h>
#include <onnxruntime_cxx_api.h>
#include <onnxruntime_session_options_config_keys.h>
#include <eigen3/Eigen/Dense>

#define NUM_LEGS 4
#define NUM_ACTIONS NUM_LEGS * 3
#define NUM_OBS 9 + NUM_ACTIONS * 3
#define NUM_LATENT 27
#define HIDDEN_SIZE 256

template <typename T>
class FSM_State_RL : public FSM_State<T>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FSM_State_RL(ControlFSMData<T> *_controlFSMData);
    ~FSM_State_RL();
    // Behavior to be carried out when entering a state
    void onEnter();

    // Run the normal behavior for the state
    void run();

    // Checks for any transition triggers
    FSM_StateName checkTransition();

    // Manages state specific transitions
    TransitionData<T> transition();

    // Behavior to be carried out when exiting a state
    void onExit();
    void logData();

private:
    // safety checker
    bool locomotionSafe();

    // Keep track of the control iterations
    int iter = 0;
    bool emergency_stop = false;

protected:
    Ort::Session encoderSession;
    Ort::Session actorSession;
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
    Ort::AllocatorWithDefaultOptions allocator;

    float *obsPtr = new float[NUM_OBS];
    float *latentPtr = new float[NUM_LATENT];
    float *hePtr = new float[HIDDEN_SIZE];
    float *cePtr = new float[HIDDEN_SIZE];
    float *haPtr = new float[HIDDEN_SIZE];
    float *caPtr = new float[HIDDEN_SIZE];
    float *actionsPtr = new float[NUM_ACTIONS];

    std::vector<Ort::Value> actor_input_tensors, actor_output_tensors;
    std::vector<Ort::Value> encoder_input_tensors, encoder_output_tensors;

    std::vector<const char *> encoder_input_node_names, encoder_output_node_names, actor_input_node_names, actor_output_node_names;
    std::vector<std::string> tmp_ei, tmp_eo, tmp_ai, tmp_ao;

    Eigen::Matrix<float, NUM_ACTIONS, 1> _targetDofPos, _defaultDofPos, _actions, _dofPos, _dofVel;
    Eigen::Matrix<float, 3, 1> _projGravity, _bodyAngVel;
    Eigen::Vector3f _command;

    // FR_hip FR_thigh FR_calf
    // FL_hip FL_thigh FL_calf
    const int srDofIndices[NUM_ACTIONS] = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8}; // To MITSim and Real Robot
    const int rlDofIndices[NUM_ACTIONS] = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8}; // To RL Training
    const Eigen::Matrix<float, 3, 1> _G = {0, 0, -1.0};

    LegControllerCommand<float> preCommands[NUM_LEGS];
    std::chrono::steady_clock::time_point begin_;
    std::chrono::steady_clock::time_point end_;

    virtual void reset();
    virtual void infer();
    virtual void getModelInfo();
    virtual void computeObservation();
};

#endif // SDUOG_FSM_STATE_RL_H