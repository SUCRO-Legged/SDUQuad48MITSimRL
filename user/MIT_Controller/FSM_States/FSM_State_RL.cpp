#include "FSM_State_RL.h"
#include <iomanip>
#include <iostream>
#include <unistd.h>
#include <fstream>

template <typename T>
FSM_State_RL<T>::FSM_State_RL(ControlFSMData<T> *_controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::RL_TEST, "RL_JOINT_PD"),
      encoderSession(nullptr), actorSession(nullptr)
{
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    std::cout << "ONNX Setup Joint Position Control learned by Reinforcement Learning!!!" << std::endl;
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

    std::string actorPath = std::string(get_current_dir_name()) + "/../models/alarm-" + std::to_string(static_cast<int>(std::round(this->_data->userParameters->policyDirNum))) + "/actor.onnx";
    std::string encoderPath = std::string(get_current_dir_name()) + "/../models/alarm-" + std::to_string(static_cast<int>(std::round(this->_data->userParameters->policyDirNum))) + "/encoder.onnx";
    std::cout << "load actor model from " << actorPath << std::endl;
    std::cout << "load encoder model from " << encoderPath << std::endl;

    // init ONNX Runtime
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ONNXRuntimeInference");
    // Ort::Env env(ORT_LOGGING_LEVEL_VERBOSE, "ONNXRuntimeInference");

    // create Session Options
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    // OrtSessionOptionsAppendExecutionProvider_CUDA(session_options, 0);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    // load ONNX model
    encoderSession = Ort::Session(env, encoderPath.c_str(), session_options);
    actorSession = Ort::Session(env, actorPath.c_str(), session_options);

    if (!actorSession || !encoderSession)
        std::cerr << "Failed to create session!" << std::endl;
    else
        std::cout << "session created ..." << std::endl;

    getModelInfo();
    // reset
    reset();

    for (int i = 0; i < 10; i++)
    {
        std::chrono::steady_clock::time_point _end = std::chrono::steady_clock::now();
        infer();
        std::chrono::steady_clock::time_point _begin = std::chrono::steady_clock::now();
        if (i == 0)
            std::cout << "[INIT]: " << "The policy first forward time duration = " << std::chrono::duration_cast<std::chrono::microseconds>(_begin - _end).count() << " us" << std::endl;
        if (i >= 9)
            std::cout << "[INIT]: " << "The policy last forward time duration = " << std::chrono::duration_cast<std::chrono::microseconds>(_begin - _end).count() << " us" << std::endl;
    }

    std::cout << "[INFO]: FSM_State_RL is ok!" << std::endl;
    std::cout << std::endl;
}

template <typename T>
void FSM_State_RL<T>::getModelInfo()
{
    // encoder
    for (size_t i = 0; i < encoderSession.GetInputCount(); i++)
        tmp_ei.emplace_back(encoderSession.GetInputNameAllocated(i, allocator).get());
    for (const auto &str : tmp_ei)
        if (!str.empty())
            encoder_input_node_names.push_back(str.c_str());
    for (size_t i = 0; i < encoderSession.GetOutputCount(); i++)
        tmp_eo.emplace_back(encoderSession.GetOutputNameAllocated(i, allocator).get());
    for (const auto &str : tmp_eo)
        if (!str.empty())
            encoder_output_node_names.push_back(str.c_str());

    // actor
    for (size_t i = 0; i < actorSession.GetInputCount(); i++)
        tmp_ai.emplace_back(actorSession.GetInputNameAllocated(i, allocator).get());
    for (const auto &str : tmp_ai)
        if (!str.empty())
            actor_input_node_names.push_back(str.c_str());
    for (size_t i = 0; i < actorSession.GetOutputCount(); i++)
        tmp_ao.emplace_back(actorSession.GetOutputNameAllocated(i, allocator).get());
    for (const auto &str : tmp_ao)
        if (!str.empty())
            actor_output_node_names.push_back(str.c_str());
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_RL<T>::reset()
{
    // reset observations
    memset(obsPtr, 0, sizeof(float) * NUM_OBS);
    // reset latent representation
    memset(latentPtr, 0, sizeof(float) * NUM_LATENT);
    // reset hidden and cell state of encoder and actor
    memset(hePtr, 0, sizeof(float) * HIDDEN_SIZE);
    memset(cePtr, 0, sizeof(float) * HIDDEN_SIZE);
    memset(haPtr, 0, sizeof(float) * HIDDEN_SIZE);
    memset(caPtr, 0, sizeof(float) * HIDDEN_SIZE);
    // reset actions
    memset(actionsPtr, 0, sizeof(float) * NUM_ACTIONS);

    std::cout << "[INFO]: Reset Hidden State and Cell State!" << std::endl;
}

template <typename T>
void FSM_State_RL<T>::onEnter()
{
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    std::cout << "ONNX:::Start Joint Position Control learned by Reinforcement Learning" << std::endl;
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    // Default is to not transition
    this->nextStateName = this->stateName;

    // Reset the transition data
    this->transitionData.zero();

    // reset
    reset();

    _defaultDofPos << this->_data->userParameters->init_pos[0], this->_data->userParameters->init_pos[1], this->_data->userParameters->init_pos[2],
        this->_data->userParameters->init_pos[0], this->_data->userParameters->init_pos[1], this->_data->userParameters->init_pos[2],
        this->_data->userParameters->init_pos[0], this->_data->userParameters->init_pos[1], this->_data->userParameters->init_pos[2],
        this->_data->userParameters->init_pos[0], this->_data->userParameters->init_pos[1], this->_data->userParameters->init_pos[2];

    _dofPos << this->_data->_legController->datas[1].q,
        this->_data->_legController->datas[0].q,
        this->_data->_legController->datas[3].q,
        this->_data->_legController->datas[2].q;

    _dofVel << this->_data->_legController->datas[1].qd,
        this->_data->_legController->datas[0].qd,
        this->_data->_legController->datas[3].qd,
        this->_data->_legController->datas[2].qd;

    _actions.setZero();

    for (int i = 0; i < NUM_ACTIONS; i++)
        _targetDofPos[i] = _defaultDofPos[i];

    for (int i = 0; i < NUM_LEGS; i++)
        preCommands[i].zero();

    this->_data->_legController->_legsEnabled = true;
    for (int leg(0); leg < NUM_LEGS; ++leg)
    {
        for (int jidx(0); jidx < 3; ++jidx)
        {
            this->_data->_legController->commands[leg].qDes[jidx] = _targetDofPos(leg * 3 + jidx);
            this->_data->_legController->commands[leg].qdDes[jidx] = 0.;
        }
        this->_data->_legController->commands[leg].kpJoint = Vec3<T>(this->_data->userParameters->rlKp[0], this->_data->userParameters->rlKp[1], this->_data->userParameters->rlKp[2]).asDiagonal();
        this->_data->_legController->commands[leg].kdJoint = Vec3<T>(this->_data->userParameters->rlKd[0], this->_data->userParameters->rlKd[1], this->_data->userParameters->rlKd[2]).asDiagonal();

        preCommands[leg] = this->_data->_legController->commands[leg];
    }
    emergency_stop = false;
}

template <typename T>
void FSM_State_RL<T>::infer()
{
    // infer
    encoder_input_tensors.clear();
    encoder_output_tensors.clear();
    actor_input_tensors.clear();
    actor_output_tensors.clear();

    encoder_input_tensors.emplace_back(Ort::Value::CreateTensor<float>(
        memory_info,
        obsPtr, NUM_OBS,
        encoderSession.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape().data(),
        encoderSession.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape().size()));
    encoder_input_tensors.emplace_back(Ort::Value::CreateTensor<float>(
        memory_info,
        hePtr, HIDDEN_SIZE,
        encoderSession.GetInputTypeInfo(1).GetTensorTypeAndShapeInfo().GetShape().data(),
        encoderSession.GetInputTypeInfo(1).GetTensorTypeAndShapeInfo().GetShape().size()));
    encoder_input_tensors.emplace_back(Ort::Value::CreateTensor<float>(
        memory_info,
        cePtr, HIDDEN_SIZE,
        encoderSession.GetInputTypeInfo(2).GetTensorTypeAndShapeInfo().GetShape().data(),
        encoderSession.GetInputTypeInfo(2).GetTensorTypeAndShapeInfo().GetShape().size()));

    encoder_output_tensors = encoderSession.Run(Ort::RunOptions{nullptr},
                                                encoder_input_node_names.data(),
                                                encoder_input_tensors.data(),
                                                encoder_input_tensors.size(),
                                                encoder_output_node_names.data(),
                                                encoder_output_node_names.size());

    // std::cout << "[INFO]: encoder session inference done" << std::endl;

    latentPtr = encoder_output_tensors[0].GetTensorMutableData<float>();
    hePtr = encoder_output_tensors[1].GetTensorMutableData<float>();
    cePtr = encoder_output_tensors[2].GetTensorMutableData<float>();

    actor_input_tensors.emplace_back(Ort::Value::CreateTensor<float>(
        memory_info,
        obsPtr, NUM_OBS,
        actorSession.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape().data(),
        actorSession.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape().size()));
    actor_input_tensors.emplace_back(Ort::Value::CreateTensor<float>(
        memory_info,
        latentPtr, NUM_LATENT,
        actorSession.GetInputTypeInfo(1).GetTensorTypeAndShapeInfo().GetShape().data(),
        actorSession.GetInputTypeInfo(1).GetTensorTypeAndShapeInfo().GetShape().size()));
    actor_input_tensors.emplace_back(Ort::Value::CreateTensor<float>(
        memory_info,
        haPtr, HIDDEN_SIZE,
        actorSession.GetInputTypeInfo(2).GetTensorTypeAndShapeInfo().GetShape().data(),
        actorSession.GetInputTypeInfo(2).GetTensorTypeAndShapeInfo().GetShape().size()));
    actor_input_tensors.emplace_back(Ort::Value::CreateTensor<float>(
        memory_info,
        caPtr, HIDDEN_SIZE,
        actorSession.GetInputTypeInfo(3).GetTensorTypeAndShapeInfo().GetShape().data(),
        actorSession.GetInputTypeInfo(3).GetTensorTypeAndShapeInfo().GetShape().size()));

    actor_output_tensors = actorSession.Run(
        Ort::RunOptions{nullptr},       // run options
        actor_input_node_names.data(),  // input names
        actor_input_tensors.data(),     // input values
        actor_input_tensors.size(),     // input values count
        actor_output_node_names.data(), // output names
        actor_output_node_names.size()  // output names count
    );
    // std::cout << "[INFO]: actor session inference done" << std::endl;

    actionsPtr = actor_output_tensors[0].GetTensorMutableData<float>();
    haPtr = actor_output_tensors[1].GetTensorMutableData<float>();
    caPtr = actor_output_tensors[2].GetTensorMutableData<float>();
    // std::copy(actionsPtr, actionsPtr + actor_output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount(), std::ostream_iterator<float>(std::cout, " "));
    // std::cout << std::endl;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_RL<T>::run()
{
    if (std::acos(this->_data->_stateEstimator->getResult().rBody.transpose().row(2)(2)) > 3.1415 * this->_data->userParameters->emergency_orientation / 180.)
    {
        static int tmm = 0;
        tmm++;
        if (tmm % 10000 == 1)
        {
            std::cout << "Orientation is in bad condition!!!" << std::endl;
            std::cout << "RL Joint PD Control mode is denied!!!" << std::endl;
        }
        emergency_stop = true;
        return;
    }

    if (emergency_stop)
        return;

    end_ = std::chrono::steady_clock::now();

    if (std::chrono::duration_cast<std::chrono::microseconds>(end_ - begin_).count() > int(this->_data->userParameters->policy_dt * 1000000))
    {
        begin_ = std::chrono::steady_clock::now();
    }
    else
    {
        for (int leg(0); leg < NUM_LEGS; ++leg)
        {
            this->_data->_legController->commands[leg] = preCommands[leg];
        }
        return;
    }

    if (this->_data->controlParameters->use_rc)
    {
        _command(0) = this->_data->_desiredStateCommand->rcCommand->v_des[0];
        _command(1) = this->_data->_desiredStateCommand->rcCommand->v_des[1];
        _command(2) = this->_data->_desiredStateCommand->rcCommand->omega_des[2];
        if (_command.norm() < 0.1)
            _command.setZero();
    }
    else
    {
        _command(0) = 3.2 * this->_data->_desiredStateCommand->gamepadCommand->leftStickAnalog(1);
        _command(1) = -this->_data->_desiredStateCommand->gamepadCommand->leftStickAnalog(0);
        _command(2) = -2 * this->_data->_desiredStateCommand->gamepadCommand->rightStickAnalog(0);
        _command(2) /= fmax(1., std::sqrt(std::abs(_command(0))));
        if (_command.norm() < 0.3)
            _command.setZero();
    }
    // std::cout << "_command(0):" << _command(0) << std::endl;

    // get obs
    computeObservation();
    // infer
    infer();

    // step
    // change joint idx
    for (int i = 0; i < NUM_ACTIONS; i++)
    {
        _actions[i] = actionsPtr[i];
        _targetDofPos[i] = _actions[srDofIndices[i]] * this->_data->userParameters->actionScale + _defaultDofPos[i];
    }

    this->_data->_legController->_legsEnabled = true;
    for (int leg(0); leg < NUM_LEGS; ++leg)
    {
        for (int jidx(0); jidx < 3; ++jidx)
        {
            this->_data->_legController->commands[leg].qDes[jidx] = _targetDofPos(leg * 3 + jidx);
            this->_data->_legController->commands[leg].qdDes[jidx] = 0.;
        }
        this->_data->_legController->commands[leg].kpJoint = Vec3<T>(this->_data->userParameters->rlKp[0], this->_data->userParameters->rlKp[1], this->_data->userParameters->rlKp[2]).asDiagonal();
        this->_data->_legController->commands[leg].kdJoint = Vec3<T>(this->_data->userParameters->rlKd[0], this->_data->userParameters->rlKd[1], this->_data->userParameters->rlKd[2]).asDiagonal();
        preCommands[leg] = this->_data->_legController->commands[leg];
    }
    if (this->_data->userParameters->logData > 0)
        logData();
}

template <typename T>
FSM_StateName FSM_State_RL<T>::checkTransition()
{
    this->nextStateName = this->stateName;

    iter++;

    if (locomotionSafe())
    {
        // Switch FSM control mode
        switch ((int)this->_data->controlParameters->control_mode)
        {
        case K_RL_TEST:
            break;

        case K_PASSIVE:
            this->nextStateName = FSM_StateName::PASSIVE;
            this->transitionDuration = 0.0;
            break;

        case K_RECOVERY_STAND:
            this->nextStateName = FSM_StateName::RECOVERY_STAND;
            this->transitionDuration = 0.0;
            break;

        default:
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                      << K_RL_TEST << " to "
                      << this->_data->controlParameters->control_mode << std::endl;
        }
    }
    else
    {
        this->nextStateName = FSM_StateName::PASSIVE;
        this->transitionDuration = 0.;
        printf("BILLCHEN RINTF: locomotion force to Passive\n");
    }

    // Get the next state
    return this->nextStateName;
}

template <typename T>
TransitionData<T> FSM_State_RL<T>::transition()
{
    // Switch FSM control mode
    switch (this->nextStateName)
    {
    case FSM_StateName::PASSIVE:
        this->turnOffAllSafetyChecks();
        this->transitionData.done = true;
        break;

    case FSM_StateName::RECOVERY_STAND:
        this->transitionData.done = true;
        break;

    default:
        std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                  << K_RL_TEST << " to "
                  << this->_data->controlParameters->control_mode << std::endl;
    }
    // Finish transition
    this->transitionData.done = true;

    // Return the transition data to the FSM
    return this->transitionData;
}

// safety checker
template <typename T>
bool FSM_State_RL<T>::locomotionSafe()
{
    auto &seResult = this->_data->_stateEstimator->getResult();
    T max_roll = 50;
    T max_pitch = 50;

    if (std::fabs(seResult.rpy[0]) > ori::deg2rad(max_roll))
    {
        printf("Unsafe locomotion: roll is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[0]), max_roll);
        return false;
    }
    if (std::fabs(seResult.rpy[1]) > ori::deg2rad(max_pitch))
    {
        printf("Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[1]), max_pitch);
        return false;
    }

    for (int leg = 0; leg < 4; leg++)
    {
        auto p_leg = this->_data->_legController->datas[leg].p;

        if (p_leg[1] > 0.31)
        {
            printf("Unsafe locomotion: leg %d's y-position is bad (%.3f m)\n", leg, p_leg[1]);
            return false;
        }

        auto v_leg = this->_data->_legController->datas[leg].v.norm();
        if (std::fabs(v_leg) > 15.)
        {
            printf("Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)\n", leg, v_leg);
            return false;
        }

        auto v_joint = this->_data->_legController->datas[leg].qd;
        if (std::fabs(v_joint[0]) > 25. || std::fabs(v_joint[1]) > 40. || std::fabs(v_joint[2]) > 40.)
        {
            printf("Unsafe locomotion: leg %d joint is moving too quickly (%.2f\t%.2f\t%.2f rad/s)\n", leg, v_joint[0], v_joint[1], v_joint[2]);
            return false;
        }
    }

    return true;
}

template <typename T>
void FSM_State_RL<T>::computeObservation()
{
    // update observations
    this->_data->_stateEstimator->run();
    _bodyAngVel << this->_data->_stateEstimator->getResult().omegaBody;
    _projGravity = this->_data->_stateEstimator->getResult().rBody * _G;
    _dofPos << this->_data->_legController->datas[1].q,
        this->_data->_legController->datas[0].q,
        this->_data->_legController->datas[3].q,
        this->_data->_legController->datas[2].q;
    _dofVel << this->_data->_legController->datas[1].qd,
        this->_data->_legController->datas[0].qd,
        this->_data->_legController->datas[3].qd,
        this->_data->_legController->datas[2].qd;

    // concatenate observations
    for (int i = 0; i < NUM_OBS; i++)
    {
        if (i < 3)
            obsPtr[i] = _bodyAngVel[i];
        else if (i < 6)
            obsPtr[i] = _projGravity(i - 3);
        else if (i < 9)
            obsPtr[i] = _command(i - 6);
        else if (i < 9 + NUM_ACTIONS)
            obsPtr[i] = _dofPos[i - 9] - _defaultDofPos[i - 9];
        else if (i < 9 + NUM_ACTIONS * 2)
            obsPtr[i] = _dofVel[i - (9 + NUM_ACTIONS)];
        else
            obsPtr[i] = _actions[i - (9 + NUM_ACTIONS * 2)];
    }
}

template <typename T>
void FSM_State_RL<T>::logData()
{
    static long time_c = 0;
    static std::ofstream log_runtime("~/log/motor.csv");
    if (time_c == 0)
    {
        for (int i = 0; i < 3; ++i)
            log_runtime << "rpy_" << i << ",";
        log_runtime << ",";
        for (int i = 0; i < 3; ++i)
            log_runtime << "vBody_" << i << ",";
        log_runtime << ",";
        for (int i = 0; i < 3; ++i)
            log_runtime << "position_" << i << ",";
        log_runtime << ",";
        for (int i = 0; i < 12; ++i)
            log_runtime << "tau_est_" << i << ",";
        log_runtime << ",";
        for (int i = 0; i < 12; ++i)
            log_runtime << "tau_real_" << i << ",";
        log_runtime << ",";
        for (int i = 0; i < 12; ++i)
            log_runtime << "joint_pos_" << i << ",";
        log_runtime << ",";
        for (int i = 0; i < 12; ++i)
            log_runtime << "joint_pos_target_" << i << ",";
        log_runtime << ",";
        for (int i = 0; i < 12; ++i)
            log_runtime << "joint_vel_" << i << ",";
        log_runtime << std::endl;
    }
    time_c++;
    if (time_c % 10 == 0)
    {
        this->_data->_stateEstimator->run();
        log_runtime << this->_data->_stateEstimator->getResult().rpy[0] << ",";
        log_runtime << this->_data->_stateEstimator->getResult().rpy[1] << ",";
        log_runtime << this->_data->_stateEstimator->getResult().rpy[2] << ",";
        log_runtime << ",";
        log_runtime << this->_data->_stateEstimator->getResult().vBody[0] << ",";
        log_runtime << this->_data->_stateEstimator->getResult().vBody[1] << ",";
        log_runtime << this->_data->_stateEstimator->getResult().vBody[2] << ",";
        log_runtime << ",";
        log_runtime << this->_data->_stateEstimator->getResult().position[0] << ",";
        log_runtime << this->_data->_stateEstimator->getResult().position[1] << ",";
        log_runtime << this->_data->_stateEstimator->getResult().position[2] << ",";
        log_runtime << ",";

        for (int i = 0; i < 4; i++)
        {
            log_runtime << this->_data->_legController->datas[i].tauEstimate[0] << ",";
            log_runtime << this->_data->_legController->datas[i].tauEstimate[1] << ",";
            log_runtime << this->_data->_legController->datas[i].tauEstimate[2] << ",";
        }
        log_runtime << ",";
        for (int i = 0; i < 4; i++)
        {
            log_runtime << this->_data->_legController->datas[i].tauActuatual[0] << ",";
            log_runtime << this->_data->_legController->datas[i].tauActuatual[1] << ",";
            log_runtime << this->_data->_legController->datas[i].tauActuatual[2] << ",";
        }
        log_runtime << ",";
        for (int i = 0; i < 4; i++)
        {
            log_runtime << this->_data->_legController->datas[i].q[0] << ",";
            log_runtime << this->_data->_legController->datas[i].q[1] << ",";
            log_runtime << this->_data->_legController->datas[i].q[2] << ",";
        }
        log_runtime << ",";
        for (int i = 0; i < 4; i++)
        {
            log_runtime << this->_data->_legController->commands[i].qDes[0] << ",";
            log_runtime << this->_data->_legController->commands[i].qDes[1] << ",";
            log_runtime << this->_data->_legController->commands[i].qDes[2] << ",";
        }
        log_runtime << ",";
        for (int i = 0; i < 4; i++)
        {
            log_runtime << this->_data->_legController->datas[i].qd[0] << ",";
            log_runtime << this->_data->_legController->datas[i].qd[1] << ",";
            log_runtime << this->_data->_legController->datas[i].qd[2] << ",";
        }
        log_runtime << std::endl;
    }
}

template <typename T>
void FSM_State_RL<T>::onExit()
{
}

template <typename T>
FSM_State_RL<T>::~FSM_State_RL()
{
}

template class FSM_State_RL<float>;
