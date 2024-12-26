//
// Created by user on 2020/11/2.
//

/*============================= Stand Up ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "FSM_State_StandDown.h"
/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_StandDown<T>::FSM_State_StandDown(ControlFSMData<T> *_controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_DOWN, "STAND_DOWN"),
      _ini_foot_pos(4)
{
    // Do nothing
    // Set the pre controls safety checks
    this->checkSafeOrientation = false;

    // Post control safety checks
    this->checkPDesFoot = false;
    this->checkForceFeedForward = false;

    // Initialize GRF to 0s
    this->footFeedForwardForces = Mat34<T>::Zero();
}

template <typename T>
void FSM_State_StandDown<T>::onEnter()
{
    // Default is to not transition
    printf("FSM_State_StandDown<T>::onEnter() \n");
    this->nextStateName = this->stateName;

    // Reset the transition data
    this->transitionData.zero();

    // Reset iteration counter
    iter = 0;
    StandDown_time = 0;
    height_variation = 0.35;
    for (size_t i(0); i < 4; ++i)
        _ini_foot_pos[i] = this->_data->_legController->datas[i].p;

    _ini_body_pos = (this->_data->_stateEstimator->getResult()).position;

    for (size_t i(0); i < 4; ++i)
    {
        initial_jpos[i] = this->_data->_legController->datas[i].q;
    }
    sleep_jpos[0] << -0.5f, -1.4f, 2.7f;
    sleep_jpos[1] << 0.5f, -1.4f, 2.7f;
    sleep_jpos[2] << -0.5f, -1.4f, 2.7f;
    sleep_jpos[3] << 0.5f, -1.4f, 2.7f;
    last_height_command = _ini_body_pos[2];

    _ini_body_ori_rpy = (this->_data->_stateEstimator->getResult()).rpy;
    _body_weight = this->_data->_quadruped->_bodyMass * 9.81;
    zero_vec3.setZero();
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_StandDown<T>::run()
{

    if (this->_data->_quadruped->_robotType == RobotType::MINI_CHEETAH)
    {
        // T hMax = 0.10;
        // T heightDesired = std::min(hMax, iter * hMax / T(500));

        int time_flag = 99999;
        //  cout << "_ini_foot_pos:" << _ini_foot_pos[0][2] << "_ini_body_pos:" << _ini_body_pos[2] << "getResultZ:" << (this->_data->_stateEstimator->getResult()).position[2] << endl;
        if (StandDown_time < time_flag)
        {
            Vec4<T> contactState;
            contactState << 0.5, 0.5, 0.5, 0.5;
            this->_data->_stateEstimator->setContactPhase(contactState);
            BalanceStandStep();
        }

        StandDown_time++;
    }
    iter++;
}
/**
 * Calculate the commands for the leg controllers for each of the feet.
 */

template <typename T>
void FSM_State_StandDown<T>::BalanceStandStep()
{
    for (size_t i(0); i < 4; ++i)
        _SetJPosInterPts(iter, 1500, i, initial_jpos[i], sleep_jpos[i]);
}
template <typename T>
void FSM_State_StandDown<T>::_SetJPosInterPts(
    const size_t &curr_iter, size_t max_iter, int leg,
    const Vec3<T> &ini, const Vec3<T> &fin)
{

    float a(0.f);
    float b(1.f);

    // if we're done interpolating
    if (curr_iter <= max_iter)
    {
        b = (float)curr_iter / (float)max_iter;
        a = 1.f - b;
    }

    // compute setpoints
    Vec3<T> inter_pos = a * ini + b * fin;
    // do control
    this->jointPDControl(leg, inter_pos, zero_vec3);
}
/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_StandDown<T>::checkTransition()
{
    this->nextStateName = this->stateName;
    iter++;

    // Switch FSM control mode
    switch ((int)this->_data->controlParameters->control_mode)
    {

    case K_PASSIVE: // normal c
        this->nextStateName = FSM_StateName::PASSIVE;

        break;

    case K_STAND_DOWN: // normal c
        break;

    case K_RECOVERY_STAND:
        this->nextStateName = FSM_StateName::RECOVERY_STAND;
        // Transition time is immediate
        this->transitionDuration = 0.0;
        break;
    default:
        if (this->_data->_desiredStateCommand->returnBackGamepad()->rightTriggerAnalog)
            this->nextStateName = FSM_StateName::PASSIVE;
        std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                  << K_STAND_DOWN << " to "
                  << this->_data->controlParameters->control_mode << std::endl;
    }

    // Get the next state
    return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_StandDown<T>::transition()
{
    // Finish Transition
    switch (this->nextStateName)
    {
    case FSM_StateName::PASSIVE: // normal
        this->transitionData.done = true;
        break;

    case FSM_StateName::STAND_DOWN:
        this->transitionData.done = true;
        break;

    case FSM_StateName ::RECOVERY_STAND:
        this->transitionData.done = true;
        break;

    default:
        std::cout << "[CONTROL FSM] Something went wrong in transition"
                  << std::endl;
    }

    // Return the transition data to the FSM
    return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_StandDown<T>::onExit()
{
    // Nothing to clean up when exiting
}

// template class FSM_State_StandUp<double>;
template class FSM_State_StandDown<float>;
