/*============================= Recovery Stand ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "FSM_State_RecoveryStand.h"
#include <Utilities/Utilities_print.h>
#include <rt/rt_spi.h>

// extern spine_data_t g_spine_data_front;
// extern spine_data_t g_spine_data_hind;
/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_RecoveryStand<T>::FSM_State_RecoveryStand(ControlFSMData<T> *_controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::RECOVERY_STAND, "RECOVERY_STAND")
{
    rcCmd = new rc_cmd_t();
    // Do nothing
    // Set the pre controls safety checks
    this->checkSafeOrientation = false;

    // Post control safety checks
    this->checkPDesFoot = false;
    this->checkForceFeedForward = false;

    zero_vec3.setZero();
    // goal configuration
    // Folding
    for (size_t i(0); i < 4; ++i)
        fold_jpos[i] << 0.0f, -1.4f, 2.7f;

    // Stand Up
    for (size_t i(0); i < 4; ++i)
        stand_jpos[i] << this->_data->userParameters->init_pos[0], this->_data->userParameters->init_pos[1], this->_data->userParameters->init_pos[2];

    // Rolling
    rolling_jpos[0] << 1.5f, -1.6f, 2.77f;
    rolling_jpos[1] << 1.3f, -3.1f, 2.77f;
    rolling_jpos[2] << 1.5f, -1.6f, 2.77f;
    rolling_jpos[3] << 1.3f, -3.1f, 2.77f;

    f_ff << 0.f, 0.f, -25.f;
}

template <typename T>
void FSM_State_RecoveryStand<T>::onEnter()
{
    // Default is to not transition
    this->nextStateName = this->stateName;

    // Reset the transition data
    this->transitionData.zero();

    // Reset iteration counter
    iter = 0;
    _state_iter = 0;

    // initial configuration, position
    for (size_t i(0); i < 4; ++i)
    {
        initial_jpos[i] = this->_data->_legController->datas[i].q;
    }

    T body_height =
        this->_data->_stateEstimator->getResult().position[2];

    _flag = FoldLegs;
    if (!_UpsideDown())
    {
        // Proper orientation
        if ((0.2 < body_height) && (body_height < 0.45))
        {
            printf("[Recovery Stand] body height is %f; Stand Up \n", body_height);
            _flag = StandUp;
        }
        else
        {
            printf("[Recovery Stand] body height is %f; Folding legs \n", body_height);
        }
    }
    else
    {
        printf("[Recovery Stand] UpsideDown (%d) \n", _UpsideDown());
    }
    _motion_start_iter = 0;
}

template <typename T>
bool FSM_State_RecoveryStand<T>::_UpsideDown()
{
    if (this->_data->_stateEstimator->getResult().rBody(2, 2) < 0)
    {
        return true;
    }
    return false;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_RecoveryStand<T>::run()
{

    switch (_flag)
    {
    case StandUp:
        _StandUp(_state_iter - _motion_start_iter);
        break;
    case FoldLegs:
        _FoldLegs(_state_iter - _motion_start_iter);
        break;
    case RollOver:
        _RollOver(_state_iter - _motion_start_iter);
        break;
    }

    ++_state_iter;
}

template <typename T>
void FSM_State_RecoveryStand<T>::_SetJPosInterPts(
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

    // if(curr_iter == 0){
    // printf("flag:%d, curr iter: %lu, state iter: %llu, motion start iter: %d\n",
    //_flag, curr_iter, _state_iter, _motion_start_iter);
    // printf("inter pos: %f, %f, %f\n", inter_pos[0], inter_pos[1], inter_pos[2]);
    // }
    // if(curr_iter == max_iter){
    // printf("flag:%d, curr iter: %lu, state iter: %llu, motion start iter: %d\n",
    //_flag, curr_iter, _state_iter, _motion_start_iter);
    // printf("inter pos: %f, %f, %f\n", inter_pos[0], inter_pos[1], inter_pos[2]);
    // }
}

template <typename T>
void FSM_State_RecoveryStand<T>::_RollOver(const int &curr_iter)
{

    for (size_t i(0); i < 4; ++i)
    {
        _SetJPosInterPts(curr_iter, rollover_ramp_iter, i,
                         initial_jpos[i], rolling_jpos[i]);
    }

    if (curr_iter > rollover_ramp_iter + rollover_settle_iter)
    {
        _flag = FoldLegs;
        for (size_t i(0); i < 4; ++i)
            initial_jpos[i] = rolling_jpos[i];
        _motion_start_iter = _state_iter + 1;
    }
}

template <typename T>
void FSM_State_RecoveryStand<T>::_StandUp(const int &curr_iter)
{
    T body_height = this->_data->_stateEstimator->getResult().position[2];
    bool something_wrong(false);

    if (_UpsideDown() || (body_height < 0.1))
    {
        something_wrong = true;
    }

    if ((curr_iter > floor(standup_ramp_iter * 0.7)) && something_wrong)
    {
        // If body height is too low because of some reason
        // even after the stand up motion is almost over
        // (Can happen when E-Stop is engaged in the middle of Other state)
        for (size_t i(0); i < 4; ++i)
        {
            initial_jpos[i] = this->_data->_legController->datas[i].q;
        }
        _flag = FoldLegs;
        _motion_start_iter = _state_iter + 1;

        printf("[Recovery Balance - Warning] body height is still too low (%f) or UpsideDown (%d); Folding legs \n",
               body_height, _UpsideDown());
    }
    else
    {
        for (size_t leg(0); leg < 4; ++leg)
        {
            _SetJPosInterPts(curr_iter, standup_ramp_iter,
                             leg, initial_jpos[leg], stand_jpos[leg]);
        }
    }
    // feed forward mass of robot.
    // for(int i = 0; i < 4; i++)
    // this->_data->_legController->commands[i].forceFeedForward = f_ff;
    // Vec4<T> se_contactState(0.,0.,0.,0.);
    Vec4<T> se_contactState(0.5, 0.5, 0.5, 0.5);
    this->_data->_stateEstimator->setContactPhase(se_contactState);
}

template <typename T>
void FSM_State_RecoveryStand<T>::_FoldLegs(const int &curr_iter)
{
    ////////////////////////////////////////////////////////
    //    bug_checker_times++;

    //////////////////////////////////////////////////
    for (size_t i(0); i < 4; ++i)
    {
        _SetJPosInterPts(curr_iter, fold_ramp_iter, i,
                         initial_jpos[i], fold_jpos[i]);
    }

    if (curr_iter >= fold_ramp_iter + fold_settle_iter)
    {
        if (_UpsideDown())
        {
            _flag = RollOver;
            for (size_t i(0); i < 4; ++i)
                initial_jpos[i] = fold_jpos[i];
        }
        else
        {
            _flag = StandUp;
            for (size_t i(0); i < 4; ++i)
                initial_jpos[i] = fold_jpos[i];
        }
        _motion_start_iter = _state_iter + 1;
    }
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_RecoveryStand<T>::checkTransition()
{
    this->nextStateName = this->stateName;
    iter++;
    if (locomotionSafe())
    {
        // Switch FSM control mode
        switch ((int)this->_data->controlParameters->control_mode)
        {
        case K_RECOVERY_STAND:
            break;

        case K_PASSIVE: // normal c
            this->nextStateName = FSM_StateName::PASSIVE;
            break;

        case K_STAND_DOWN:
            this->nextStateName = FSM_StateName::STAND_DOWN;
            this->transitionDuration = 0.;
            break;

        case K_RL_TEST:
            this->nextStateName = FSM_StateName::RL_TEST;
            this->transitionDuration = 0.;
            break;

        default:
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                      << K_RECOVERY_STAND << " to "
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

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_RecoveryStand<T>::transition()
{
    // Finish Transition
    switch (this->nextStateName)
    {
    case FSM_StateName::PASSIVE: // normal
        this->transitionData.done = true;
        break;

    case FSM_StateName ::STAND_DOWN:
        this->transitionData.done = true;
        break;

    case FSM_StateName ::RL_TEST:
        this->transitionData.done = true;
        break;

    default:
        std::cout << "[CONTROL FSM] Something went wrong in transition"
                  << std::endl;
    }

    // Return the transition data to the FSM
    return this->transitionData;
}

template <typename T>
bool FSM_State_RecoveryStand<T>::locomotionSafe()
{

    auto &seResult = this->_data->_stateEstimator->getResult();
    T max_roll = 50;
    T max_pitch = 50;
    if (rcCmd->gait == 1 || rcCmd->gait == 2 || rcCmd->gait == 4 || rcCmd->gait == 5)
    {
        max_pitch = 85;
    }
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
        if (rcCmd->gait == 2 || rcCmd->gait == 5 || rcCmd->gait == 12 || rcCmd->gait == 13)
        {
            if (p_leg[2] > 0 + 0.2f)
            {
                printf("Unsafe locomotion: leg %d is above hip (%.3f m)\n", leg, p_leg[2]);
                return false;
            }
        }
        else
        {
            if (p_leg[2] > 0)
            {
                printf("Unsafe locomotion: leg %d is above hip (%.3f m)\n", leg, p_leg[2]);
                return false;
            }
        }

        if (p_leg[1] > 0.31)
        {
            printf("Unsafe locomotion: leg %d's y-position is bad (%.3f m)\n", leg, p_leg[1]);
            return false;
        }
    }

    return true;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_RecoveryStand<T>::onExit()
{
    // Nothing to clean up when exiting
}

// template class FSM_State_RecoveryStand<double>;
template class FSM_State_RecoveryStand<float>;
