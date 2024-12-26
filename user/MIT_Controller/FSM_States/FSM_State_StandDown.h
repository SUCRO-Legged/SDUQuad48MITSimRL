//
// Created by Billchen on 2020/11/2.
//

#ifndef CHEETAH_SDUOG_SMALL_TERRAIN_SOFTWARE_FSM_STATE_STANDDOWN_H
#define CHEETAH_SDUOG_SMALL_TERRAIN_SOFTWARE_FSM_STATE_STANDDOWN_H

#include "FSM_State.h"
template <typename T>
class WBC_Ctrl;
template <typename T>
class LocomotionCtrlData;
/**
 *
 */
template <typename T>
class FSM_State_StandDown : public FSM_State<T>
{
public:
    FSM_State_StandDown(ControlFSMData<T> *_controlFSMData);

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
    TransitionData<T> testTransition();
    void _SetJPosInterPts(const size_t &curr_iter, size_t max_iter, int leg, const Vec3<T> &ini, const Vec3<T> &fin);

    int StandDown_time;

private:
    // Keep track of the control iterations
    int iter = 0;
    std::vector<Vec3<T>> _ini_foot_pos;
    void BalanceStandStep();
    float it;
    T last_height_command = 0;
    float height_variation;
    Vec3<T> _ini_body_pos;
    Vec3<T> _ini_body_ori_rpy;
    Vec3<T> initial_jpos[4];
    Vec3<T> sleep_jpos[4];
    Mat3<T> kpMat, kdMat;
    T _body_weight;
    Vec3<T> zero_vec3;
};
#endif // CHEETAH_SDUOG_SMALL_TERRAIN_SOFTWARE_FSM_STATE_STANDDOWN_H
