#include "Utilities/EdgeTrigger.h"
#include "common/include/global_config.h"
#include "rt/rt_socket.h"
#include <pthread.h>
#include <rt/rt_rc_interface.h>
#include <rt/rt_sbus.h>
#include <stdio.h>
#include <string.h> // memcpy
extern Client_Command_Message receive_message;
static pthread_mutex_t lcm_get_set_mutex =
    PTHREAD_MUTEX_INITIALIZER; /**< mutex to protect gui settings coming over
                             LCM */

// Controller Settings
rc_control_settings rc_control;

/* ------------------------- HANDLERS ------------------------- */

// Controller Settings
void get_rc_control_settings(void *settings)
{
    pthread_mutex_lock(&lcm_get_set_mutex);
    v_memcpy(settings, &rc_control, sizeof(rc_control_settings));
    pthread_mutex_unlock(&lcm_get_set_mutex);
}

// void get_rc_channels(void *settings) {
// pthread_mutex_lock(&lcm_get_set_mutex);
// v_memcpy(settings, &rc_channels, sizeof(rc_channels));
// pthread_mutex_unlock(&lcm_get_set_mutex);
// }

EdgeTrigger<int> mode_edge_trigger(0);
EdgeTrigger<TaranisSwitchState> backflip_prep_edge_trigger(SWITCH_UP);
EdgeTrigger<TaranisSwitchState> experiment_prep_edge_trigger(SWITCH_UP);
TaranisSwitchState initial_mode_go_switch = SWITCH_DOWN;

void sbus_packet_complete_at9s()
{
    AT9s_data data;
    update_taranis_at9s(&data);
    float v_scale = 1.5; // 2.0
    float w_scale = 2 * v_scale;

    auto estop_switch = data.SWE;
    auto QP_Locomotion_switch = data.SWA;

    auto left_select = data.SWC;
    auto right_select = data.SWD;

    auto normal_jump_flip_switch = data.SWG;
    auto roll_show = 0.0;
    auto step_height = data.varB + 1.0;
    int selected_mode = 0;
#ifdef USE_TCPnet_Control
    if (right_select == AT9S_BOOL_DOWN)
    {
        rc_control.mode = receive_message.mode;
        rc_control.v_des[0] = deadband(receive_message.velocity_x, 0.1, -1.0, 1.0);
        rc_control.v_des[1] = deadband(receive_message.velocity_y, 0.1, -0.5, 0.5);
        rc_control.v_des[2] = 0;

        rc_control.omega_des[0] = 0;
        rc_control.omega_des[1] = deadband(receive_message.pitch, 0.1, -1.0, 1.0);
        rc_control.omega_des[2] = deadband(receive_message.omega_z, 0.1, -1.0, 1.0);

        rc_control.rpy_des[0] = deadband(receive_message.roll, 0.1, -0.5, 0.5);  //
        rc_control.rpy_des[1] = deadband(receive_message.pitch, 0.1, -0.5, 0.5); //
        rc_control.rpy_des[2] = deadband(receive_message.yaw, 0.1, -0.5, 0.5);   //
        rc_control.step_height = receive_message.step_height * 10.0;
        rc_control.variable[0] = receive_message.gait;
        rc_control.height_variation = receive_message.body_height_variation;
        //        printf("rc_control.mode: %d\n",(int)rc_control.mode);
    }
    else if (right_select == AT9S_BOOL_UP)
    {
#endif
        switch (estop_switch)
        {
        case AT9S_TRI_UP:
            selected_mode = RC_mode::OFF;
            break;

        case AT9S_TRI_MIDDLE:
            if (normal_jump_flip_switch == AT9S_TRI_MIDDLE)
            {
                selected_mode = RC_mode::RECOVERY_STAND;
                rc_control.height_variation = 0;
            }
            else if (normal_jump_flip_switch == AT9S_TRI_UP)
            {
                selected_mode = RC_mode::STAND_DOWN;
            }
            if (normal_jump_flip_switch == AT9S_TRI_DOWN)
            {
                selected_mode = RC_mode::RL_TEST;
                rc_control.height_variation = 0;
            }
            break;

        case AT9S_TRI_DOWN:
            if (normal_jump_flip_switch == AT9S_TRI_MIDDLE)
            {
                if (QP_Locomotion_switch == AT9S_BOOL_UP)
                {
                    data.left_stick_x = deadband(data.left_stick_x, 0.1, -1., 1.);
                    data.left_stick_y = deadband(data.left_stick_y, 0.1, -1., 1.);
                    data.right_stick_x = deadband(data.right_stick_x, 0.1, -1., 1.);
                    data.right_stick_y = deadband(data.right_stick_y, 0.1, -1., 1.);

                    int gait_id = 9;
                    if (right_select == AT9S_BOOL_UP)
                    {
                        if (left_select == AT9S_TRI_UP)
                            gait_id = 9; // trot
                        else if (left_select == AT9S_TRI_MIDDLE)
                            gait_id = 3;
                        else if (left_select == AT9S_TRI_DOWN) // walk
                            gait_id = 6;
                    }
                    else if (right_select == AT9S_BOOL_DOWN)
                    {

                        if (left_select == AT9S_TRI_UP)
                            gait_id = 5; // flying trot
                        else if (left_select == AT9S_TRI_MIDDLE)
                            gait_id = 1; // bound
                        else if (left_select == AT9S_TRI_DOWN)
                            gait_id = 2; // pronk
                    }

                    rc_control.variable[0] = gait_id;
                    rc_control.v_des[0] = data.right_stick_x > 0 ? data.right_stick_x : 0.5 * data.right_stick_x;
                    rc_control.v_des[1] = -1.0 * data.right_stick_y; // -v_scale * data.right_stick_y;
                    rc_control.v_des[2] = 0;

                    rc_control.omega_des[0] = 0;
                    rc_control.omega_des[1] = data.left_stick_x; // 0;//pitch
                    rc_control.omega_des[2] = w_scale * data.left_stick_y;

                    rc_control.rpy_des[0] = roll_show;
                    rc_control.step_height = step_height;
                }
                else if (QP_Locomotion_switch == AT9S_BOOL_DOWN)
                {
                    rc_control.rpy_des[0] = data.left_stick_y;
                    rc_control.rpy_des[1] = data.left_stick_x;
                    rc_control.rpy_des[2] = data.right_stick_y;

                    rc_control.height_variation = 1.5 * data.right_stick_x;

                    rc_control.omega_des[0] = 0;
                    rc_control.omega_des[1] = 0;
                    rc_control.omega_des[2] = 0;
                }
            }
            else if (normal_jump_flip_switch == AT9S_TRI_DOWN)
            {
                selected_mode = RC_mode::RL_TEST;

                int gait_id = 0;
                if (QP_Locomotion_switch == AT9S_BOOL_DOWN)
                {
                    if (right_select == AT9S_BOOL_UP)
                    {
                        if (left_select == AT9S_TRI_UP)
                            gait_id = 0; // trot
                        else if (left_select == AT9S_TRI_MIDDLE)
                            gait_id = 1;                       // slow trot
                        else if (left_select == AT9S_TRI_DOWN) // walk
                            gait_id = 2;
                    }
                    else if (right_select == AT9S_BOOL_DOWN)
                    {
                        if (left_select == AT9S_TRI_UP)
                            gait_id = 3; // trot
                        else if (left_select == AT9S_TRI_MIDDLE)
                            gait_id = 4;                       // slow trot
                        else if (left_select == AT9S_TRI_DOWN) // walk
                            gait_id = 5;
                    }
                }
                else
                {
                    if (right_select == AT9S_BOOL_UP)
                    {
                        if (left_select == AT9S_TRI_UP)
                            gait_id = 6; // trot
                        else if (left_select == AT9S_TRI_MIDDLE)
                            gait_id = 7;                       // slow trot
                        else if (left_select == AT9S_TRI_DOWN) // walk
                            gait_id = 8;
                    }
                    else if (right_select == AT9S_BOOL_DOWN)
                    {
                        if (left_select == AT9S_TRI_UP)
                            gait_id = 9; // trot
                        else if (left_select == AT9S_TRI_MIDDLE)
                            gait_id = 10;                      // slow trot
                        else if (left_select == AT9S_TRI_DOWN) // walk
                            gait_id = 11;
                    }
                }

                data.left_stick_x = deadband(data.left_stick_x, 0.1, -1., 1.);
                data.left_stick_y = deadband(data.left_stick_y, 0.1, -1., 1.);
                data.right_stick_y = deadband(data.right_stick_y, 0.1, -1., 1.);

                rc_control.v_des[0] = data.right_stick_x > 0 ? data.right_stick_x : 0.5 * data.right_stick_x;
                rc_control.v_des[1] = -1.0 * data.right_stick_y; // -v_scale * data.right_stick_y;
                rc_control.v_des[2] = 0;

                rc_control.omega_des[2] = -data.left_stick_y;
                rc_control.omega_des[1] = 0;
                rc_control.omega_des[0] = 0;

                rc_control.rpy_des[1] = 0.5f * data.left_stick_x;
                rc_control.rpy_des[0] = 0;
                rc_control.rpy_des[2] = 0;

                rc_control.step_height = 0.1f * (step_height / 2.f) + 0.01f;
                rc_control.height_variation = 0.0;

                rc_control.variable[0] = gait_id;
            }
            break;
        }
        rc_control.mode = selected_mode;
#ifdef USE_TCPnet_Control
    }
#endif
}

void *v_memcpy(void *dest, volatile void *src, size_t n)
{
    void *src_2 = (void *)src;
    return memcpy(dest, src_2, n);
}

float deadband(float command, float deadbandRegion, float minVal, float maxVal)
{
    if (command < deadbandRegion && command > -deadbandRegion)
    {
        return 0.0;
    }
    else
    {
        return (command / (2)) * (maxVal - minVal);
    }
}
