//
// Created by user on 2020/9/26.
//

#ifndef CHEETAH_SOFTWARE_24NM_RT_SOCKET_H
#define CHEETAH_SOFTWARE_24NM_RT_SOCKET_H

#include <unistd.h>
#include <pthread.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <pthread.h>
#include <sys/time.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <errno.h>
#include <bits/socket.h>
#include <sys/socket.h>

#define SERV_PORT 1986

typedef struct
{
    unsigned int start;

    float roll;
    float pitch;
    float yaw;

    float velocity_x;
    float velocity_y;
    float omega_z;

    int mode;
    float step_height;
    int gait;
    float body_height_variation;

    float offset_x;
    float offset_y;

    unsigned int end;

} Client_Command_Message;

typedef struct
{
    unsigned int start;

    float roll;
    float pitch;
    float yaw;

    float velocity_x;
    float velocity_y;
    float omega_z;

    float position_x;
    float position_y;
    float position_z;

    float leg_joint[4][3];

    int mode;
    float voltage;
    int gait;

    unsigned int end;

} ReplyMessage;

void *TCPThread(void *arg);
void init_TCPThread();
#endif // CHEETAH_SOFTWARE_24NM_RT_SOCKET_H
