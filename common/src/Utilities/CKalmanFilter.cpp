//
// Created by user on 2021/1/12.
//

#include "../include/Utilities/CKalmanFilter.h"

CKalmanFilter::CKalmanFilter() {
    R_signal = 0.5;
    R_noise = 5;
    s = 0;
    p = 0;
}

CKalmanFilter::~CKalmanFilter() {
}
float CKalmanFilter::Filter(float data)
{
    p1 = p + R_signal;
    b = p1 / (p1 + R_noise);
    s = s + b*(data - s);
    p = p1 - b*p1;
    return s;
}
float CKalmanFilter::Filter(float data,float signal,float noise)
{
    p1 = p + signal;
    b = p1 / (p1 + noise);
    s = s + b * (data - s);
    p = p1 - b * p1;
    return s;
}


