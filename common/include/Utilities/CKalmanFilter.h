//
// Created by user on 2021/1/12.
//

#ifndef CHEETAH_SDUOG_SMALL_TERRAIN_SOFTWARE_VMWBC_CKALMANFILTER_H
#define CHEETAH_SDUOG_SMALL_TERRAIN_SOFTWARE_VMWBC_CKALMANFILTER_H
class CKalmanFilter {
public:
    CKalmanFilter();
    virtual ~CKalmanFilter();
    float R_signal;
    float R_noise;
    float Filter(float data);
    float Filter(float data, float R_signal, float R_noise);
    float p1;
    float p;
    float s;
    float b;
};

#endif //CHEETAH_SDUOG_SMALL_TERRAIN_SOFTWARE_VMWBC_CKALMANFILTER_H
