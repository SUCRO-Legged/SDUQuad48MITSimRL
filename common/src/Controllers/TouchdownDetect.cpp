//
// Created by user on 2022/3/24.
//

#include "Controllers/TouchdownDetect.h"
#include <fstream>
//std::ofstream out("/home/user/log/out.csv");

int touchdownDetect_downstair_flag[4] = {1, 1, 1, 1};

template <typename T>
void  TouchdownDetect<T>::run(){

    for(int leg = 0; leg < 4; leg++){
        if(this->_stateEstimatorData.legControllerData[leg].footforceActuatual[2] < -15.0){
            this->_stateEstimatorData.result->touchdownDetect_upstair[leg] = 1;
        }else{
            this->_stateEstimatorData.result->touchdownDetect_upstair[leg] = 0;
        }
        auto contact_phase = *this->_stateEstimatorData.contactPhase;
        auto result = this->_stateEstimatorData.legControllerData[leg].v;

        //if(this->_stateEstimatorData.legControllerData[leg].footforceActuatual[2] < -12.0){
        if((contact_phase[leg] > 0.001f && contact_phase[leg] < 0.3f) && (result[2] < -0.2f)
           && (this->_stateEstimatorData.legControllerData[leg].footforceActuatual[2] > -25.0)
           && touchdownDetect_downstair_flag[leg]){
            this->_stateEstimatorData.result->touchdownDetect_downstair[leg] ++;
        }else{
            this->_stateEstimatorData.result->touchdownDetect_downstair[leg] = 0;
            //复位标志位，本次接触相不再进行判断
            touchdownDetect_downstair_flag[leg] = 0;
            if (contact_phase[leg] == 0)
            {
                touchdownDetect_downstair_flag[leg] = 1;
            }
        }
    }
}

template class TouchdownDetect<float>;
template class TouchdownDetect<double>;