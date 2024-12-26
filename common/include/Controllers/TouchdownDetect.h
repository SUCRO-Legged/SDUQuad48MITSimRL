//
// Created by user on 2022/3/24.
//

#ifndef CHEETAH_SDUOG_48_VMWBC_LITTLEWHITE_CHANGE_F_V2_0_TOUCHDOWNDETECT_H
#define CHEETAH_SDUOG_48_VMWBC_LITTLEWHITE_CHANGE_F_V2_0_TOUCHDOWNDETECT_H

#include "Controllers/StateEstimatorContainer.h"

/*!
 * A "passthrough" contact estimator which returns the expected contact state
 */
template <typename T>
class TouchdownDetect : public GenericEstimator<T> {
public:

    /*!
     * Set the estimated contact by copying the exptected contact state into the
     * estimated contact state
     */
    virtual void run();

    /*!
     * Set up the contact estimator
     */
    virtual void setup() {}
};

#endif //CHEETAH_SDUOG_48_VMWBC_LITTLEWHITE_CHANGE_F_V2_0_TOUCHDOWNDETECT_H
