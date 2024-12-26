/*! @file OrientationEstimator.h
 *  @brief All Orientation Estimation Algorithms
 *
 *  This file will contain all orientation algorithms.
 *  Orientation estimators should compute:
 *  - orientation: a quaternion representing orientation
 *  - rBody: coordinate transformation matrix (satisfies vBody = Rbody * vWorld)
 *  - omegaBody: angular velocity in body frame
 *  - omegaWorld: angular velocity in world frame
 *  - rpy: roll pitch yaw
 */
#ifndef PROJECT_ORIENTATIONESTIMATOR_H
#define PROJECT_ORIENTATIONESTIMATOR_H

#include "Controllers/StateEstimatorContainer.h"
#include "T265position_t.hpp"
#include "lcm-cpp.hpp"
#include <thread>
//#include <librealsense2/rs.hpp>
/*!
 * "Cheater" estimator for orientation which always returns the correct value in simulation
 */
template <typename T>
class CheaterOrientationEstimator : public GenericEstimator<T> {
 public:
  virtual void run();
  virtual void setup() {}
};

/*!
 * Estimator for the VectorNav IMU.  The VectorNav provides an orientation already and
 * we just return that.
 */
template <typename T>
class VectorNavOrientationEstimator : public GenericEstimator<T> {
 public:
  virtual void run();
  virtual void setup() {}
    VectorNavOrientationEstimator();
    std::thread _t265OritationLcmThread;
    void handleLCM();
    volatile bool _interfaceLcmQuit = false;
    lcm::LCM myOritationLCM;
    void handleT265LCM ( const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                         const T265position_t* msg );
    T t265_roll,t265_pitch,t265_yaw,t265_omega_x,t265_omega_y,t265_omega_z;
 protected:
  bool _b_first_visit = true;
  Quat<T> _ori_ini_inv;
//    rs2::pipeline pipe;
//    rs2::config cfg;
//    long pipe_cout;
};


#endif  // PROJECT_ORIENTATIONESTIMATOR_H
