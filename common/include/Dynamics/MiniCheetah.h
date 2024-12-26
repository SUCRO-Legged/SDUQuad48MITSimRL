/*! @file MiniCheetah.h
 *  @brief Utility function to build a Mini Cheetah Quadruped object
 *
 * This file is based on MiniCheetahFullRotorModel_mex.m and builds a model
 * of the Mini Cheetah robot.  The inertia parameters of all bodies are
 * determined from CAD.
 *
 */

#ifndef PROJECT_MINICHEETAH_H
#define PROJECT_MINICHEETAH_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of Mini Cheetah
 */
template <typename T>
//Quadruped<T> buildMiniCheetah() {
//  Quadruped<T> cheetah;
//  cheetah._robotType = RobotType::MINI_CHEETAH;
//
//    cheetah._bodyMass = 5.5;//7.5 ;//5.5
//    cheetah._bodyLength = 0.225*2;//0.2465*2;//0.2055 * 2;//0.185*2;//0.195*2;//
//    cheetah._bodyWidth = 0.05*2;//0.06*2;//0.05 *2;// 0.049 * 2;//
//    cheetah._bodyHeight = 0.05 * 2;
//    cheetah._abadGearRatio = 6;
//    cheetah._hipGearRatio = 6;
//    cheetah._kneeGearRatio = 6;
//    cheetah._abadLinkLength = 0.087;//0.069;//0.062;//
//    cheetah._hipLinkLength = 0.225;//0.209;
//    cheetah._kneeLinkY_offset = 0;//0.004;
//    cheetah._kneeLinkLength = 0.235;//0.195;
//    cheetah._maxLegLength = 0.45;
//
//
//    cheetah._motorTauMax = 8.f;//3.f;
//    cheetah._batteryV = 36;
//    cheetah._motorKT = .05;  // this is flux linkage * pole pairs
//    cheetah._motorR = 0.173;
//    cheetah._jointDamping = .01;
//    cheetah._jointDryFriction = .2;
//
//
//
//  // rotor inertia if the rotor is oriented so it spins around the z-axis
//  Mat3<T> rotorRotationalInertiaZ;
////  rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
//  rotorRotationalInertiaZ << 133.72, 0, 0, 0, 133.72, 0, 0, 0, 257.61;
//  rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;
//
//  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
//  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
//  Mat3<T> rotorRotationalInertiaX =
//      RY * rotorRotationalInertiaZ * RY.transpose();
//  Mat3<T> rotorRotationalInertiaY =
//      RX * rotorRotationalInertiaZ * RX.transpose();
//
//  // spatial inertias
//  Mat3<T> abadRotationalInertia;
////  abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
//    abadRotationalInertia << 502.76, 0.62, 0.33, 0.62, 775.21, 0.18, 0.33, 0.18, 502.21;
//  abadRotationalInertia = abadRotationalInertia * 1e-6;
//  Vec3<T> abadCOM(0, 0.036, 0);  // LEFT  ?!??????????????????
//  SpatialInertia<T> abadInertia(0.71, abadCOM, abadRotationalInertia);
//
//  Mat3<T> hipRotationalInertia;
////  hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
//  hipRotationalInertia << 4602.753, 11.59, -87.34, 11.59, 4730.21, 674.636, 87.343, 674.636, 908.955;
//  hipRotationalInertia = hipRotationalInertia * 1e-6;
////  Vec3<T> hipCOM(0, 0.016, -0.02);
//  //Vec3<T> hipCOM(0, 0.016, -0.025);
//  Vec3<T> hipCOM(0, -0.016, -0.025);
////  SpatialInertia<T> hipInertia(0.634, hipCOM, hipRotationalInertia);
//  SpatialInertia<T> hipInertia(1.066, hipCOM, hipRotationalInertia);
//
//  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
////  kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
//    kneeRotationalInertiaRotated << 1029.84, 0, -132.77, 0, 1062.13, 0, -132.77, 0, 46.68;
//  kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
//  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
////  Vec3<T> kneeCOM(0, 0, -0.061);
//    Vec3<T> kneeCOM(0.01, 0, -0.097);
//  SpatialInertia<T> kneeInertia(0.14, kneeCOM, kneeRotationalInertia);
//
//  Vec3<T> rotorCOM(0, 0, 0);
//  SpatialInertia<T> rotorInertiaX(0.22, rotorCOM, rotorRotationalInertiaX);
//  SpatialInertia<T> rotorInertiaY(0.22, rotorCOM, rotorRotationalInertiaY);
//
//  Mat3<T> bodyRotationalInertia;
//  bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
////    bodyRotationalInertia << 18417.282, 9.4, -468.8, 9.4, 65594, 1.72, -468.82, 0, 77059.5;
//  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
//  Vec3<T> bodyCOM(0, 0, 0);
//  SpatialInertia<T> bodyInertia(cheetah._bodyMass, bodyCOM,
//                                bodyRotationalInertia);
//
//  cheetah._abadInertia = abadInertia;
//  cheetah._hipInertia = hipInertia;
//  cheetah._kneeInertia = kneeInertia;
//  cheetah._abadRotorInertia = rotorInertiaX;
//  cheetah._hipRotorInertia = rotorInertiaY;
//  cheetah._kneeRotorInertia = rotorInertiaY;
//  cheetah._bodyInertia = bodyInertia;
//
//  // locations
////  cheetah._abadRotorLocation = Vec3<T>(0.125, 0.049, 0);
//  cheetah._abadRotorLocation = Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
//  cheetah._abadLocation =
//      Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
//  cheetah._hipLocation = Vec3<T>(0, cheetah._abadLinkLength, 0);
//  cheetah._hipRotorLocation = Vec3<T>(0, 0.04, 0);
//  cheetah._kneeLocation = Vec3<T>(0, 0, -cheetah._hipLinkLength);
//  cheetah._kneeRotorLocation = Vec3<T>(0, 0, 0);
//
//  return cheetah;
//}

Quadruped<T> buildMiniCheetah() {
  Quadruped<T> cheetah;
  cheetah._robotType = RobotType::MINI_CHEETAH;

  //cheetah._bodyMass = 7.0;//7.5 ;//5.5
  // todo
  cheetah._bodyMass = 5.0;//7.5 ;//5.5
  cheetah._bodyLength = 0.226*2;//0.2465*2;//0.2055 * 2;//0.185*2;//0.195*2;//
  cheetah._bodyWidth = 0.055*2;//0.06*2;//0.05 *2;// 0.049 * 2;//
  cheetah._bodyHeight = 0.05 * 2;
  cheetah._abadGearRatio = 6;
  cheetah._hipGearRatio = 6;
  cheetah._kneeGearRatio = 6;
  cheetah._abadLinkLength = 0.0875;//0.069;//0.062;//
  cheetah._hipLinkLength = 0.225;//0.209;
  cheetah._kneeLinkY_offset = 0;//0.004;
  cheetah._kneeLinkLength = 0.235;//0.195;
  cheetah._maxLegLength = 0.45;


  cheetah._motorTauMax = 8.f;//3.f;
  cheetah._batteryV = 36;
  cheetah._motorKT = .05;  // this is flux linkage * pole pairs
  cheetah._motorR = 0.173;
  cheetah._jointDamping = .01;
  cheetah._jointDryFriction = .2;

  // rotor inertia if the rotor is oriented so it spins around the z-axis
  Mat3<T> rotorRotationalInertiaZ;
  //  rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
  rotorRotationalInertiaZ << 133.72,    0,      0,
                              0,        133.72, 0,
                              0,        0,      257.61;
  rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX =
          RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY =
          RX * rotorRotationalInertiaZ * RX.transpose();

  // spatial inertias
  Mat3<T> abadRotationalInertia;
  //  abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
  abadRotationalInertia <<  502.76,   0.62,     0.33,
                            0.62,     775.21,   0.18,
                            0.33,     0.18,     502.21;
  abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<T> abadCOM(-0.003311, 0.000635, 0.000031);  // LEFT  ?!??????????????????
  SpatialInertia<T> abadInertia(0.75, abadCOM, abadRotationalInertia);

  Mat3<T> hipRotationalInertia;
  //  hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
  hipRotationalInertia << 4602.753,   11.59,    -87.34,
                          11.59,      4730.21,  674.636,
                          -87.343,    674.636,  908.955;
  hipRotationalInertia = hipRotationalInertia * 1e-6;
  //  Vec3<T> hipCOM(0, 0.016, -0.02);
  //Vec3<T> hipCOM(0, 0.016, -0.025);
  Vec3<T> hipCOM(0, -0.016, -0.025);
  //  SpatialInertia<T> hipInertia(0.634, hipCOM, hipRotationalInertia);
  SpatialInertia<T> hipInertia(1.05, hipCOM, hipRotationalInertia);

  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
  //  kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
  kneeRotationalInertiaRotated << 1482.84,  0,        -191.77,
                                  0,        1529.13,  0,
                                  -191.77,  0,        67.68;
  kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
  //  Vec3<T> kneeCOM(0, 0, -0.061);
  Vec3<T> kneeCOM(0.01, 0, -0.097);
  SpatialInertia<T> kneeInertia(0.2, kneeCOM, kneeRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.22, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.22, rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  // todo
  bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
  // bodyRotationalInertia <<  30338,  1323,   764,
  //                           1323,   106987, -97,
  //                           764,    -97,    124175;
  //    bodyRotationalInertia << 18417.282, 9.4, -468.8, 9.4, 65594, 1.72, -468.82, 0, 77059.5;
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0, 0, 0);
  //Vec3<T> bodyCOM(-0.0100, 0.002, -0.005);
  SpatialInertia<T> bodyInertia(cheetah._bodyMass, bodyCOM,
                                bodyRotationalInertia);

  cheetah._abadInertia = abadInertia;
  cheetah._hipInertia = hipInertia;
  cheetah._kneeInertia = kneeInertia;
  cheetah._abadRotorInertia = rotorInertiaX;
  cheetah._hipRotorInertia = rotorInertiaY;
  cheetah._kneeRotorInertia = rotorInertiaY;
  cheetah._bodyInertia = bodyInertia;

  // locations
  //  cheetah._abadRotorLocation = Vec3<T>(0.125, 0.049, 0);
  cheetah._abadRotorLocation = Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
  cheetah._abadLocation =
          Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
  cheetah._hipLocation = Vec3<T>(0, cheetah._abadLinkLength, 0);
  cheetah._hipRotorLocation = Vec3<T>(0, 0.04, 0);
  cheetah._kneeLocation = Vec3<T>(0, 0, -cheetah._hipLinkLength);
  cheetah._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return cheetah;
}

#endif  // PROJECT_MINICHEETAH_H
