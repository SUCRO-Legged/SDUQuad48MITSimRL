# sduog48MITSimRL
This repository contains the code for the MIT controller for the SDUog48 robot developed by ourselves. The deployment code is based on the [MIT controller](https://github.com/mit-biomimetics/Cheetah-Software) for the Mini Cheetah robot. The code has been modified to work for the sim-to-sim and sim-to-real of Deep Reinforcement Learning control method.

## Build
Clone this repository and navigate to this repository directory. To build all code:
```
mkdir build
cd build
cmake ..
make -j4
```

If you are building code on your computer that you would like to copy over to the real robot, you must replace the cmake command with
```
cmake -DREAL_ROBOT_BUILD=TRUE
```
otherwise it will not work.  

## Run Simulator
To run the simulator:
1. Open the control board
```
cd build/
./sim/sim
```
2. In the another command window, run the robot control code
```
./user/MIT_Controller/mit_ctrl m s
```
m: Mini Cheetah s: simulation

## Run Real Robot
1. Create build folder `mkdir mc-build`
2. Build as mini cheetah executable `cd mc-build; cmake -DREAL_ROBOT_BUILD=TRUE ..; make -j`
3. Connect to mini cheetah over ethernet, verify you can ssh in
4. Copy program to mini cheetah with `../scripts/send_to_mini_cheetah.sh`
5. ssh into the mini cheetah `ssh user@10.0.0.34`
6. Enter the robot program folder `cd robot-software-....`
7. Run robot code `./run_mc.sh ./mit_ctrl` 


## Dependencies:
- Qt - https://www.qt.io/download-qt-installer (v5.12.4 is recommended)
- LCM - https://lcm-proj.github.io/ (v1.4.0 is recommended) (Please make it sure that you have a java to let lcm compile java-extension together) 
- Eigen - http://eigen.tuxfamily.org
- Onnxruntime - https://github.com/microsoft/onnxruntime (v1.16.3 is recommended)
- `mesa-common-dev`
- `freeglut3-dev`
- `libblas-dev liblapack-dev`

## References:
- mit-biomimetics/Cheetah-Software: https://github.com/mit-biomimetics/Cheetah-Software.git