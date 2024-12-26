/*!
 * @file RobotRunner.cpp
 * @brief Common framework for running robot controllers.
 * This code is a common interface between control code and hardware/simulation
 * for mini cheetah and cheetah 3
 */

#include <unistd.h>

#include "RobotRunner.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/OrientationEstimator.h"
#include "Dynamics/Cheetah3.h"
#include "Dynamics/MiniCheetah.h"
#include "Utilities/Utilities_print.h"
#include "ParamHandler.hpp"
#include "Utilities/Timer.h"
#include "Controllers/PositionVelocityEstimator.h"
#include "rt/rt_socket.h"

extern ReplyMessage replymessage;
float x_vel_des;

RobotRunner::RobotRunner(RobotController *robot_ctrl,
						 PeriodicTaskManager *manager,
						 float period, std::string name) : PeriodicTask(manager, period, name),
														   _lcm(getLcmUrl(255))
{

	_robot_ctrl = robot_ctrl;
}

/**
 * Initializes the robot model, state estimator, leg controller,
 * robot data, and any control logic specific data.
 */
void RobotRunner::init()
{
	printf("[RobotRunner] initialize\n");

	// Build the appropriate Quadruped object
	if (robotType == RobotType::MINI_CHEETAH)
	{
		_quadruped = buildMiniCheetah<float>();
	}
	else
	{
		_quadruped = buildCheetah3<float>();
	}

	// Initialize the model and robot data
	_model = _quadruped.buildModel();
	_jpos_initializer = new JPosInitializer<float>(3., controlParameters->controller_dt);

	// Always initialize the leg controller and state entimator
	_legController = new LegController<float>(_quadruped);
	_stateEstimator = new StateEstimatorContainer<float>(
		cheaterState, vectorNavData, _legController->datas,
		&_stateEstimate, controlParameters);
	initializeStateEstimator(false);

	memset(&rc_control, 0, sizeof(rc_control_settings));
	// Initialize the DesiredStateCommand object
	_desiredStateCommand =
		new DesiredStateCommand<float>(driverCommand,
									   &rc_control,
									   controlParameters,
									   &_stateEstimate,
									   controlParameters->controller_dt);

	// Controller initializations
	_robot_ctrl->_model = &_model;
	_robot_ctrl->_quadruped = &_quadruped;
	_robot_ctrl->_legController = _legController;
	_robot_ctrl->_stateEstimator = _stateEstimator;
	_robot_ctrl->_stateEstimate = &_stateEstimate;
	_robot_ctrl->_visualizationData = visualizationData;
	_robot_ctrl->_robotType = robotType;
	_robot_ctrl->_driverCommand = driverCommand;
	_robot_ctrl->_controlParameters = controlParameters;
	_robot_ctrl->_desiredStateCommand = _desiredStateCommand;

	_robot_ctrl->initializeController();
}

/**
 * Runs the overall robot control system by calling each of the major components
 * to run each of their respective steps.
 * 这是定时周期函数,定时2ms
 */
void RobotRunner::run()
{
	_stateEstimator->run();
	visualizationData->clear();

	replymessage.position_x = _stateEstimator->getResult().position[0];
	replymessage.position_y = _stateEstimator->getResult().position[1];
	replymessage.position_z = _stateEstimator->getResult().position[2];
	replymessage.roll = _stateEstimator->getResult().rpy[0];
	replymessage.pitch = _stateEstimator->getResult().rpy[1];
	replymessage.yaw = _stateEstimator->getResult().rpy[2];
	for (int leg_num = 0; leg_num < 4; ++leg_num)
	{
		for (int joint_num = 0; joint_num < 3; ++joint_num)
		{
			replymessage.leg_joint[leg_num][joint_num] = _legController->datas[leg_num].q(joint_num);
		}
	}
	replymessage.mode = rc_control.mode;
	replymessage.velocity_x = _stateEstimator->getResult().vBody[0];
	replymessage.velocity_y = _stateEstimator->getResult().vBody[1];
	replymessage.omega_z = _stateEstimator->getResult().omegaBody[2];

	// Update the data from the robot
	setupStep();

	static int count_ini(0);
	++count_ini;
	if (count_ini < 10)
	{
		_legController->setEnabled(false);
	}
	else if (20 < count_ini && count_ini < 30)
	{
		_legController->setEnabled(false);
	}
	else if (40 < count_ini && count_ini < 50)
	{
		_legController->setEnabled(false);
	}
	else
	{
		_legController->setEnabled(true);

		if ((rc_control.mode == 0) && controlParameters->use_rc)
		{
			if (count_ini % 1000 == 0)
				printf("ESTOP!\n");
			for (int leg = 0; leg < 4; leg++)
			{
				_legController->commands[leg].zero();
			}
			_robot_ctrl->Estop();
		}
		else
		{
			// Controller
			if (0) // !_jpos_initializer->IsInitialized(_legController))
			{
				Mat3<float> kpMat;
				Mat3<float> kdMat;
				// Update the jpos feedback gains
				if (robotType == RobotType::MINI_CHEETAH)
				{
					kpMat << 5, 0, 0, 0, 5, 0, 0, 0, 5;
					kdMat << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
				}
				else if (robotType == RobotType::CHEETAH_3)
				{
					kpMat << 50, 0, 0, 0, 50, 0, 0, 0, 50;
					kdMat << 1, 0, 0, 0, 1, 0, 0, 0, 1;
				}
				else
				{
					assert(false);
				}

				for (int leg = 0; leg < 4; leg++)
				{
					_legController->commands[leg].kpJoint = kpMat;
					_legController->commands[leg].kdJoint = kdMat;
				}
			}
			else
			{
				// Run Control
				_robot_ctrl->runController();
				cheetahMainVisualization->p = _stateEstimate.position;

				// Update Visualization
				_robot_ctrl->updateVisualization();
				cheetahMainVisualization->p = _stateEstimate.position;
			}
		}
	}

	// Visualization (will make this into a separate function later)
	for (int leg = 0; leg < 4; leg++)
	{
		for (int joint = 0; joint < 3; joint++)
		{
			cheetahMainVisualization->q[leg * 3 + joint] = _legController->datas[leg].q[joint];
		}
	}
	cheetahMainVisualization->p.setZero();
	cheetahMainVisualization->p = _stateEstimate.position;
	cheetahMainVisualization->quat = _stateEstimate.orientation;

	//    cheetahMainVisualization->p[0] = _t265stateEstimate.position[0];
	//    cheetahMainVisualization->p[1] = _t265stateEstimate.position[1];
	//    cheetahMainVisualization->quat[2] = _t265stateEstimate.orientation[2];

	// Sets the leg controller commands for the robot appropriate commands
	finalizeStep();

	//  clock_gettime(CLOCK_MONOTONIC, &_startTime);
	float vel_x_des = _desiredStateCommand->rcCommand->v_des[0];
	if (fabs(vel_x_des) <= 0.5 && fabs(vel_x_des) > 0.2)
		x_vel_des = x_vel_des * (1 - 0.01) + vel_x_des * 0.01;
	else
		x_vel_des = x_vel_des * (1 - 0.05) + vel_x_des * 0.05;
}

/*!
 * Before running user code, setup the leg control and estimators
 */
void RobotRunner::setupStep()
{
	// Update the leg data
	if (robotType == RobotType::MINI_CHEETAH)
	{
		_legController->updateData(spiData);
	}
	else if (robotType == RobotType::CHEETAH_3)
	{
		_legController->updateData(tiBoardData);
	}
	else
	{
		assert(false);
	}

	// Setup the leg controller for a new iteration
	_legController->zeroCommand();
	_legController->setEnabled(true);
	_legController->setMaxTorqueCheetah3(208.5);

	// state estimator
	// check transition to cheater mode:
	if (!_cheaterModeEnabled && controlParameters->cheater_mode)
	{
		printf("[RobotRunner] Transitioning to Cheater Mode...\n");
		initializeStateEstimator(true);
		// todo any configuration
		_cheaterModeEnabled = true;
	}

	// check transition from cheater mode:
	if (_cheaterModeEnabled && !controlParameters->cheater_mode)
	{
		printf("[RobotRunner] Transitioning from Cheater Mode...\n");
		initializeStateEstimator(false);
		// todo any configuration
		_cheaterModeEnabled = false;
	}

	get_rc_control_settings(&rc_control);

	// todo safety checks, sanity checks, etc...
}

/*!
 * After the user code, send leg commands, update state estimate, and publish debug data
 */
void RobotRunner::finalizeStep()
{
	if (robotType == RobotType::MINI_CHEETAH)
	{
		_legController->updateCommand(spiCommand);
	}
	else if (robotType == RobotType::CHEETAH_3)
	{
		_legController->updateCommand(tiBoardCommand);
	}
	else
	{
		assert(false);
	}
	_legController->setLcm(&leg_control_data_lcm, &leg_control_command_lcm);
	_stateEstimate.setLcm(state_estimator_lcm);

#ifdef LCM_PUBLISH_ENABLE
	_lcm.publish("leg_control_command", &leg_control_command_lcm);
	_lcm.publish("leg_control_data", &leg_control_data_lcm);
	_lcm.publish("state_estimator", &state_estimator_lcm);
#endif
	global_to_robot_lcmt.rpy[0] = state_estimator_lcm.rpy[0];
	global_to_robot_lcmt.rpy[1] = state_estimator_lcm.rpy[1];
	global_to_robot_lcmt.rpy[2] = state_estimator_lcm.rpy[2];
	global_to_robot_lcmt.xyz[0] = state_estimator_lcm.p[0];
	global_to_robot_lcmt.xyz[1] = state_estimator_lcm.p[1];
	global_to_robot_lcmt.xyz[2] = state_estimator_lcm.p[2];
	//    static int tmp_show=0;
	//    tmp_show++;
	//    if(tmp_show%500==0)
	//        printf("state_estimator_lcm :%.2f\t%.2f\t%.2f\n",state_estimator_lcm.p[0],state_estimator_lcm.p[1],state_estimator_lcm.p[2]);
	if (global_to_robot_lcmt.xyz[2] < 0)
	{

		static int global_to_robot_lcmt_times = 0;
		global_to_robot_lcmt_times++;
		if (global_to_robot_lcmt_times % 100 == 0)
			// printf("global_to_robot_lcmt z :%.3f below 0!!!!!!!!!!!\n",global_to_robot_lcmt.xyz[2]);
			global_to_robot_lcmt.xyz[2] = 0;
	}
#ifdef LCM_PUBLISH_ENABLE
	// _lcm.publish("global_to_robot", &global_to_robot_lcmt);
#endif
	_iterations++;
}

/*!
 * Reset the state estimator in the given mode.
 * @param cheaterMode
 */
void RobotRunner::initializeStateEstimator(bool cheaterMode)
{
	_stateEstimator->removeAllEstimators();
	_stateEstimator->addEstimator<ContactEstimator<float>>();
	Vec4<float> contactDefault;
	contactDefault << 0.5, 0.5, 0.5, 0.5;
	_stateEstimator->setContactPhase(contactDefault);
	if (cheaterMode)
	{
		_stateEstimator->addEstimator<CheaterOrientationEstimator<float>>();
		_stateEstimator->addEstimator<CheaterPositionVelocityEstimator<float>>();
	}
	else
	{
		_stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
		_stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
	}
}

RobotRunner::~RobotRunner()
{
	delete _legController;
	delete _stateEstimator;
	delete _jpos_initializer;
}

void RobotRunner::cleanup() {}
