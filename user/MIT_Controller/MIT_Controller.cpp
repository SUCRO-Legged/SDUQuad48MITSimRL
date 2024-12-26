#include "MIT_Controller.hpp"

MIT_Controller::MIT_Controller() : RobotController() {}

// #define RC_ESTOP
/**
 * Initializes the Control FSM.
 */
void MIT_Controller::initializeController()
{
	// Initializes the Control FSM with all the required data
	_controlFSM = new ControlFSM<float>(_quadruped, _stateEstimator, _legController,
										_desiredStateCommand, _controlParameters,
										_visualizationData, &userParameters);
}

/**
 * Calculate the commands for the leg controllers using the ControlFSM logic.
 */
void MIT_Controller::runController()
{
	// Find the desired state trajectory
	_desiredStateCommand->convertToStateCommands();
	// Run the Control FSM code
	_controlFSM->runFSM();
}
