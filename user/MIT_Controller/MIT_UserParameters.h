#ifndef PROJECT_MITUSERPARAMETERS_H
#define PROJECT_MITUSERPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class MIT_UserParameters : public ControlParameters
{
public:
	MIT_UserParameters()
		: ControlParameters("user-parameters"),

		  // RL Locomotion parameter by KK-Zhou
		  INIT_PARAMETER(init_pos),
		  INIT_PARAMETER(body_height),
		  INIT_PARAMETER(actionScale),
		  INIT_PARAMETER(rlKp),
		  INIT_PARAMETER(rlKd),
		  INIT_PARAMETER(policy_dt),
		  INIT_PARAMETER(emergency_orientation),
		  INIT_PARAMETER(logData),
		  INIT_PARAMETER(policyDirNum)

	{
	}

	// RL Locomotion parameter by KK-Zhou
	DECLARE_PARAMETER(Vec3<double>, init_pos);
	DECLARE_PARAMETER(double, body_height);
	DECLARE_PARAMETER(double, actionScale);
	DECLARE_PARAMETER(Vec3<double>, rlKp);
	DECLARE_PARAMETER(Vec3<double>, rlKd);
	DECLARE_PARAMETER(double, policy_dt);
	DECLARE_PARAMETER(double, emergency_orientation);
	DECLARE_PARAMETER(double, logData);
	DECLARE_PARAMETER(double, policyDirNum);
};

#endif // PROJECT_MITUSERPARAMETERS_H
