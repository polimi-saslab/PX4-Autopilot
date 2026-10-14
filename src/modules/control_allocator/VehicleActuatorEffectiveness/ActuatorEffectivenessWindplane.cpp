/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "ActuatorEffectivenessWindplane.hpp"

ActuatorEffectivenessWindplane::ActuatorEffectivenessWindplane(ModuleParams *parent) : ModuleParams(parent)
{
}

bool ActuatorEffectivenessWindplane::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// Placeholder allocation matrix: motors 1-4, then control surfaces 1-2.
	// Rows: roll, pitch, yaw, thrust X, thrust Y, thrust Z.
	// Columns: motors 1-4, then control surfaces 1-2.
	static constexpr float effectiveness[NUM_AXES][6] = {
		{1.f, 0.f, 0.f, 0.f, 0.f, 0.f}, // roll
		{0.f, 1.f, 0.f, 0.f, 0.f, 0.f}, // pitch
		{0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, // yaw
		{0.f, 0.f, 0.f, 1.f, 0.f, 0.f}, // thrust X
		{0.f, 0.f, 0.f, 0.f, 1.f, 0.f}, // thrust Y
		{0.f, 0.f, 0.f, 0.f, 0.f, 1.f}, // thrust Z
	};

	configuration.selected_matrix = 0;

	for (int axis = 0; axis < NUM_AXES; ++axis) {
		for (int actuator = 0; actuator < 6; ++actuator) {
			configuration.effectiveness_matrices[0](axis, actuator) = effectiveness[axis][actuator];
		}
	}

	configuration.actuatorsAdded(ActuatorType::MOTORS, 4);
	configuration.actuatorsAdded(ActuatorType::SERVOS, 2);

	return true;
}
