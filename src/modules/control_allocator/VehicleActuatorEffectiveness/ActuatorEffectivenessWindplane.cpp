/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "ActuatorEffectivenessWindplane.hpp"

#include <lib/mathlib/mathlib.h>

using namespace matrix;

ActuatorEffectivenessWindplane::ActuatorEffectivenessWindplane(ModuleParams *parent) : ModuleParams(parent)
{
	for (unsigned i = 0; i < NUM_ROTORS_MAX; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_WP_R%u_MX", i);
		_param_handles[i].effectiveness[ControlAxis::ROLL] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_WP_R%u_MY", i);
		_param_handles[i].effectiveness[ControlAxis::PITCH] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_WP_R%u_MZ", i);
		_param_handles[i].effectiveness[ControlAxis::YAW] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_WP_R%u_FX", i);
		_param_handles[i].effectiveness[ControlAxis::THRUST_X] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_WP_R%u_FY", i);
		_param_handles[i].effectiveness[ControlAxis::THRUST_Y] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_WP_R%u_FZ", i);
		_param_handles[i].effectiveness[ControlAxis::THRUST_Z] = param_find(buffer);
	}

	for (unsigned i = 0; i < NUM_SERVOS_MAX; ++i) {
		const unsigned j = NUM_ROTORS_MAX + i;
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_WP_SV%u_MX", i);
		_param_handles[j].effectiveness[ControlAxis::ROLL] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_WP_SV%u_MY", i);
		_param_handles[j].effectiveness[ControlAxis::PITCH] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_WP_SV%u_MZ", i);
		_param_handles[j].effectiveness[ControlAxis::YAW] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_WP_SV%u_FX", i);
		_param_handles[j].effectiveness[ControlAxis::THRUST_X] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_WP_SV%u_FY", i);
		_param_handles[j].effectiveness[ControlAxis::THRUST_Y] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_WP_SV%u_FZ", i);
		_param_handles[j].effectiveness[ControlAxis::THRUST_Z] = param_find(buffer);
	}

	updateParams();
}

void ActuatorEffectivenessWindplane::updateParams()
{
	ModuleParams::updateParams();

	_wrench.num_rotors = math::min(NUM_ROTORS_MAX, static_cast<int>(_param_ca_rotor_count.get()));
	_wrench.num_servos = math::min(NUM_SERVOS_MAX, static_cast<int>(_param_ca_sv_cs_count.get()));

	for (int i = 0; i < _wrench.num_rotors; ++i) {
		ActuatorWrench &wrench = _wrench._wrenches[i];
		const ParamHandles &handles = _param_handles[i];

		for (int j = 0; j < NUM_AXES; ++j) {
			float effectiveness = 0.f;

			if (param_get(handles.effectiveness[j], &effectiveness) == PX4_OK) {
				wrench.setEffectiveness(static_cast<ControlAxis>(j), effectiveness);
			}
		}
	}

	for (int i = 0; i < _wrench.num_servos; ++i) {
		const int j = NUM_ROTORS_MAX + i;
		ActuatorWrench &wrench = _wrench._wrenches[j];
		const ParamHandles &handles = _param_handles[j];

		for (int k = 0; k < NUM_AXES; ++k) {
			float effectiveness = 0.f;

			if (param_get(handles.effectiveness[k], &effectiveness) == PX4_OK) {
				wrench.setEffectiveness(static_cast<ControlAxis>(k), effectiveness);
			}
		}
	}
}

bool ActuatorEffectivenessWindplane::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	configuration.selected_matrix = 0;
	configuration.wrench_lin_bias[0](ControlAxis::ROLL) = _param_wlb_mx.get();
	configuration.wrench_lin_bias[0](ControlAxis::PITCH) = _param_wlb_my.get();
	configuration.wrench_lin_bias[0](ControlAxis::YAW) = _param_wlb_mz.get();
	configuration.wrench_lin_bias[0](ControlAxis::THRUST_X) = _param_wlb_fx.get();
	configuration.wrench_lin_bias[0](ControlAxis::THRUST_Y) = _param_wlb_fy.get();
	configuration.wrench_lin_bias[0](ControlAxis::THRUST_Z) = _param_wlb_fz.get();

	configuration.wrench_lin_bias_set[0] = true;

	// rotors
	for (int i = 0; i < _wrench.num_rotors; ++i) {
		const ActuatorWrench &wrench = _wrench._wrenches[i];

		if (configuration.addActuator(ActuatorType::MOTORS, wrench.torque, wrench.thrust) < 0) {
			return false;
		}
	}

	// servos
	for (int i = 0; i < _wrench.num_servos; ++i) {
		const ActuatorWrench &wrench = _wrench._wrenches[NUM_ROTORS_MAX + i];

		if (configuration.addActuator(ActuatorType::SERVOS, wrench.torque, wrench.thrust) < 0) {
			return false;
		}
	}

	return true;
}
