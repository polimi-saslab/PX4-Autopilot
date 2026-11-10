#include "windplane.h"

#include <uORB/topics/vehicle_torque_setpoint.h>

Windplane::Windplane(VtolAttitudeControl *attc) :
	Tailsitter(attc)
{
}

void Windplane::fill_actuator_outputs()
{
	// Retain tailsitter thrust routing and the existing throttle handover guards.
	Tailsitter::fill_actuator_outputs();

	const hrt_abstime now = hrt_absolute_time();
	float fw_weight = 0.f;

	if (_common_vtol_mode == mode::FIXED_WING) {
		fw_weight = 1.f;

	} else if (_common_vtol_mode == mode::TRANSITION_TO_FW) {
		const float duration = math::max(_param_vt_f_trans_dur.get(), 0.01f);
		fw_weight = math::constrain((now - _transition_start_timestamp) * 1e-6f / duration, 0.f, 1.f);
	}

	const matrix::Vector3f mc_torque(_vehicle_torque_setpoint_virtual_mc->xyz);
	const matrix::Vector3f fw_torque(_vehicle_torque_setpoint_virtual_fw->xyz);
	// Never blend an old or invalid FW sample into an active MC transition.
	const bool fw_valid = _vehicle_torque_setpoint_virtual_fw->timestamp != 0
			      && now - _vehicle_torque_setpoint_virtual_fw->timestamp < 100000
			      && fw_torque.isAllFinite();

	if (_common_vtol_mode == mode::TRANSITION_TO_FW && !fw_valid) {
		fw_weight = 0.f;
	}

	// FW rate control already converts its output to the hover body frame.
	matrix::Vector3f torque;

	if (fw_weight <= 0.f) {
		torque = mc_torque;

	} else if (fw_weight >= 1.f) {
		torque = fw_torque;

	} else {
		torque = (1.f - fw_weight) * mc_torque + fw_weight * fw_torque;
	}

	torque.copyTo(_torque_setpoint_0->xyz);
	_torque_setpoint_0->timestamp_sample = fw_weight >= 1.f
					    ? _vehicle_torque_setpoint_virtual_fw->timestamp_sample
					    : _vehicle_torque_setpoint_virtual_mc->timestamp_sample;

	// Windplane allocates motors and surfaces together using matrix 0.
	matrix::Vector3f().copyTo(_torque_setpoint_1->xyz);
}
