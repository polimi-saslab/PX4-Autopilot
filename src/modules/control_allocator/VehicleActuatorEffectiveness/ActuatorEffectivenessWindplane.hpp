/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"

#include <px4_platform_common/module_params.h>

class ActuatorEffectivenessWindplane : public ModuleParams, public ActuatorEffectiveness
{
public:

	ActuatorEffectivenessWindplane(ModuleParams *parent);
	virtual ~ActuatorEffectivenessWindplane() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	const char *name() const override { return "Windplane"; }

	static constexpr int NUM_ROTORS_MAX = 12;
	static constexpr int MAX_COUNT = 4;

	static constexpr int NUM_ACTUATORS_TOT_MAX = NUM_ROTORS_MAX + MAX_COUNT;

private:
	void updateParams() override;

	/** @brief Torque and thrust effectiveness contributed by single actuator. */
	struct ActuatorWrench {
		matrix::Vector3f torque{};
		matrix::Vector3f thrust{};

		void setEffectiveness(ControlAxis axis, float value)
		{
			switch (axis) {
			case ControlAxis::ROLL:
				torque(0) = value;
				break;

			case ControlAxis::PITCH:
				torque(1) = value;
				break;

			case ControlAxis::YAW:
				torque(2) = value;
				break;

			case ControlAxis::THRUST_X:
				thrust(0) = value;
				break;

			case ControlAxis::THRUST_Y:
				thrust(1) = value;
				break;

			case ControlAxis::THRUST_Z:
				thrust(2) = value;
				break;
			}
		}
	};

	/** @brief Full wrench. */
	struct Wrench {
		ActuatorWrench _wrenches[NUM_ACTUATORS_TOT_MAX];
		int num_rotors{0};
		int num_servos{0};
	};

	struct ParamHandles {
		param_t effectiveness[NUM_AXES] {};
	};

	ParamHandles _param_handles[NUM_ACTUATORS_TOT_MAX] {};
	Wrench _wrench{};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CA_ROTOR_COUNT>) _param_ca_rotor_count,
		(ParamInt<px4::params::CA_SV_CS_COUNT>) _param_ca_sv_cs_count
	)
};
