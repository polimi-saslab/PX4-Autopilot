#pragma once

#include "tailsitter.h"

/** Windplane VTOL controller */
class Windplane : public Tailsitter
{
public:
	explicit Windplane(VtolAttitudeControl *attc);
	~Windplane() override = default;

	void fill_actuator_outputs() override;
	void update_transition_state() override;
	void update_fw_state() override;
	void waiting_on_tecs() override;
	void blendThrottleAfterFrontTransition(float scale) override;
	void blendThrottleBeginningBackTransition(float scale) override;

private:
	// Attitude-setpoint throttle history
	float _last_normalized_thr_in_mc{0.f};
	float _last_normalized_thr_in_fw{0.f};
};
