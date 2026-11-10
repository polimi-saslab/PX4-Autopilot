#pragma once

#include "tailsitter.h"

/** Windplane VTOL controller */
class Windplane : public Tailsitter
{
public:
	explicit Windplane(VtolAttitudeControl *attc);
	~Windplane() override = default;

	void fill_actuator_outputs() override;

};
