#pragma once

#include "ControlAllocationPseudoInverse.hpp"

/** Pseudo-inverse allocator that minimizes weighted actuator effort. */
class ControlAllocationWeightedPseudoInverse : public ControlAllocationPseudoInverse
{
public:
	ControlAllocationWeightedPseudoInverse();
	~ControlAllocationWeightedPseudoInverse() override = default;

	/** Set positive actuator effort costs. Invalid/non-positive entries use unit cost. */
	void setActuatorWeights(const ActuatorVector &weights);

protected:
	void updatePseudoInverse() override;

private:
	void computeWeightedPseudoInverse();

	ActuatorVector _actuator_weights;
};
