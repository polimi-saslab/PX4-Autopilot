/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 ****************************************************************************/

#include <gtest/gtest.h>

#include <ControlAllocationPseudoInverse.hpp>
#include <ControlAllocationWeightedPseudoInverse.hpp>

using namespace matrix;

TEST(ControlAllocationWeightedPseudoInverseTest, WeightsSetRedundantActuatorEffort)
{
	ControlAllocationWeightedPseudoInverse allocator;
	Matrix<float, 6, 16> effectiveness;
	effectiveness(0, 0) = 1.f;
	effectiveness(0, 1) = 1.f;

	Vector<float, 16> weights;
	weights.setAll(1.f);
	weights(1) = 4.f;
	allocator.setActuatorWeights(weights);
	allocator.setMetricAllocation(true);
	allocator.setEffectivenessMatrix(effectiveness, {}, {}, 2, false);

	Vector<float, 6> control_setpoint;
	control_setpoint(0) = 1.f;
	allocator.setControlSetpoint(control_setpoint);
	allocator.allocate();

	EXPECT_NEAR(allocator.getActuatorSetpoint()(0), 0.8f, 1e-5f);
	EXPECT_NEAR(allocator.getActuatorSetpoint()(1), 0.2f, 1e-5f);
}

TEST(ControlAllocationWeightedPseudoInverseTest, UnitWeightsMatchPseudoInverse)
{
	ControlAllocationPseudoInverse pseudo_inverse;
	ControlAllocationWeightedPseudoInverse weighted_pseudo_inverse;
	Matrix<float, 6, 16> effectiveness;
	effectiveness(0, 0) = 1.f;
	effectiveness(0, 1) = 1.f;
	effectiveness(1, 0) = 1.f;
	effectiveness(1, 1) = -1.f;

	Vector<float, 16> weights;
	weights.setAll(1.f);
	weighted_pseudo_inverse.setActuatorWeights(weights);
	pseudo_inverse.setMetricAllocation(true);
	weighted_pseudo_inverse.setMetricAllocation(true);
	pseudo_inverse.setEffectivenessMatrix(effectiveness, {}, {}, 2, false);
	weighted_pseudo_inverse.setEffectivenessMatrix(effectiveness, {}, {}, 2, false);

	Vector<float, 6> control_setpoint;
	control_setpoint(0) = 0.6f;
	control_setpoint(1) = -0.2f;
	pseudo_inverse.setControlSetpoint(control_setpoint);
	weighted_pseudo_inverse.setControlSetpoint(control_setpoint);
	pseudo_inverse.allocate();
	weighted_pseudo_inverse.allocate();

	EXPECT_NEAR(weighted_pseudo_inverse.getActuatorSetpoint()(0), pseudo_inverse.getActuatorSetpoint()(0), 1e-5f);
	EXPECT_NEAR(weighted_pseudo_inverse.getActuatorSetpoint()(1), pseudo_inverse.getActuatorSetpoint()(1), 1e-5f);
}
