/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

/**
 * @file ControlAllocationPseudoInverseTest.cpp
 *
 * Tests for Control Allocation Algorithms
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include <gtest/gtest.h>
#include <ControlAllocationPseudoInverse.hpp>

using namespace matrix;

TEST(ControlAllocationTest, ControlLimitsMatchActuatorBoxVertices)
{
	ControlAllocationPseudoInverse method;
	Matrix<float, 6, 16> effectiveness;
	Vector<float, 16> minimum;
	Vector<float, 16> maximum;
	Vector<float, 16> trim;
	Vector<float, 16> linearization_point;

	// Unidirectional motor, reversible motor, and a servo with nonzero trim.
	minimum(0) = 0.f;
	minimum(1) = -1.f;
	minimum(2) = -0.8f;
	maximum(0) = 1.f;
	maximum(1) = 1.f;
	maximum(2) = 0.9f;
	trim(2) = 0.2f;
	linearization_point(0) = 0.4f;

	for (int axis = 0; axis < 5; ++axis) {
		effectiveness(axis, 0) = 2.f * (axis + 1);
		effectiveness(axis, 1) = -3.f;
		effectiveness(axis, 2) = (axis % 2 == 0) ? 4.f : -4.f;
	}

	// Unconfigured columns must not contribute.
	effectiveness(5, 15) = 100.f;
	method.setActuatorMin(minimum);
	method.setActuatorMax(maximum);
	method.setEffectivenessMatrix(effectiveness, trim, linearization_point, 3, false);
	Vector<float, 6> lower;
	Vector<float, 6> upper;
	method.getControlLimits(lower, upper);

	Vector<float, 6> expected_lower;
	Vector<float, 6> expected_upper;

	for (int vertex = 0; vertex < 8; ++vertex) {
		Vector<float, 16> actuator;

		for (int i = 0; i < 3; ++i) {
			actuator(i) = (vertex & (1 << i)) ? maximum(i) : minimum(i);
		}

		const Vector<float, 6> wrench = effectiveness * (actuator - trim);

		for (int axis = 0; axis < 6; ++axis) {
			expected_lower(axis) = vertex == 0 ? wrench(axis) : fminf(expected_lower(axis), wrench(axis));
			expected_upper(axis) = vertex == 0 ? wrench(axis) : fmaxf(expected_upper(axis), wrench(axis));
		}
	}

	for (int axis = 0; axis < 6; ++axis) {
		EXPECT_NEAR(lower(axis), expected_lower(axis), 1e-5f);
		EXPECT_NEAR(upper(axis), expected_upper(axis), 1e-5f);
	}
}

TEST(ControlAllocationTest, ControlLimitsEmptyAndInvalidRanges)
{
	ControlAllocationPseudoInverse method;
	Vector<float, 6> lower;
	Vector<float, 6> upper;
	method.getControlLimits(lower, upper);
	EXPECT_EQ(lower, (Vector<float, 6>{}));
	EXPECT_EQ(upper, (Vector<float, 6>{}));

	Matrix<float, 6, 16> effectiveness;
	effectiveness(0, 0) = -5.f;
	effectiveness(0, 1) = 3.f;
	Vector<float, 16> minimum;
	Vector<float, 16> maximum;
	minimum(0) = 1.f;
	maximum(0) = -1.f; // Invalid: actuator held at trim.
	minimum(1) = 0.5f;
	maximum(1) = 0.5f; // Fixed actuator.
	method.setActuatorMin(minimum);
	method.setActuatorMax(maximum);
	method.setEffectivenessMatrix(effectiveness, {}, {}, 2, false);
	method.getControlLimits(lower, upper);
	EXPECT_FLOAT_EQ(lower(0), 1.5f);
	EXPECT_FLOAT_EQ(upper(0), 1.5f);
}

TEST(ControlAllocationTest, AllZeroCase)
{
	ControlAllocationPseudoInverse method;

	matrix::Vector<float, 6> control_sp;
	matrix::Vector<float, 6> control_allocated;
	matrix::Vector<float, 6> control_allocated_expected;
	matrix::Matrix<float, 6, 16> effectiveness;
	matrix::Vector<float, 16> actuator_sp;
	matrix::Vector<float, 16> actuator_trim;
	matrix::Vector<float, 16> linearization_point;
	matrix::Vector<float, 16> actuator_sp_expected;

	method.setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, 16, false);
	method.setControlSetpoint(control_sp);
	method.allocate();
	method.clipActuatorSetpoint();
	actuator_sp = method.getActuatorSetpoint();
	control_allocated_expected = method.getAllocatedControl();

	EXPECT_EQ(actuator_sp, actuator_sp_expected);
	EXPECT_EQ(control_allocated, control_allocated_expected);
}

TEST(ControlAllocationMetricTest, AllZeroCase)
{
	ControlAllocationPseudoInverse method;

	matrix::Vector<float, 6> control_sp;
	matrix::Vector<float, 6> control_allocated;
	matrix::Vector<float, 6> control_allocated_expected;
	matrix::Matrix<float, 6, 16> effectiveness;
	matrix::Vector<float, 16> actuator_sp;
	matrix::Vector<float, 16> actuator_trim;
	matrix::Vector<float, 16> linearization_point;
	matrix::Vector<float, 16> actuator_sp_expected;

	method.setMetricAllocation(true);
	method.setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, 16, false);
	method.setControlSetpoint(control_sp);
	method.allocate();
	actuator_sp = method.getActuatorSetpoint();
	control_allocated_expected = method.getAllocatedControl();

	EXPECT_EQ(actuator_sp, actuator_sp_expected);
	EXPECT_EQ(control_allocated, control_allocated_expected);
}

TEST(ControlAllocationMetricTest, MetricAllocationSkipsNormalization)
{
	ControlAllocationPseudoInverse normalized;
	ControlAllocationPseudoInverse metric;

	Matrix<float, 6, 16> effectiveness;
	effectiveness(0, 0) = 1.f;
	effectiveness(1, 1) = 2.f;
	effectiveness(2, 2) = 4.f;
	effectiveness(5, 3) = 8.f;

	Vector<float, 16> actuator_trim;
	Vector<float, 16> linearization_point;
	Vector<float, 6> control_sp;
	control_sp(0) = 1.f;
	control_sp(1) = 1.f;
	control_sp(2) = 1.f;
	control_sp(5) = 1.f;

	normalized.setNormalizeRPY(true);
	normalized.setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, 4, true);
	normalized.setControlSetpoint(control_sp);
	normalized.allocate();

	metric.setMetricAllocation(true);
	metric.setNormalizeRPY(true);
	metric.setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, 4, true);
	metric.setControlSetpoint(control_sp);
	metric.allocate();

	EXPECT_NEAR(normalized.getActuatorSetpoint()(0), 1.f / sqrtf(2.f), 1e-5f);
	EXPECT_NEAR(metric.getActuatorSetpoint()(0), 1.f, 1e-5f);
	EXPECT_NEAR(normalized.getActuatorSetpoint()(1), 1.f / (2.f * sqrtf(2.f)), 1e-5f);
	EXPECT_NEAR(metric.getActuatorSetpoint()(1), 0.5f, 1e-5f);
	EXPECT_NEAR(normalized.getActuatorSetpoint()(2), 1.f, 1e-5f);
	EXPECT_NEAR(metric.getActuatorSetpoint()(2), 0.25f, 1e-5f);
	EXPECT_NEAR(normalized.getActuatorSetpoint()(3), 1.f, 1e-5f);
	EXPECT_NEAR(metric.getActuatorSetpoint()(3), 0.125f, 1e-5f);
}
