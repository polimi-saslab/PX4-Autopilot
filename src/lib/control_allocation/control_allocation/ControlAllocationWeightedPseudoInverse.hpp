#pragma once

#include "ControlAllocationPseudoInverse.hpp"

/**
 * @brief Weighted minimum-norm pseudoinverse allocator.
 *
 * For effectiveness matrix B, actuator command u, requested control v,
 * and symmetric positive-definite actuator cost matrix W, the allocation
 * minimizes
 *
 *     u^T W u, subject to B u = v
 *
 * when the requested control is achievable.
 *
 * The weighted pseudoinverse rapresents optimal solution
 *
 *     B_W^+ = W^{-1} B^T (B W^{-1} B^T)^+
 *
 * The implementation uses whitening:
 *
 *     x = W^{1/2} u
 *     B_w = B W^{-1/2}
 *
 *     B_W^+ = W^{-1/2} B_w^+
 *
 * where B_w^+ is computed using the existing pseudoinverse algorithm.
 */
class ControlAllocationWeightedPseudoInverse : public ControlAllocationPseudoInverse
{
public:
	ControlAllocationWeightedPseudoInverse();
	~ControlAllocationWeightedPseudoInverse() override = default;

	/**
	 * @brief Set diagonal actuator effort costs used for whitening.
	 * @param weights Positive per-actuator costs. Larger values discourage use;
	 *        non-finite or non-positive entries are replaced with 1.
	 */
	void setActuatorWeights(const ActuatorVector &weights) override;

protected:
	void updatePseudoInverse() override;

private:
	void computeWeightedPseudoInverse();

	ActuatorVector _actuator_weights;
};
