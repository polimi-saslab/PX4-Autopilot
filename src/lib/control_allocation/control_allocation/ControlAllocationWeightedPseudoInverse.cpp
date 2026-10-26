#include "ControlAllocationWeightedPseudoInverse.hpp"

ControlAllocationWeightedPseudoInverse::ControlAllocationWeightedPseudoInverse()
{
	_actuator_weights.setAll(1.f);
}

void ControlAllocationWeightedPseudoInverse::setActuatorWeights(const ActuatorVector &weights)
{
	for (int i = 0; i < NUM_ACTUATORS; ++i) {
		_actuator_weights(i) = (PX4_ISFINITE(weights(i)) && weights(i) > FLT_EPSILON) ? weights(i) : 1.f;
	}

	_mix_update_needed = true;
	_normalization_needs_update = true;
}

void ControlAllocationWeightedPseudoInverse::updatePseudoInverse()
{
	if (_mix_update_needed) {
		computeWeightedPseudoInverse();

		if (!_metric_allocation) {
			if (_normalization_needs_update && !_had_actuator_failure) {
				updateControlAllocationMatrixScale();
				_normalization_needs_update = false;
			}

			normalizeControlAllocationMatrix();
		}

		_mix_update_needed = false;
	}
}

void ControlAllocationWeightedPseudoInverse::computeWeightedPseudoInverse()
{
	// M = W^-1 B^T (B W^-1 B^T)^+
	matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> weighted_effectiveness = _effectiveness;

	for (int i = 0; i < NUM_ACTUATORS; ++i) {
		weighted_effectiveness.col(i) /= _actuator_weights(i);
	}

	const matrix::SquareMatrix<float, NUM_AXES> gram = weighted_effectiveness * _effectiveness.transpose();
	matrix::SquareMatrix<float, NUM_AXES> gram_pseudo_inverse;
	matrix::geninv(gram, gram_pseudo_inverse);
	_mix = weighted_effectiveness.transpose() * gram_pseudo_inverse;
}
