// Inspired in part by InfiniTAM/ITMLib/Utils/ITMSurfelVolumeParameters of the original InfiniTAM repository, Oxford University

#pragma once

#include "Serialization/Serialization.h"


namespace ITMLib {
/**
 * \brief An instance of this struct can be used to specify parameters for a surfel scene.
 */

GENERATE_SERIALIZABLE_STRUCT(
		SurfelVolumeParameters,
/** The maximum fraction by which a new surfel can have a larger radius than the surfel into which it is being fused if full fusion is to occur. */
        (float, delta_radius, 0.5f, PRIMITIVE),
/** The sigma value for the Gaussian used when calculating the sample confidence. */
		(float, gaussian_confidence_sigma, 0.6f, PRIMITIVE),
/** The maximum angle allowed between the normals of a pair of surfels if they are to be merged. */
		(float, max_merge_angle, 20.f * M_PI / 180.f, PRIMITIVE),
/** The maximum distance allowed between a pair of surfels if they are to be merged. */
		(float, max_merge_dist, 0.01f, PRIMITIVE),
/** The maximum radius a surfel is allowed to have. */
		(float, max_surfel_radius, 0.0004f, PRIMITIVE),
/** The minimum factor by which the radii of a pair of surfels must overlap if they are to be merged. */
		(float, min_radius_overlap_factor, 3.5f, PRIMITIVE),
/** The confidence value a surfel must have in order for it to be considered "stable". */
		(float, stable_surfel_confidence, 25.0f, PRIMITIVE),
/** The factor by which to supersample (in each axis) the index image used for finding surfel correspondences. */
		(int, supersampling_factor, 4, PRIMITIVE),
/** The maximum depth a surfel must have in order for it to be used for tracking. */
		(float, tracking_surfel_max_depth, 1.0f, PRIMITIVE),
/** The minimum confidence value a surfel must have in order for it to be used for tracking. */
		(float, tracking_surfel_min_confidence, 5.0f, PRIMITIVE),
/** The number of time steps a surfel is allowed to be unstable without being updated before being removed. */
		(int, unstable_surfel_period, 20, PRIMITIVE),
/** The z offset to apply to unstable surfels when trying to ensure that they are only rendered if there is no stable alternative. */
		(int, unstable_surfel_z_offset, 10000000, PRIMITIVE),
/** Whether or not to use a Gaussian-weighted sample confidence as described in the Keller paper. */
		(bool, use_gaussian_sample_confidence, true, PRIMITIVE),
/** Whether or not to use surfel merging. */
		(bool, use_surfel_merging, true, PRIMITIVE)
);
} // namespace ITMLib
