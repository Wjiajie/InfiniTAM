// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#pragma once

//boost
#include <boost/program_options.hpp>
namespace po = boost::program_options;

namespace ITMLib {
/**
 * \brief An instance of this struct can be used to specify parameters for a surfel scene.
 */
struct ITMSurfelSceneParameters {
	//#################### PUBLIC VARIABLES ####################

	/** The maximum fraction by which a new surfel can have a larger radius than the surfel into which it is being fused if full fusion is to occur. */
	float deltaRadius;

	/** The sigma value for the Gaussian used when calculating the sample confidence. */
	float gaussianConfidenceSigma;

	/** The maximum angle allowed between the normals of a pair of surfels if they are to be merged. */
	float maxMergeAngle;

	/** The maximum distance allowed between a pair of surfels if they are to be merged. */
	float maxMergeDist;

	/** The maximum radius a surfel is allowed to have. */
	float maxSurfelRadius;

	/** The minimum factor by which the radii of a pair of surfels must overlap if they are to be merged. */
	float minRadiusOverlapFactor;

	/** The confidence value a surfel must have in order for it to be considered "stable". */
	float stableSurfelConfidence;

	/** The factor by which to supersample (in each axis) the index image used for finding surfel correspondences. */
	int supersamplingFactor;

	/** The maximum depth a surfel must have in order for it to be used for tracking. */
	float trackingSurfelMaxDepth;

	/** The minimum confidence value a surfel must have in order for it to be used for tracking. */
	float trackingSurfelMinConfidence;

	/** The number of time steps a surfel is allowed to be unstable without being updated before being removed. */
	int unstableSurfelPeriod;

	/** The z offset to apply to unstable surfels when trying to ensure that they are only rendered if there is no stable alternative. */
	int unstableSurfelZOffset;

	/** Whether or not to use a Gaussian-weighted sample confidence as described in the Keller paper. */
	bool useGaussianSampleConfidence;

	/** Whether or not to use surfel merging. */
	bool useSurfelMerging;

	//#################### CONSTRUCTORS ####################
	ITMSurfelSceneParameters();

	explicit ITMSurfelSceneParameters(const po::variables_map& vm);

	explicit ITMSurfelSceneParameters(float deltaRadius_, float gaussianConfidenceSigma_, float maxMergeAngle_,
	                                  float maxMergeDist_, float maxSurfelRadius_, float minRadiusOverlapFactor_,
	                                  float stableSurfelConfidence_, int supersamplingFactor_,
	                                  float trackingSurfelMaxDepth_, float trackingSurfelMinConfidence_,
	                                  int unstableSurfelPeriod_, int unstableSurfelZOffset_,
	                                  bool useGaussianSampleConfidence_, bool useSurfelMerging_);
};
}
