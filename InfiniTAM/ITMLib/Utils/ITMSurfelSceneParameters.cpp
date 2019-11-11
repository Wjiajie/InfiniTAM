//  ================================================================
//  Created by Gregory Kramida on 11/10/19.
//  Copyright (c)  2019 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================
#include "ITMSurfelSceneParameters.h"

using namespace ITMLib;

/**
 * \brief Constructs the default set of surfel scene parameters.
 */
ITMSurfelSceneParameters::ITMSurfelSceneParameters()
//0.5f, 0.6f, static_cast<float>(20 * M_PI / 180), 0.01f, 0.004f, 3.5f, 25.0f, 4, 1.0f,
//5.0f, 20, 10000000, true, true
		: deltaRadius(0.5f),
		  gaussianConfidenceSigma(0.6f),
		  maxMergeAngle(static_cast<float>(20 * M_PI / 180)),
		  maxMergeDist(0.01f),
		  maxSurfelRadius(0.0004f),
		  minRadiusOverlapFactor(3.5f),
		  stableSurfelConfidence(25.0f),
		  supersamplingFactor(4),
		  trackingSurfelMaxDepth(1.0f),
		  trackingSurfelMinConfidence(5.0f),
		  unstableSurfelPeriod(20),
		  unstableSurfelZOffset(10000000),
		  useGaussianSampleConfidence(true),
		  useSurfelMerging(true) {}

/**
     * \brief Constructs a set of surfel scene parameters.
     *
     * \param deltaRadius_                  The maximum fraction by which a new surfel can have a larger radius than the surfel into which it is being fused if full fusion is to occur.
     * \param gaussianConfidenceSigma_      The sigma value for the Gaussian used when calculating the sample confidence.
     * \param maxMergeAngle_                The maximum angle allowed between the normals of a pair of surfels if they are to be merged.
     * \param maxMergeDist_                 The maximum distance allowed between a pair of surfels if they are to be merged.
     * \param maxSurfelRadius_              The maximum radius a surfel is allowed to have.
     * \param minRadiusOverlapFactor_       The minimum factor by which the radii of a pair of surfels must overlap if they are to be merged.
     * \param stableSurfelConfidence_       The confidence value a surfel must have in order for it to be considered "stable".
     * \param supersamplingFactor_          The factor by which to supersample (in each axis) the index image used for finding surfel correspondences.
     * \param trackingSurfelMaxDepth_       The maximum depth a surfel must have in order for it to be used for tracking.
     * \param trackingSurfelMinConfidence_  The minimum confidence value a surfel must have in order for it to be used for tracking.
     * \param unstableSurfelPeriod_         The number of time steps a surfel is allowed to be unstable without being updated before being removed.
     * \param unstableSurfelZOffset_        The z offset to apply to unstable surfels when trying to ensure that they are only rendered if there is no stable alternative.
     * \param useGaussianSampleConfidence_  Whether or not to use a Gaussian-weighted sample confidence as described in the Keller paper.
     * \param useSurfelMerging_             Whether or not to use surfel merging.
     */
ITMSurfelSceneParameters::ITMSurfelSceneParameters(float deltaRadius_, float gaussianConfidenceSigma_,
                                                   float maxMergeAngle_, float maxMergeDist_, float maxSurfelRadius_,
                                                   float minRadiusOverlapFactor_, float stableSurfelConfidence_,
                                                   int supersamplingFactor_, float trackingSurfelMaxDepth_,
                                                   float trackingSurfelMinConfidence_, int unstableSurfelPeriod_,
                                                   int unstableSurfelZOffset_, bool useGaussianSampleConfidence_,
                                                   bool useSurfelMerging_)
		: deltaRadius(deltaRadius_),
		  gaussianConfidenceSigma(gaussianConfidenceSigma_),
		  maxMergeAngle(maxMergeAngle_),
		  maxMergeDist(maxMergeDist_),
		  maxSurfelRadius(maxSurfelRadius_),
		  minRadiusOverlapFactor(minRadiusOverlapFactor_),
		  stableSurfelConfidence(stableSurfelConfidence_),
		  supersamplingFactor(supersamplingFactor_),
		  trackingSurfelMaxDepth(trackingSurfelMaxDepth_),
		  trackingSurfelMinConfidence(trackingSurfelMinConfidence_),
		  unstableSurfelPeriod(unstableSurfelPeriod_),
		  unstableSurfelZOffset(unstableSurfelZOffset_),
		  useGaussianSampleConfidence(useGaussianSampleConfidence_),
		  useSurfelMerging(useSurfelMerging_) {}

/**
* \brief constructs a set of surfel scene parameters from the variable map (typically generated from program arguments)
* \details see code for expected argument names and value types
* \param vm variable map
*/
ITMSurfelSceneParameters::ITMSurfelSceneParameters(const po::variables_map& vm)
		: deltaRadius(vm["surfel_delta_radius"].empty() ? ITMSurfelSceneParameters().deltaRadius
		                                                : vm["surfel_delta_radius"].as<float>()),
		  gaussianConfidenceSigma(vm["surfel_gaussian_convergence_sigma"].empty() ?
		                          ITMSurfelSceneParameters().gaussianConfidenceSigma :
		                          vm["surfel_gaussian_convergence_sigma"].as<float>()),
		  maxMergeAngle(vm["surfel_max_merge_angle"].empty() ?
		                ITMSurfelSceneParameters().maxMergeAngle :
		                vm["surfel_max_merge_angle"].as<float>()),
		  maxMergeDist(vm["surfel_max_merge_distance"].empty() ?
		               ITMSurfelSceneParameters().maxMergeDist :
		               vm["surfel_max_merge_distance"].as<float>()),
		  maxSurfelRadius(vm["surfel_max_radius"].empty() ?
		                  ITMSurfelSceneParameters().maxSurfelRadius :
		                  vm["surfel_max_radius"].as<float>()),
		  minRadiusOverlapFactor(vm["surfel_min_radius_overlap_factor"].empty() ?
		                         ITMSurfelSceneParameters().minRadiusOverlapFactor :
		                         vm["surfel_min_radius_overlap_factor"].as<float>()),
		  stableSurfelConfidence(vm["stable_surfel_confidence"].empty() ?
		                         ITMSurfelSceneParameters().stableSurfelConfidence :
		                         vm["stable_surfel_confidence"].as<float>()),
		  supersamplingFactor(vm["surfel_supersampling_factor"].empty() ?
		                      ITMSurfelSceneParameters().supersamplingFactor :
		                      vm["surfel_supersampling_factor"].as<int>()),
		  trackingSurfelMaxDepth(vm["tracking_surfel_max_depth"].empty() ?
		                         ITMSurfelSceneParameters().trackingSurfelMaxDepth :
		                         vm["tracking_surfel_max_depth"].as<float>()),
		  trackingSurfelMinConfidence(vm["tracking_surfel_min_confidence"].empty() ?
		                              ITMSurfelSceneParameters().trackingSurfelMinConfidence :
		                              vm["tracking_surfel_min_confidence"].as<float>()),
		  unstableSurfelPeriod(vm["unstable_surfel_period"].empty() ?
		                       ITMSurfelSceneParameters().unstableSurfelPeriod :
		                       vm["unstable_surfel_period"].as<int>()),
		  unstableSurfelZOffset(vm["unstable_surfel_z_offset"].empty() ?
		                        ITMSurfelSceneParameters().unstableSurfelZOffset :
		                        vm["unstable_surfel_z_offset"].as<int>()),
		  useGaussianSampleConfidence(vm["disable_gaussian_sample_confidence"].empty() ?
		                              ITMSurfelSceneParameters().useGaussianSampleConfidence :
		                              !vm["disable_gaussian_sample_confidence"].as<bool>()),
		  useSurfelMerging(vm["disable_surfel_merging"].empty() ?
		                   ITMSurfelSceneParameters().useSurfelMerging :
		                   !vm["disable_surfel_merging"].as<bool>()) {}

ITMSurfelSceneParameters ITMSurfelSceneParameters::BuildFromPTree(const pt::ptree& tree) {
	boost::optional<float> deltaRadius_opt = tree.get_optional<float>("surfel_delta_radius");
	boost::optional<float> gaussianConfidenceSigma_opt = tree.get_optional<float>("surfel_gaussian_convergence_sigma");
	boost::optional<float> maxMergeAngle_opt = tree.get_optional<float>("surfel_max_merge_angle");
	boost::optional<float> maxMergeDist_opt = tree.get_optional<float>("surfel_max_merge_distance");
	boost::optional<float> maxSurfelRadius_opt = tree.get_optional<float>("surfel_max_radius");
	boost::optional<float> minRadiusOverlapFactor_opt = tree.get_optional<float>("surfel_min_radius_overlap_factor");
	boost::optional<float> stableSurfelConfidence_opt = tree.get_optional<float>("stable_surfel_confidence");
	boost::optional<int> maxW_opt = tree.get_optional<int>("tracking_surfel_max_depth");
	boost::optional<float> voxelSize_opt = tree.get_optional<float>("voxel_size_meters");
	boost::optional<float> viewFrustum_min_opt = tree.get_optional<float>("view_frustum_near_clipping_distance");
	boost::optional<float> viewFrustum_max_opt = tree.get_optional<float>("view_frustum_far_clipping_distance");
	boost::optional<bool> stopIntegratingAtMaxW_opt = tree.get_optional<bool>("stop_integration_at_max_weight");

	ITMSurfelSceneParameters default_ssp;

	return {mu_opt ? mu_opt.get() : default_ssp.mu,
	        maxW_opt ? maxW_opt.get() : default_ssp.maxW,
	        voxelSize_opt ? voxelSize_opt.get() : default_ssp.voxelSize,
	        viewFrustum_min_opt ? viewFrustum_min_opt.get() : default_ssp.viewFrustum_min,
	        viewFrustum_max_opt ? viewFrustum_max_opt.get() : default_ssp.viewFrustum_max,
	        stopIntegratingAtMaxW_opt ? stopIntegratingAtMaxW_opt.get() : default_ssp.stopIntegratingAtMaxW};
}

