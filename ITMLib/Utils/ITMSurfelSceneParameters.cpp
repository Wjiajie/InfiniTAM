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
#include "json_utils.h"

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

pt::ptree ITMSurfelSceneParameters::ToPTree() const {
	pt::ptree tree;
	tree.add("surfel_delta_radius", deltaRadius);
	tree.add("surfel_gaussian_convergence_sigma", gaussianConfidenceSigma);
	tree.add("surfel_max_merge_angle", maxMergeAngle);
	tree.add("surfel_max_merge_distance", maxMergeDist);
	tree.add("surfel_max_radius",maxSurfelRadius);
	tree.add("surfel_min_radius_overlap_factor", minRadiusOverlapFactor);
	tree.add("stable_surfel_confidence", stableSurfelConfidence);
	tree.add("surfel_supersampling_factor", supersamplingFactor);
	tree.add("tracking_surfel_max_depth", trackingSurfelMaxDepth);
	tree.add("tracking_surfel_min_confidence", trackingSurfelMinConfidence);
	tree.add("unstable_surfel_period", unstableSurfelPeriod);
	tree.add("unstable_surfel_z_offset", unstableSurfelZOffset);
	tree.add("disable_gaussian_sample_confidence", !useGaussianSampleConfidence);
	tree.add("disable_surfel_merging", !useSurfelMerging);

	return tree;
}

ITMSurfelSceneParameters ITMSurfelSceneParameters::BuildFromPTree(const pt::ptree& tree) {
	boost::optional<float> deltaRadius_opt = tree.get_optional<float>("surfel_delta_radius");
	boost::optional<float> gaussianConfidenceSigma_opt = tree.get_optional<float>("surfel_gaussian_convergence_sigma");
	boost::optional<float> maxMergeAngle_opt = tree.get_optional<float>("surfel_max_merge_angle");
	boost::optional<float> maxMergeDist_opt = tree.get_optional<float>("surfel_max_merge_distance");
	boost::optional<float> maxSurfelRadius_opt = tree.get_optional<float>("surfel_max_radius");
	boost::optional<float> minRadiusOverlapFactor_opt = tree.get_optional<float>("surfel_min_radius_overlap_factor");
	boost::optional<float> stableSurfelConfidence_opt = tree.get_optional<float>("stable_surfel_confidence");

	boost::optional<int> supersamplingFactor_opt = tree.get_optional<int>("surfel_supersampling_factor");
	boost::optional<float> trackingSurfelMaxDepth_opt = tree.get_optional<float>("tracking_surfel_max_depth");
	boost::optional<float> trackingSurfelMinConfidence_opt = tree.get_optional<float>("tracking_surfel_min_confidence");
	boost::optional<int> unstableSurfelPeriod_opt = tree.get_optional<int>("unstable_surfel_period");
	boost::optional<int> unstableSurfelZOffset_opt = tree.get_optional<int>("unstable_surfel_z_offset");
	boost::optional<bool> disableGaussianSampleConfidence_opt = tree.get_optional<bool>("disable_gaussian_sample_confidence");
	boost::optional<bool> disableSurfelMerging_opt = tree.get_optional<bool>("disable_surfel_merging");

	ITMSurfelSceneParameters default_ssp;

	return {deltaRadius_opt ? deltaRadius_opt.get() : default_ssp.deltaRadius,
	        gaussianConfidenceSigma_opt ? gaussianConfidenceSigma_opt.get() : default_ssp.gaussianConfidenceSigma,
	        maxMergeAngle_opt ? maxMergeAngle_opt.get() : default_ssp.maxMergeAngle,
	        maxMergeDist_opt ? maxMergeDist_opt.get() : default_ssp.maxMergeDist,
	        maxSurfelRadius_opt ? maxSurfelRadius_opt.get() : default_ssp.maxSurfelRadius,
	        minRadiusOverlapFactor_opt ? minRadiusOverlapFactor_opt.get() : default_ssp.minRadiusOverlapFactor,
	        stableSurfelConfidence_opt ? stableSurfelConfidence_opt.get() : default_ssp.stableSurfelConfidence,
	        supersamplingFactor_opt ? supersamplingFactor_opt.get() : default_ssp.supersamplingFactor,
	        trackingSurfelMaxDepth_opt ? trackingSurfelMaxDepth_opt.get() : default_ssp.trackingSurfelMaxDepth,
	        trackingSurfelMinConfidence_opt ? trackingSurfelMinConfidence_opt.get() : default_ssp.trackingSurfelMinConfidence,
	        unstableSurfelPeriod_opt ? unstableSurfelPeriod_opt.get() : default_ssp.unstableSurfelPeriod,
	        unstableSurfelZOffset_opt ? unstableSurfelZOffset_opt.get() : default_ssp.unstableSurfelZOffset,
	        disableGaussianSampleConfidence_opt ? !disableGaussianSampleConfidence_opt.get() : default_ssp.useGaussianSampleConfidence,
	        disableSurfelMerging_opt ? !disableSurfelMerging_opt.get() : default_ssp.useSurfelMerging};
}

namespace ITMLib {
bool operator==(const ITMSurfelSceneParameters& p1, const ITMSurfelSceneParameters& p2) {
	return p1.deltaRadius == p2.deltaRadius &&
	       p1.gaussianConfidenceSigma == p2.gaussianConfidenceSigma &&
	       p1.maxMergeAngle == p2.maxMergeAngle &&
	       p1.maxMergeDist == p2.maxMergeDist &&
	       p1.maxSurfelRadius == p2.maxSurfelRadius &&
	       p1.minRadiusOverlapFactor == p2.minRadiusOverlapFactor &&
	       p1.stableSurfelConfidence == p2.stableSurfelConfidence &&
	       p1.minRadiusOverlapFactor == p2.minRadiusOverlapFactor &&
	       p1.stableSurfelConfidence == p2.stableSurfelConfidence &&
	       p1.supersamplingFactor == p2.supersamplingFactor &&
	       p1.trackingSurfelMaxDepth == p2.trackingSurfelMaxDepth &&
	       p1.trackingSurfelMinConfidence == p2.trackingSurfelMinConfidence &&
	       p1.unstableSurfelPeriod == p2.unstableSurfelPeriod &&
	       p1.unstableSurfelZOffset == p2.unstableSurfelZOffset &&
	       p1.useGaussianSampleConfidence == p2.useGaussianSampleConfidence &&
	       p1.useSurfelMerging == p2.useSurfelMerging;
}

std::ostream& operator<<(std::ostream& out, const ITMSurfelSceneParameters& p){
	pt::ptree tree(p.ToPTree());
	pt::write_json_no_quotes(out, tree, true);
}

}//namespace ITMLib