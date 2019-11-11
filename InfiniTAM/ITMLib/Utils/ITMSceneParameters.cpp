//  ================================================================
//  Created by Gregory Kramida on 11/9/19.
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

#include "ITMSceneParameters.h"

using namespace ITMLib;

ITMSceneParameters::ITMSceneParameters(float mu, int maxW, float voxelSize,
                                       float viewFrustum_min, float viewFrustum_max, bool stopIntegratingAtMaxW) :
		mu(mu),
		maxW(maxW),
		voxelSize(voxelSize),
		viewFrustum_min(viewFrustum_min),
		viewFrustum_max(viewFrustum_max),
		stopIntegratingAtMaxW(stopIntegratingAtMaxW)
		{}

ITMSceneParameters::ITMSceneParameters():
		mu(0.04f),
		maxW(100),
		voxelSize(0.004f),
		viewFrustum_min(0.2f),
		viewFrustum_max(3.0f),
		stopIntegratingAtMaxW(false)
{}

ITMSceneParameters::ITMSceneParameters(const po::variables_map& vm):
	mu(vm["narrow_band_half_width_meters"].empty() ? ITMSceneParameters().mu : vm["narrow_band_half_width_meters"].as<float>() ),
	maxW(vm["max_integration_weight"].empty() ? ITMSceneParameters().maxW : vm["max_integration_weight"].as<float>() ),
	voxelSize(vm["voxel_size_meters"].empty() ? ITMSceneParameters().voxelSize : vm["voxel_size_meters"].as<float>() ),
	viewFrustum_min(vm["view_frustum_near_clipping_distance"].empty() ? ITMSceneParameters().viewFrustum_min : vm["view_frustum_near_clipping_distance"].as<float>() ),
	viewFrustum_max(vm["view_frustum_far_clipping_distance"].empty() ? ITMSceneParameters().viewFrustum_max : vm["view_frustum_far_clipping_distance"].as<float>() ),
	stopIntegratingAtMaxW(vm["stop_integration_at_max_weight"].empty() ? ITMSceneParameters().stopIntegratingAtMaxW : vm["stop_integration_at_max_weight"].as<bool>() )
{}

ITMSceneParameters ITMSceneParameters::BuildFromPTree(const pt::ptree& tree) {
	boost::optional<float> mu_opt = tree.get_optional<float>("narrow_band_half_width_meters");
	boost::optional<int> maxW_opt = tree.get_optional<int>("max_integration_weight");
	boost::optional<float> voxelSize_opt = tree.get_optional<float>("voxel_size_meters");
	boost::optional<float> viewFrustum_min_opt = tree.get_optional<float>("view_frustum_near_clipping_distance");
	boost::optional<float> viewFrustum_max_opt = tree.get_optional<float>("view_frustum_far_clipping_distance");
	boost::optional<bool> stopIntegratingAtMaxW_opt = tree.get_optional<bool>("stop_integration_at_max_weight");

	ITMSceneParameters default_sp;

	return {mu_opt ? mu_opt.get() : default_sp.mu,
			maxW_opt ? maxW_opt.get() : default_sp.maxW,
			voxelSize_opt ? voxelSize_opt.get() : default_sp.voxelSize,
			viewFrustum_min_opt ? viewFrustum_min_opt.get() : default_sp.viewFrustum_min,
			viewFrustum_max_opt ? viewFrustum_max_opt.get() : default_sp.viewFrustum_max,
			stopIntegratingAtMaxW_opt ? stopIntegratingAtMaxW_opt.get() : default_sp.stopIntegratingAtMaxW};
}
