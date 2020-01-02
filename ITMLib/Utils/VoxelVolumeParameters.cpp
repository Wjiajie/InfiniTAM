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

#include "VoxelVolumeParameters.h"
#include "../Utils/json_utils.h"

using namespace ITMLib;

VoxelVolumeParameters::VoxelVolumeParameters(float narrow_band_half_width, int max_integration_weight, float voxel_size,
                                             float near_clipping_distance, float far_clipping_distance, bool stop_integration_at_max_weight) :
		narrow_band_half_width(narrow_band_half_width),
		max_integration_weight(max_integration_weight),
		voxel_size(voxel_size),
		near_clipping_distance(near_clipping_distance),
		far_clipping_distance(far_clipping_distance),
		stop_integration_at_max_weight(stop_integration_at_max_weight) {}

VoxelVolumeParameters::VoxelVolumeParameters() :
		narrow_band_half_width(0.04f),
		max_integration_weight(100),
		voxel_size(0.004f),
		near_clipping_distance(0.2f),
		far_clipping_distance(3.0f),
		stop_integration_at_max_weight(false) {}

VoxelVolumeParameters::VoxelVolumeParameters(const po::variables_map& vm) :
		narrow_band_half_width(vm["narrow_band_half_width"].empty() ? VoxelVolumeParameters().narrow_band_half_width
		                                                                   : vm["narrow_band_half_width"].as<float>()),
		max_integration_weight(vm["max_integration_weight"].empty() ? VoxelVolumeParameters().max_integration_weight
		                                                            : vm["max_integration_weight"].as<float>()),
		voxel_size(
				vm["voxel_size"].empty() ? VoxelVolumeParameters().voxel_size : vm["voxel_size"].as<float>()),
		near_clipping_distance(vm["near_clipping_distance"].empty() ? VoxelVolumeParameters().near_clipping_distance
		                                                                         : vm["near_clipping_distance"].as<float>()),
		far_clipping_distance(vm["far_clipping_distance"].empty() ? VoxelVolumeParameters().far_clipping_distance
		                                                                       : vm["far_clipping_distance"].as<float>()),
		stop_integration_at_max_weight(vm["stop_integration_at_max_weight"].empty() ? VoxelVolumeParameters().stop_integration_at_max_weight
		                                                                            : vm["stop_integration_at_max_weight"].as<bool>()) {}

pt::ptree VoxelVolumeParameters::ToPTree() const {
	pt::ptree tree;
	tree.add("narrow_band_half_width", narrow_band_half_width);
	tree.add("max_integration_weight", max_integration_weight);
	tree.add("voxel_size", voxel_size);
	tree.add("near_clipping_distance", near_clipping_distance);
	tree.add("far_clipping_distance", far_clipping_distance);
	tree.add("stop_integration_at_max_weight", stop_integration_at_max_weight);
	return tree;
}

VoxelVolumeParameters VoxelVolumeParameters::BuildFromPTree(const pt::ptree& tree) {
	boost::optional<float> mu_opt = tree.get_optional<float>("narrow_band_half_width");
	boost::optional<int> maxW_opt = tree.get_optional<int>("max_integration_weight");
	boost::optional<float> voxelSize_opt = tree.get_optional<float>("voxel_size");
	boost::optional<float> viewFrustum_min_opt = tree.get_optional<float>("near_clipping_distance");
	boost::optional<float> viewFrustum_max_opt = tree.get_optional<float>("far_clipping_distance");
	boost::optional<bool> stopIntegratingAtMaxW_opt = tree.get_optional<bool>("stop_integration_at_max_weight");

	VoxelVolumeParameters default_sp;

	return {mu_opt ? mu_opt.get() : default_sp.narrow_band_half_width,
	        maxW_opt ? maxW_opt.get() : default_sp.max_integration_weight,
	        voxelSize_opt ? voxelSize_opt.get() : default_sp.voxel_size,
	        viewFrustum_min_opt ? viewFrustum_min_opt.get() : default_sp.near_clipping_distance,
	        viewFrustum_max_opt ? viewFrustum_max_opt.get() : default_sp.far_clipping_distance,
	        stopIntegratingAtMaxW_opt ? stopIntegratingAtMaxW_opt.get() : default_sp.stop_integration_at_max_weight};
}

namespace ITMLib{
bool operator==(const VoxelVolumeParameters& p1, const VoxelVolumeParameters& p2) {
	return p1.voxel_size == p2.voxel_size &&
	       p1.near_clipping_distance == p2.near_clipping_distance &&
	       p1.far_clipping_distance == p2.far_clipping_distance &&
	       p1.narrow_band_half_width == p2.narrow_band_half_width &&
	       p1.max_integration_weight == p2.max_integration_weight &&
	       p1.stop_integration_at_max_weight == p2.stop_integration_at_max_weight;
}

std::ostream& operator<<(std::ostream& out, const VoxelVolumeParameters& p){
	pt::ptree tree(p.ToPTree());
	pt::write_json_no_quotes(out, tree, true);
}

}//namespace ITMLib