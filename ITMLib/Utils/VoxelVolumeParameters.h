//  ================================================================
//  Created by Gregory Kramida on 01/03/20.
//  Copyright (c) 2020 Gregory Kramida
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

// inspired in part by InifinTAM/Utils/ITMSceneParameters.h of the original InfiniTAM repository, Oxford University

#pragma once

//local
#include "Serialization/Serialization.h"

//boost
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>

namespace po = boost::program_options;
namespace pt = boost::property_tree;

namespace ITMLib {

#define VOXEL_VOLUME_PARAMETERS_STRUCT_DESCRIPTION \
        VoxelVolumeParameters, \
        (float, voxel_size, 0.004f, PRIMITIVE, "Size of a voxel, usually given in meters"), \
        (float, near_clipping_distance, 0.2f, PRIMITIVE, \
        "Consider only depth values between near_clipping_distance to far_clipping_distance."), \
        (float, far_clipping_distance, 3.0f, PRIMITIVE, \
        "Consider only depth values between near_clipping_distance to far_clipping_distance."), \
        (float, narrow_band_half_width, 0.04f, PRIMITIVE, \
        "Encodes the width of the band of the truncated signed distance transform that is actually stored in the " \
        "volume. This is again usually specified in meters. " \
        "The resulting width in voxels isnarrow_band_half_width divided by voxel_size."), \
        (int, max_integration_weight, 100, PRIMITIVE, \
        "Up to max_integration_weight observations per voxel are averaged. " \
        "Beyond that a sliding average is computed."), \
        (bool, stop_integration_at_max_weight, false, PRIMITIVE, \
        "Whether to stop integration for a given voxel when its max_integration_weight is reached."), \
        (bool, add_extra_block_ring_during_allocation, false, PRIMITIVE, \
        "(Voxel block hash indexing only) allocates an extra 1-ring of hash blocks around " \
        "the depth-based ones."), \
        (float, block_allocation_band_factor, 2.0f, PRIMITIVE, \
        "(Voxel block hash indexing only) factor of narrow band width that will be " \
        "considered for depth-based allocation. For instance, a factor of 2 will make " \
        "sure that blocks that are twice as far from the surface as the boundary of the " \
        "narrow (non-truncated) TSDF band will be allocated")



#pragma message (BOOST_PP_STRINGIZE(GENERATE_SERIALIZABLE_STRUCT(\
        VoxelVolumeParameters, \
        (float, voxel_size, 0.004f, PRIMITIVE, "Size of a voxel, usually given in meters"), \
        (float, near_clipping_distance, 0.2f, PRIMITIVE, \
        "Consider only depth values between near_clipping_distance to far_clipping_distance."), \
        (float, far_clipping_distance, 3.0f, PRIMITIVE, \
        "Consider only depth values between near_clipping_distance to far_clipping_distance.") \
)))
//
//struct VoxelVolumeParameters {
//	float voxel_size = 0.004f;
//	=; =;
//	std::string origin = "";
//	VoxelVolumeParameters() = default;
//	VoxelVolumeParameters(float voxel_size,,
//	,
//	const std::string& origin = ""
//	):
//
//	voxel_size ( std::move(voxel_size)), ( std::move()), ( std::move()), origin(origin) {}
//
//	explicit VoxelVolumeParameters(const boost::program_options::variables_map& vm, const std::string& origin = "")
//			: voxel_size(vm["voxel_size"].empty() ? VoxelVolumeParameters().voxel_size : vm["voxel_size"].as<float>()),
//			  SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT_(VoxelVolumeParameters, , ,),
//			  SERIALIZABLE_STRUCT_IMPL_FIELD_VM_INIT_(VoxelVolumeParameters, , ,), origin(origin) {}
//
//	static VoxelVolumeParameters BuildFromPTree(const boost::property_tree::ptree& tree,
//	                                            const std::string& origin = "") {
//		VoxelVolumeParameters default_instance;
//		boost::optional<float> voxel_size = tree.get_optional<float>("voxel_size");
//		SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE_(, ,)
//		SERIALIZABLE_STRUCT_IMPL_FIELD_OPTIONAL_FROM_TREE_(, ,)
//		return {voxel_size ? voxel_size.get()
//		                   : default_instance.voxel_size, ? .get() : default_instance., ? .get() : default_instance., origin};
//	}
//
//	boost::property_tree::ptree ToPTree(const std::string& origin = "") const {
//		boost::property_tree::ptree tree;
//		tree.add("voxel_size", voxel_size);
//		SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE_(,)
//		SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_TREE_(,)
//		return tree;
//	}
//
//	friend bool operator==(const VoxelVolumeParameters& instance1, const VoxelVolumeParameters& instance2) {
//		return instance1.voxel_size == instance2.voxel_size && instance1.== instance2.&& instance1.== instance2.;
//	}
//
//	friend std::ostream& operator<<(std::ostream& out, const VoxelVolumeParameters& instance) {
//		boost::property_tree::ptree tree(instance.ToPTree());
//		boost::property_tree::write_json_no_quotes(out, tree, true);
//		return out;
//	}
//
//	static void AddToOptionsDescription(boost::program_options::options_description& od) {
//		od.add_options()(generate_cli_argument_identifiers_snake_case("voxel_size", od).c_str(),
//		                 boost::program_options::value<float>()->default_value(0.004f),
//		                 "\"Size of a voxel, usually given in meters\"");
//		SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_OPTIONS_DESCRIPTION_(, , ,)
//		SERIALIZABLE_STRUCT_IMPL_ADD_FIELD_TO_OPTIONS_DESCRIPTION_(, , ,);
//	}
//}

//====
/** \brief
	Stores parameters of a voxel volume, such as voxel size
*/
//GENERATE_SERIALIZABLE_STRUCT(VOXEL_VOLUME_PARAMETERS_STRUCT_DESCRIPTION);
} // namespace ITMLib
