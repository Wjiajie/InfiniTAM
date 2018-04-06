//  ================================================================
//  Created by Gregory Kramida on 3/2/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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
#pragma once

#include "../../ITMLib/Utils/ITMMath.h"
#include "../../ITMLib/ITMLibDefines.h"

#define COMPUTE_VOXEL_SCALE_HIDE_UNKNOWNS(sdf, flags) (flags == ITMLib::VOXEL_UNKNOWN ? 0.0f : 1.0f - 0.9f * std::abs(sdf))
#define COMPUTE_VOXEL_SCALE(sdf) (1.0f - 0.9f * std::abs(sdf))

/**
 * \brief indexes to be used for coloring different categories of voxels (based on the semantic flags of each voxel)
 * \details Note: @refitem VoxelColorIndexAsCString has magic strings associated with it (static reflection didn't make
 * it yet into the C++17 standard, stay put for C++20), so you'll have to change that if you're going to change any of
 * these enum names here.
 */
enum VoxelColorIndex : int {
	POSITIVE_TRUNCATED_SDF_COLOR_INDEX = 0,
	POSITIVE_NON_TRUNCATED_SDF_COLOR_INDEX = 1,
	NEGATIVE_NON_TRUNCATED_SDF_COLOR_INDEX = 2,
	NEGATIVE_TRUNCATED_SDF_COLOR_INDEX = 3,
	UNKNOWN_SDF_COLOR_INDEX = 4,
	HIGHLIGHT_SDF_COLOR_INDEX = 5,
	COLOR_INDEX_COUNT = 6
};

enum VoxelScaleMode {
	VOXEL_SCALE_HIDE_UNKNOWNS = 0,
	VOXEL_SCALE_SHOW_UNKNOWNS = 1
};

inline
const char* VoxelColorIndexAsCString(const VoxelColorIndex& index){
	switch(index){
		case POSITIVE_TRUNCATED_SDF_COLOR_INDEX:
			return "POSITIVE_TRUNCATED_SDF_COLOR_INDEX";
		case POSITIVE_NON_TRUNCATED_SDF_COLOR_INDEX:
			return "POSITIVE_NON_TRUNCATED_SDF_COLOR_INDEX";
		case NEGATIVE_NON_TRUNCATED_SDF_COLOR_INDEX:
			return "NEGATIVE_NON_TRUNCATED_SDF_COLOR_INDEX";
		case NEGATIVE_TRUNCATED_SDF_COLOR_INDEX:
			return "NEGATIVE_TRUNCATED_SDF_COLOR_INDEX";
		case UNKNOWN_SDF_COLOR_INDEX:
			return "UNKNOWN_SDF_COLOR_INDEX";
		case HIGHLIGHT_SDF_COLOR_INDEX:
			return "HIGHLIGHT_SDF_COLOR_INDEX";
		default:
			std::cerr << "Unrecognized color index: " << index << std::endl;
			return "[UNRECOGNIZED COLOR INDEX]";
	}
}


template<typename TVoxel>
inline
void ComputeVoxelAttributes(const Vector3i& currentBlockPositionVoxels, int x, int y, int z, const TVoxel* localVoxelBlock,
                            vtkPoints* points, vtkFloatArray* scaleAttribute, vtkFloatArray* alternativeScaleAttribute,
                            vtkIntArray* colorAttribute,
                            const ITMLib::ITM3DNestedMapOfArrays<ITMLib::ITMHighlightIterationInfo>& highlights,
                            const int& hash) {
	Vector3i voxelPosition = currentBlockPositionVoxels + Vector3i(x, y, z);
	int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
	const TVoxel& voxel = localVoxelBlock[locId];
	float sdf = TVoxel::valueToFloat(voxel.sdf);
	float voxelScale = COMPUTE_VOXEL_SCALE_HIDE_UNKNOWNS(sdf, voxel.flags);
	float alternativeVoxelScale = COMPUTE_VOXEL_SCALE(sdf);
	bool truncated = voxel.flags == ITMLib::VOXEL_TRUNCATED;
	float voxelColor;
	if(highlights.Contains(hash, locId)){
		voxelColor = HIGHLIGHT_SDF_COLOR_INDEX;
		std::cout << "yoohooo" << std::endl;
	}else{
		voxelColor = voxel.flags == ITMLib::VOXEL_UNKNOWN ? UNKNOWN_SDF_COLOR_INDEX :
	                   sdf > 0 ?
	                   (truncated ? POSITIVE_TRUNCATED_SDF_COLOR_INDEX : POSITIVE_NON_TRUNCATED_SDF_COLOR_INDEX) :
	                   (truncated ? NEGATIVE_TRUNCATED_SDF_COLOR_INDEX : NEGATIVE_NON_TRUNCATED_SDF_COLOR_INDEX);
	}
	// need to filp the y & z axes (unlike InfiniTAM, VTK uses proper right-hand rule system))
	points->InsertNextPoint(voxelPosition.x,
	                        -voxelPosition.y,
	                        -voxelPosition.z);

	scaleAttribute->InsertNextValue(voxelScale);
	alternativeScaleAttribute->InsertNextValue(alternativeVoxelScale);
	colorAttribute->InsertNextValue(voxelColor);
}