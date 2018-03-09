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

#define COMPUTE_VOXEL_SCALE_HIDE_UNKNOWNS(sdf) (sdf == -1.0f ? 0.0f : 1.0f - 0.9f * std::abs(sdf))
#define COMPUTE_VOXEL_SCALE(sdf) (1.0f - 0.9f * std::abs(sdf))


enum VoxelColorIndex : int{
	NEGATIVE_SDF_COLOR_INDEX = 0,
	POSITIVE_SDF_COLOR_INDEX = 1,
	HIGHLIGHT_SDF_COLOR_INDEX = 2
};

enum VoxelScaleMode{
	VOXEL_SCALE_DEFAULT = 0,
	VOXEL_SCALE_ALTERNATIVE = 2
};


template <typename TVoxel>
inline
void ComputeVoxelAttributes(const Vector3i& currentBlockPositionVoxels, int x, int y, int z,
                            const TVoxel* localVoxelBlock, vtkPoints* points,
                            vtkFloatArray* scaleAttribute,
                            vtkFloatArray* alternativeScaleAttribute,
                            vtkIntArray* colorAttribute){
	Vector3i voxelPosition = currentBlockPositionVoxels + Vector3i(x, y, z);
	int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
	const TVoxel& voxel = localVoxelBlock[locId];
	float sdf = TVoxel::valueToFloat(voxel.sdf);
	float voxelScale = COMPUTE_VOXEL_SCALE_HIDE_UNKNOWNS(sdf);
	float alternativeVoxelScale = COMPUTE_VOXEL_SCALE(sdf);
	float voxelColor = voxel.sdf < 0 ? NEGATIVE_SDF_COLOR_INDEX : POSITIVE_SDF_COLOR_INDEX;

	// need to filp the y & z axes (unlike InfiniTAM, VTK uses proper right-hand rule system))
	points->InsertNextPoint(voxelPosition.x,
	                        -voxelPosition.y,
	                        -voxelPosition.z);

	scaleAttribute->InsertNextValue(voxelScale);
	alternativeScaleAttribute->InsertNextValue(alternativeVoxelScale);
	colorAttribute->InsertNextValue(voxelColor);
}
