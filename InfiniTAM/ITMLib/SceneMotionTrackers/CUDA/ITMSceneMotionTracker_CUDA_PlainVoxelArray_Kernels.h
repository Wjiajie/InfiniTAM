//  ================================================================
//  Created by Gregory Kramida on 7/26/18.
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

#include "../../../ORUtils/JetbrainsCUDASyntax.hpp"

namespace {
//CUDA kernels

//TODO: incomplete
template<typename TVoxel, typename TWarp>
__global__ void
computeGradientDataTerm(TVoxel* liveVoxels,
                        TVoxel* canonicalVoxels,
                        TWarp* warpField,
                        const ITMLib::ITMPlainVoxelArray::ITMVoxelArrayInfo* arrayInfo) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;

	Vector3i voxelPosition;

	voxelPosition.x = x + arrayInfo->offset.x;
	voxelPosition.y = y + arrayInfo->offset.y;
	voxelPosition.z = z + arrayInfo->offset.z;


	TVoxel& voxelLive = liveVoxels[locId];
	TVoxel& voxelCanonical = canonicalVoxels[locId];
	if (!VoxelIsConsideredForTracking(voxelCanonical, voxelLive)
	    || !VoxelIsConsideredForDataTerm(voxelCanonical, voxelLive))
		return;
	Vector3f liveSdfJacobian;
	Matrix3f liveSdfHessian;

	ITMLib::ITMPlainVoxelArray::IndexCache liveCache;

	ComputeLiveJacobian_CentralDifferences(
			liveSdfJacobian, voxelPosition, liveVoxels, arrayInfo, liveCache);

};
} //end anonymous namespace