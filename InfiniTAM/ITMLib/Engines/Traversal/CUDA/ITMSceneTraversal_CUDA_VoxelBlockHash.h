//  ================================================================
//  Created by Gregory Kramida on 8/13/19.
//  Copyright (c) 2019 Gregory Kramida
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

#include "../../../Objects/Scene/ITMVoxelBlockHash.h"
#include "../../../../ORUtils/JetbrainsCUDASyntax.hpp"


/* NOTE: this functionality is purposefully kept in headers w/o explicit instantiations to allow for further inlining by
 * the compiler*/
namespace {
//CUDA device functions
template<typename TStaticFunctor, typename TVoxel>
__global__ void
staticVoxelTraversal_device(TVoxel* voxelBlocks, const ITMLib::ITMVoxelBlockHash::IndexData* hashEntries) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	int locId = x + y * hashEntries->size.x + z * hashEntries->size.x * hashEntries->size.y;
	TVoxel& voxel = voxelBlocks[locId];
	TStaticFunctor::run(voxel);
}

template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
__global__ void
dualVoxelPositionTraversal_device(TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
                                  const ITMLib::ITMVoxelBlockHash::IndexData* indexData,
                                  TFunctor& functor) {
	Vector3i globalPos;

	const ITMHashEntry &currentHashEntry = indexData[blockIdx.x];

	if (currentHashEntry.ptr < 0) return;

	globalPos = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

	TVoxel* localVoxelBlock = &(localVBA[currentHashEntry.ptr * SDF_BLOCK_SIZE3]);

	int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;

	Vector3f voxelPosition; int locId;

	locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

	voxelPosition.x = (float)(globalPos.x + x) * _voxelSize;
	voxelPosition.y = (float)(globalPos.y + y) * _voxelSize;
	voxelPosition.z = (float)(globalPos.z + z) * _voxelSize;

	TVoxelPrimary& voxelPrimary = primaryVoxels[locId];
	TVoxelSecondary& voxelSecondary = secondaryVoxels[locId];

	functor(voxelPrimary, voxelSecondary, voxelPosition);

};

}// end anonymous namespace
