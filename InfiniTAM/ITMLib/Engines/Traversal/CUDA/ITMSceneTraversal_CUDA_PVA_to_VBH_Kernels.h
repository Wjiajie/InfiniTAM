//  ================================================================
//  Created by Gregory Kramida on 10/4/19.
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

//local
#include "../Interface/ITMSceneTraversal.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "../../../Objects/Scene/ITMPlainVoxelArray.h"
#include "../../../Objects/Scene/ITMVoxelBlockHash.h"
#include "../Shared/ITMSceneTraversal_Shared.h"
#include "../../../Utils/Analytics/ITMIsAltered.h"

namespace {
//CUDA kernels

__global__ void findBlocksNotSpannedByArray(
		const ITMLib::ITMPlainVoxelArray::ITMVoxelArrayInfo* arrayInfo,
		const ITMHashEntry* hashTable, int hashEntryCount,
		int* hashesNotSpanned, int* countHashesNotSpanned) {
	int hash = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
	if (hash > hashEntryCount) return;
	const ITMHashEntry& hashEntry = hashTable[hash];
	Vector3i blockPosVoxels = hashEntry.pos.toInt() * SDF_BLOCK_SIZE;
	Vector3i arrayMinCoord = arrayInfo->offset;
	Vector3i arrayMaxCoord = arrayInfo->offset + arrayInfo->size;
	if (blockPosVoxels.x < arrayMinCoord.x || blockPosVoxels.x + SDF_BLOCK_SIZE > arrayMaxCoord.x ||
	    blockPosVoxels.y < arrayMinCoord.y || blockPosVoxels.y + SDF_BLOCK_SIZE > arrayMaxCoord.y ||
	    blockPosVoxels.z < arrayMinCoord.z || blockPosVoxels.z + SDF_BLOCK_SIZE > arrayMaxCoord.z) {
		int unspannedHashIdx = atomicAdd(*countHashesNotSpanned, 1);
		hashesNotSpanned[unspannedHashIdx] = hash;
	}
}

template<typename TVoxelArray, typename TVoxelHash>
__global__ void checkIfArrayContentIsUnalteredOrYieldsTrue(
		const TVoxelArray* arrayVoxels,
		const ITMLib::ITMPlainVoxelArray::ITMVoxelArrayInfo* arrayInfo,
		const TVoxelHash* hashVoxels, const ITMHashEntry* hashTable,
		int hashEntryCount, bool* falseOrAlteredEncountered) {

	if (*falseOrAlteredEncountered) return;
	int xVoxel = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
	int yVoxel = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
	int zVoxel = static_cast<int>(blockIdx.z * blockDim.z + threadIdx.z);

	//TODO: starting index is messed up here!!!! should be determined by passed-in minArrayCoord and maxArrayCoord!
	// minArrayCoord would be the arrayInfo->offset - minIndexingCoord
	// maxArrayCoord would be maxIndexingCoord - (arrayInfo->offset + arrayInfo->size)
	if (xVoxel < arrayInfo->offset.x || xVoxel > arrayInfo->offset.x + arrayInfo->size.x ||
	    yVoxel < arrayInfo->offset.y || yVoxel > arrayInfo->offset.y + arrayInfo->size.y ||
	    zVoxel < arrayInfo->offset.z || zVoxel > arrayInfo->offset.z + arrayInfo->size.z) {
		return;
	}

	Vector3s blockPosition(blockIdx.x, blockIdx.y, blockIdx.z);

	int hash;

	if(FindHashAtPosition(hash, blockPosition, hashTable)){
		//TODO
	}else{
		//if(isAltered(arrayVoxels[]))
	}


}

//template<typename TStaticFunctor, typename TVoxel>

} // anonymous namespace (CUDA kernels)