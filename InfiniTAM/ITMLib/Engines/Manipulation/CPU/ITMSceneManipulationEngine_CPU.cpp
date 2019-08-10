//  ================================================================
//  Created by Gregory Kramida on 11/3/17.
//  Copyright (c) 2017-2025 Gregory Kramida
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
#include "ITMSceneManipulationEngine_CPU.h"
#include "../../../Utils/ITMMath.h"
#include "../../../Objects/Scene/ITMVoxelBlockHash.h"
#include "../../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../../ITMLibDefines.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "../../../Utils/ITMLibSettings.h"
#include "../../Reconstruction/Interface/ITMSceneReconstructionEngine.h"
#include "../../Reconstruction/ITMSceneReconstructionEngineFactory.h"

namespace ITMLib {

bool AllocateHashEntry_CPU(const Vector3s& hashEntryPosition, ITMHashEntry* hashTable, ITMHashEntry*& resultEntry,
                           int& lastFreeVoxelBlockId, int& lastFreeExcessListId, const int* voxelAllocationList,
                           const int* excessAllocationList, int& hash) {
	hash = hashIndex(hashEntryPosition);
	ITMHashEntry hashEntry = hashTable[hash];
	if (!IS_EQUAL3(hashEntry.pos, hashEntryPosition) || hashEntry.ptr < -1) {
		bool isExcess = false;
		//search excess list only if there is no room in ordered part
		if (hashEntry.ptr >= -1) {
			while (hashEntry.offset >= 1) {
				hash = SDF_BUCKET_NUM + hashEntry.offset - 1;
				hashEntry = hashTable[hash];
				if (IS_EQUAL3(hashEntry.pos, hashEntryPosition) && hashEntry.ptr >= -1) {
					resultEntry = &hashTable[hash];
					return true;
				}
			}
			isExcess = true;

		}
		//still not found, allocate
		if (isExcess && lastFreeVoxelBlockId >= 0 && lastFreeExcessListId >= 0) {
			//there is room in the voxel block array and excess list
			ITMHashEntry newHashEntry;
			newHashEntry.pos = hashEntryPosition;
			newHashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
			newHashEntry.offset = 0;
			int exlOffset = excessAllocationList[lastFreeExcessListId];
			hashTable[hash].offset = exlOffset + 1; //connect to child
			hashTable[SDF_BUCKET_NUM +
			          exlOffset] = newHashEntry; //add child to the excess list
			resultEntry = &hashTable[SDF_BUCKET_NUM +
			                         exlOffset];
			lastFreeVoxelBlockId--;
			lastFreeExcessListId--;
			return true;
		} else if (lastFreeVoxelBlockId >= 0) {
			//there is room in the voxel block array
			ITMHashEntry newHashEntry;
			newHashEntry.pos = hashEntryPosition;
			newHashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
			newHashEntry.offset = 0;
			hashTable[hash] = newHashEntry;
			resultEntry = &hashTable[hash];
			lastFreeVoxelBlockId--;
			return true;
		} else {
			return false;
		}
	} else {
		//HashEntry already exists, return the pointer to it
		resultEntry = &hashTable[hash];
		return true;
	}
}

_CPU_AND_GPU_CODE_ int
FindHashBlock(const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData)* voxelIndex, const THREADPTR(Vector3s)& at) {
	int hash = hashIndex(at);
	while (true)
	{
		ITMHashEntry hashEntry = voxelIndex[hash];

		if (IS_EQUAL3(hashEntry.pos, at) && hashEntry.ptr >= 0)
		{
			return hash;
		}

		if (hashEntry.offset < 1) break;
		hash = SDF_BUCKET_NUM + hashEntry.offset - 1;
	}
	return -1;
}

// stub (mostly)
void GetVoxelHashLocals(int& vmIndex, int& locId, int& xInBlock, int& yInBlock, int& zInBlock,
                        const CONSTPTR(ITMLib::ITMPlainVoxelArray::IndexData)* indexData,
                        ITMLib::ITMPlainVoxelArray::IndexCache& cache, const CONSTPTR(Vector3i)& point) {
	locId = findVoxel(indexData, point, vmIndex);
	xInBlock = point.x;
	yInBlock = point.y;
	zInBlock = point.z;
}
/**
 * \brief Return the exact local positioning indices for a voxel with the given coordinates within a hash data structure
 * \tparam TVoxel type of voxel
 * \param vmIndex 0 if not found, 1 if in cache, positive integer representing hash + 1
 * \param locId
 * \param xInBlock
 * \param yInBlock
 * \param zInBlock
 * \param voxels
 * \param hashEntries
 * \param cache
 * \param point
 */
_CPU_AND_GPU_CODE_
void GetVoxelHashLocals(int& vmIndex, int& locId, int& xInBlock, int& yInBlock, int& zInBlock,
                        const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData)* hashEntries,
                        ITMLib::ITMVoxelBlockHash::IndexCache& cache, const CONSTPTR(Vector3i)& point) {
	Vector3i blockPos;
	int linearIdx = pointToVoxelBlockPos(point, blockPos);
	zInBlock = linearIdx / (SDF_BLOCK_SIZE * SDF_BLOCK_SIZE);
	yInBlock = (linearIdx % (SDF_BLOCK_SIZE * SDF_BLOCK_SIZE)) / SDF_BLOCK_SIZE;
	xInBlock = linearIdx - zInBlock * (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE) - yInBlock * SDF_BLOCK_SIZE;

	if IS_EQUAL3(blockPos, cache.blockPos){
		vmIndex = true;
	}

	int hashIdx = hashIndex(blockPos);

	while (true){
		ITMHashEntry hashEntry = hashEntries[hashIdx];

		if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= 0){
			cache.blockPos = blockPos; cache.blockPtr = hashEntry.ptr * SDF_BLOCK_SIZE3;
			vmIndex = hashIdx + 1; // add 1 to support legacy true / false operations for isFound
		}

		if (hashEntry.offset < 1) break;
		hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
	}

	vmIndex = false;
};

}//namespace ITMLib