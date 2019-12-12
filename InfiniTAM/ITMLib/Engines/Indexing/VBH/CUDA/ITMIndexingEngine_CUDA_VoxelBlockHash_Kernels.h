//  ================================================================
//  Created by Gregory Kramida on 10/8/19.
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

#include "../../../../../ORUtils/JetbrainsCUDASyntax.hpp"
#include "../../../../Utils/ITMCUDAUtils.h"
#include "../../../../Objects/Scene/ITMGlobalCache.h"
#include "../../../Manipulation/Shared/ITMSceneManipulationEngine_Shared.h"
#include "../../Shared/ITMIndexingEngine_Shared.h"

namespace {
//CUDA kernels

//TODO: provide a better nomenclature for hash block visibility, i.e. an enum inheriting from unsigned char
__global__ void setToType3(uchar* entriesVisibleType, int* visibleEntryIDs, int noVisibleEntries) {
	int entryId = threadIdx.x + blockIdx.x * blockDim.x;
	if (entryId > noVisibleEntries - 1) return;
	entriesVisibleType[visibleEntryIDs[entryId]] = 3;
}

__global__
void allocateHashedVoxelBlocksUsingLists_SetVisibility_device(
		int* voxelAllocationList, int* excessAllocationList,
		AllocationTempData* allocData,
		ITMHashEntry* hashTable, int noTotalEntries,
		const ITMLib::HashEntryState* hashEntryStates, Vector3s* blockCoords,
		uchar* entriesVisibleType) {
	int hashCode = threadIdx.x + blockIdx.x * blockDim.x;
	if (hashCode >= noTotalEntries) return;
	int vbaIdx, exlIdx;

	switch (hashEntryStates[hashCode]) {
		case ITMLib::NEEDS_ALLOCATION_IN_ORDERED_LIST: //needs allocation, fits in the ordered list
			vbaIdx = atomicSub(&allocData->noAllocatedVoxelEntries, 1);

			if (vbaIdx >= 0) //there is room in the voxel block array
			{
				ITMHashEntry hashEntry;
				hashEntry.pos = blockCoords[hashCode];
				hashEntry.ptr = voxelAllocationList[vbaIdx];
				hashEntry.offset = 0;
				hashTable[hashCode] = hashEntry;
			} else {
				entriesVisibleType[hashCode] = 0;
				// Restore the previous value to avoid leaks.
				atomicAdd(&allocData->noAllocatedVoxelEntries, 1);
			}
			break;

		case ITMLib::NEEDS_ALLOCATION_IN_EXCESS_LIST: //needs allocation in the excess list
			vbaIdx = atomicSub(&allocData->noAllocatedVoxelEntries, 1);
			exlIdx = atomicSub(&allocData->noAllocatedExcessEntries, 1);

			if (vbaIdx >= 0 && exlIdx >= 0) //there is room in the voxel block array and excess list
			{
				ITMHashEntry hashEntry;
				hashEntry.pos = blockCoords[hashCode];
				hashEntry.ptr = voxelAllocationList[vbaIdx];
				hashEntry.offset = 0;

				int exlOffset = excessAllocationList[exlIdx];

				hashTable[hashCode].offset = exlOffset + 1; //connect to child

				hashTable[ORDERED_LIST_SIZE + exlOffset] = hashEntry; //add child to the excess list
				entriesVisibleType[ORDERED_LIST_SIZE + exlOffset] = 1;
			} else {
				// Restore the previous values to avoid leaks.
				atomicAdd(&allocData->noAllocatedVoxelEntries, 1);
				atomicAdd(&allocData->noAllocatedExcessEntries, 1);
			}

			break;
	}
}


__global__
void allocateHashedVoxelBlocksUsingLists_device(
		int* voxelAllocationList, int* excessAllocationList,
		AllocationTempData* allocData,
		ITMHashEntry* hashTable, const int hashEntryCount,
		const ITMLib::HashEntryState* hashEntryStates, Vector3s* blockCoords) {
	int hashCode = threadIdx.x + blockIdx.x * blockDim.x;
	if (hashCode >= hashEntryCount) return;

	int vbaIdx, exlIdx;

	switch (hashEntryStates[hashCode]) {
		case ITMLib::NEEDS_ALLOCATION_IN_ORDERED_LIST: //needs allocation, fits in the ordered list
			vbaIdx = atomicSub(&allocData->noAllocatedVoxelEntries, 1);

			if (vbaIdx >= 0) //there is room in the voxel block array
			{
				ITMHashEntry hashEntry;
				hashEntry.pos = blockCoords[hashCode];
				hashEntry.ptr = voxelAllocationList[vbaIdx];
				hashEntry.offset = 0;
				printf("hash entry being allocated: %d %d %d, code %d\n", hashEntry.pos.x, hashEntry.pos.y, hashEntry.pos.z, hashCode);
				hashTable[hashCode] = hashEntry;
			} else {
				// Restore the previous value to avoid leaks.
				atomicAdd(&allocData->noAllocatedVoxelEntries, 1);
			}
			break;

		case ITMLib::NEEDS_ALLOCATION_IN_EXCESS_LIST: //needs allocation in the excess list
			vbaIdx = atomicSub(&allocData->noAllocatedVoxelEntries, 1);
			exlIdx = atomicSub(&allocData->noAllocatedExcessEntries, 1);

			if (vbaIdx >= 0 && exlIdx >= 0) //there is room in the voxel block array and excess list
			{
				ITMHashEntry hashEntry;
				hashEntry.pos = blockCoords[hashCode];
				hashEntry.ptr = voxelAllocationList[vbaIdx];
				hashEntry.offset = 0;

				int exlOffset = excessAllocationList[exlIdx];

				hashTable[hashCode].offset = exlOffset + 1; //connect to child

				hashTable[ORDERED_LIST_SIZE + exlOffset] = hashEntry; //add child to the excess list
				printf("hash entry being allocated: %d %d %d, code %d\n", hashEntry.pos.x, hashEntry.pos.y, hashEntry.pos.z, hashCode);
			} else {
				// Restore the previous values to avoid leaks.
				atomicAdd(&allocData->noAllocatedVoxelEntries, 1);
				atomicAdd(&allocData->noAllocatedExcessEntries, 1);
			}

			break;
	}
}

__global__
void buildHashAllocationTypeList_VolumeToVolume(ITMLib::HashEntryState* hashEntryStates, Vector3s* blockCoordinates,
                                                ITMHashEntry* targetHashTable, const ITMHashEntry* sourceHashTable,
                                                int hashEntryCount,
                                                bool* collisionDetected) {
	int hashCode = threadIdx.x + blockIdx.x * blockDim.x;
	if (hashCode >= hashEntryCount) return;
	const ITMHashEntry& sourceHashEntry = sourceHashTable[hashCode];
	if (sourceHashEntry.ptr < 0) return;
	Vector3s sourceBlockCoordinates = sourceHashEntry.pos;
	int targetHash = hashIndex(sourceBlockCoordinates);
	MarkAsNeedingAllocationIfNotFound(hashEntryStates, blockCoordinates,
	                                  targetHash, sourceBlockCoordinates, targetHashTable,
	                                  *collisionDetected);
}


__global__ void buildHashAllocAndVisibleType_device(ITMLib::HashEntryState* hashEntryStates, uchar* entriesVisibleType,
                                                    Vector3s* blockCoords, const float* depth,
                                                    Matrix4f invertedCameraTransform, Vector4f projParams_d, float mu,
                                                    Vector2i _imgSize, float oneOverHashBlockSizeMeters,
                                                    ITMHashEntry* hashTable, float viewFrustum_min,
                                                    float viewFrustum_max, bool* collisionDetected) {
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x > _imgSize.x - 1 || y > _imgSize.y - 1) return;

	buildHashAllocAndVisibleTypePP(hashEntryStates, entriesVisibleType, x, y, blockCoords, depth,
	                               invertedCameraTransform, projParams_d,
	                               mu, _imgSize, oneOverHashBlockSizeMeters,
	                               hashTable, viewFrustum_min, viewFrustum_max, *collisionDetected);
}


template<bool useSwapping>
__global__ void
buildVisibleList_device(ITMHashEntry* hashTable, ITMLib::ITMHashSwapState* swapStates, int noTotalEntries,
                        int* visibleEntryIDs, AllocationTempData* allocData, uchar* entriesVisibleType,
                        Matrix4f M_d, Vector4f projParams_d, Vector2i depthImgSize, float voxelSize) {
	int targetIdx = threadIdx.x + blockIdx.x * blockDim.x;
	if (targetIdx > noTotalEntries - 1) return;

	__shared__ bool shouldPrefix;
	shouldPrefix = false;
	__syncthreads();

	unsigned char hashVisibleType = entriesVisibleType[targetIdx];
	const ITMHashEntry& hashEntry = hashTable[targetIdx];

	if (hashVisibleType == 3) {
		bool isVisibleEnlarged, isVisible;

		if (useSwapping) {
			checkBlockVisibility<true>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize,
			                           depthImgSize);
			if (!isVisibleEnlarged) hashVisibleType = 0;
		} else {
			checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize,
			                            depthImgSize);
			if (!isVisible) hashVisibleType = 0;
		}
		entriesVisibleType[targetIdx] = hashVisibleType;
	}

	if (hashVisibleType > 0) shouldPrefix = true;

	if (useSwapping) {
		if (hashVisibleType > 0 && swapStates[targetIdx].state != 2) swapStates[targetIdx].state = 1;
	}

	__syncthreads();

	if (shouldPrefix) {
		int offset = computePrefixSum_device<int>(hashVisibleType > 0, &allocData->noVisibleEntries,
		                                          blockDim.x * blockDim.y, threadIdx.x);
		if (offset != -1) visibleEntryIDs[offset] = targetIdx;
	}

}

__global__ void
reAllocateSwappedOutVoxelBlocks_device(int* voxelAllocationList, ITMHashEntry* hashTable, int hashEntryCount,
                                       AllocationTempData* allocData, /*int *countOfAllocatedOrderedEntries,*/
                                       uchar* entriesVisibleType) {
	int hashCode = threadIdx.x + blockIdx.x * blockDim.x;
	if (hashCode >= hashEntryCount) return;

	int vbaIdx;
	int hashEntry_ptr = hashTable[hashCode].ptr;

	if (entriesVisibleType[hashCode] > 0 &&
	    hashEntry_ptr == -1) //it is visible and has been previously allocated inside the hash, but deallocated from VBA
	{
		vbaIdx = atomicSub(&allocData->noAllocatedVoxelEntries, 1);
		if (vbaIdx >= 0) hashTable[hashCode].ptr = voxelAllocationList[vbaIdx];
		else atomicAdd(&allocData->noAllocatedVoxelEntries, 1);
	}
}

__global__ void findHashEntry_device(ITMHashEntry* entry, const ITMHashEntry* hashTable, Vector3s pos, int* hashCode) {

	*hashCode = FindHashCodeAt(hashTable, pos);
	if (*hashCode != -1) {
		const ITMHashEntry& targetEntry = hashTable[*hashCode];
		entry->offset = targetEntry.offset;
		entry->ptr = targetEntry.ptr;
		entry->pos = targetEntry.pos;
	}
}

__global__ void markForAllocationBasedOnOneRingAroundAnotherAllocation_device(
		ITMLib::HashEntryState* hashEntryStates, Vector3s* blockCoordinates, bool* collisionDetected,
		const ITMHashEntry* sourceHashTable, const ITMHashEntry* targetHashTable, int hashEntryCount) {

	if (*collisionDetected) return;
	int hashCode = blockIdx.x;

	if (hashCode >= hashEntryCount) return;
	const ITMHashEntry& sourceHashEntry = sourceHashTable[hashCode];
	if (sourceHashEntry.ptr < 0) return;

	Vector3s targetBlockCoordinates = sourceHashEntry.pos + Vector3s(threadIdx.x - 1, threadIdx.y - 1, threadIdx.z -1);

	int targetHash = hashIndex(targetBlockCoordinates);

	bool allocate = MarkAsNeedingAllocationIfNotFound(hashEntryStates, blockCoordinates,
	                                  targetHash, targetBlockCoordinates, targetHashTable,
	                                  *collisionDetected);
//_DEBUG EXPAND
//	printf("Allocating block: %d %d %d, hash: %d, collision: %s, allocate %s\n",
//			targetBlockCoordinates.x, targetBlockCoordinates.y, targetBlockCoordinates.z, targetHash, *collisionDetected ? "true" : "false",
//			allocate ? "true": "false");
}

struct SingleHashAllocationData {
	int lastFreeVoxelBlockId;
	int lastFreeExcessListId;
	int hashCode;
	bool success;
};

__global__ void allocateHashEntry_device(SingleHashAllocationData* data,
                                         const Vector3s at, ITMHashEntry* hashTable,
                                         const int* voxelAllocationList, const int* excessAllocationList) {
	ITMHashEntry* entry;
	data->success = FindOrAllocateHashEntry(at, hashTable, entry, data->lastFreeVoxelBlockId,
	                                        data->lastFreeExcessListId, voxelAllocationList, excessAllocationList,
	                                        data->hashCode);
};

} // end anonymous namespace (CUDA kernels)