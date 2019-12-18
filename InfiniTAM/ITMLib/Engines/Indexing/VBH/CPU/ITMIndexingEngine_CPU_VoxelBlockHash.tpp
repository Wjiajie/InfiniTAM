//  ================================================================
//  Created by Gregory Kramida on 11/1/19.
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

#include "ITMIndexingEngine_CPU_VoxelBlockHash.h"
#include "../../../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../../Manipulation/Shared/ITMSceneManipulationEngine_Shared.h"
#include "../../Shared/ITMIndexingEngine_Shared.h"
#include "../CUDA/ITMIndexingEngine_CUDA_VoxelBlockHash.h"
#include "../../Interface/ITMIndexingEngine.h"


using namespace ITMLib;


template<typename TVoxel>
void
ITMIndexingEngine<TVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::AllocateFromDepth(
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, const ITMView* view, const ITMTrackingState* trackingState,
		bool onlyUpdateVisibleList, bool resetVisibleList) {
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f cameraPose, invertedCameraPose;
	Vector4f projParams_d, invertedCameraProjectionParameters;

	if (resetVisibleList) scene->index.SetVisibleHashBlockCount(0);

	cameraPose = trackingState->pose_d->GetM();
	cameraPose.inv(invertedCameraPose);

	projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all;
	invertedCameraProjectionParameters = projParams_d;
	invertedCameraProjectionParameters.x = 1.0f / invertedCameraProjectionParameters.x;
	invertedCameraProjectionParameters.y = 1.0f / invertedCameraProjectionParameters.y;

	float mu = scene->sceneParams->mu;

	float* depth = view->depth->GetData(MEMORYDEVICE_CPU);
	int* voxelAllocationList = scene->localVBA.GetAllocationList();
	ITMHashEntry* hashTable = scene->index.GetEntries();
	ITMHashSwapState* swapStates = scene->Swapping() ? scene->globalCache->GetSwapStates(false) : 0;
	int* visibleEntryHashCodes = scene->index.GetVisibleBlockHashCodes();
	HashBlockVisibility* hashBlockVisibilityTypes = scene->index.GetBlockVisibilityTypes();
	int hashEntryCount = scene->index.hashEntryCount;

	HashEntryAllocationState* hashEntryStates_device = scene->index.GetHashEntryAllocationStates();
	Vector3s* allocationBlockCoordinates = scene->index.GetAllocationBlockCoordinates();

	float oneOverHashEntrySize = 1.0f / (voxelSize * VOXEL_BLOCK_SIZE);//m

	bool collisionDetected;

	bool useSwapping = scene->globalCache != nullptr;
	do {
		scene->index.ClearHashEntryAllocationStates();
		collisionDetected = false;
		for (int i = 0; i < scene->index.GetVisibleHashBlockCount(); i++)
			hashBlockVisibilityTypes[visibleEntryHashCodes[i]] = HashBlockVisibility::VISIBLE_AT_PREVIOUS_FRAME_AND_UNSTREAMED;
#ifdef WITH_OPENMP
#pragma omp parallel for default(none)
#endif
		for (int locId = 0; locId < depthImgSize.x * depthImgSize.y; locId++) {
			int y = locId / depthImgSize.x;
			int x = locId - y * depthImgSize.x;

			buildHashAllocAndVisibleTypePP(hashEntryStates_device, hashBlockVisibilityTypes, x, y,
			                               allocationBlockCoordinates, depth, invertedCameraPose,
			                               invertedCameraProjectionParameters, mu, depthImgSize,
			                               oneOverHashEntrySize,
			                               hashTable, scene->sceneParams->viewFrustum_min,
			                               scene->sceneParams->viewFrustum_max, collisionDetected);
		}

		if (onlyUpdateVisibleList) {
			useSwapping = false;
			collisionDetected = false;
		} else {
			AllocateHashEntriesUsingLists_SetVisibility(scene, hashEntryStates_device,
			                                            allocationBlockCoordinates, hashBlockVisibilityTypes);
		}
	} while (collisionDetected);

	int visibleEntryCount = 0;
	//build visible list
	for (int targetIdx = 0; targetIdx < hashEntryCount; targetIdx++) {
		HashBlockVisibility hashVisibleType = hashBlockVisibilityTypes[targetIdx];
		const ITMHashEntry& hashEntry = hashTable[targetIdx];

		if (hashVisibleType == 3) {
			bool isVisibleEnlarged, isVisible;

			if (useSwapping) {
				checkBlockVisibility<true>(isVisible, isVisibleEnlarged, hashEntry.pos, cameraPose, projParams_d,
				                           voxelSize, depthImgSize);
				if (!isVisibleEnlarged) hashVisibleType = INVISIBLE;
			} else {
				checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, cameraPose, projParams_d,
				                            voxelSize, depthImgSize);
				if (!isVisible) { hashVisibleType = INVISIBLE; }
			}
			hashBlockVisibilityTypes[targetIdx] = hashVisibleType;
		}

		if (useSwapping) {
			if (hashVisibleType > 0 && swapStates[targetIdx].state != 2) swapStates[targetIdx].state = 1;
		}

		if (hashVisibleType > 0) {
			visibleEntryHashCodes[visibleEntryCount] = targetIdx;
			visibleEntryCount++;
		}
	}

	int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;

	//reallocate deleted hash blocks from previous swap operation
	if (useSwapping) {
		for (int targetIdx = 0; targetIdx < hashEntryCount; targetIdx++) {
			if (hashBlockVisibilityTypes[targetIdx] > 0 && hashTable[targetIdx].ptr == -1) {
				if (lastFreeVoxelBlockId >= 0) {
					hashTable[lastFreeVoxelBlockId].ptr = voxelAllocationList[lastFreeVoxelBlockId];
					lastFreeVoxelBlockId--;
				}
			}
		}
	}
	scene->index.SetVisibleHashBlockCount(visibleEntryCount);
	scene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
}

template<typename TVoxel>
template<typename TVoxelTarget, typename TVoxelSource>
void ITMIndexingEngine<TVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::AllocateUsingOtherVolume(
		ITMVoxelVolume<TVoxelTarget, ITMVoxelBlockHash>* targetVolume,
		ITMVoxelVolume<TVoxelSource, ITMVoxelBlockHash>* sourceVolume) {

	assert(targetVolume->index.hashEntryCount == sourceVolume->index.hashEntryCount);

	const int hashEntryCount = targetVolume->index.hashEntryCount;

	HashEntryAllocationState* hashEntryStates_device = targetVolume->index.GetHashEntryAllocationStates();
	Vector3s* blockCoordinates_device = targetVolume->index.GetAllocationBlockCoordinates();
	ITMHashEntry* targetHashEntries = targetVolume->index.GetEntries();
	ITMHashEntry* sourceHashEntries = sourceVolume->index.GetEntries();

	bool collisionDetected;

	do {
		collisionDetected = false;
		//reset target allocation states
		targetVolume->index.ClearHashEntryAllocationStates();
#ifdef WITH_OPENMP
#pragma omp parallel for default(none)
#endif
		for (int sourceHash = 0; sourceHash < hashEntryCount; sourceHash++) {

			const ITMHashEntry& currentSourceHashBlock = sourceHashEntries[sourceHash];
			//skip unfilled live blocks
			if (currentSourceHashBlock.ptr < 0) {
				continue;
			}
			Vector3s sourceHashBlockCoords = currentSourceHashBlock.pos;

			//try to find a corresponding canonical block, and mark it for allocation if not found
			int targetHash = HashCodeFromBlockPosition(sourceHashBlockCoords);

			MarkAsNeedingAllocationIfNotFound(hashEntryStates_device, blockCoordinates_device,
			                                  targetHash, sourceHashBlockCoords, targetHashEntries,
			                                  collisionDetected);
		}

		ITMIndexingEngine<TVoxelTarget, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
				.AllocateHashEntriesUsingLists(targetVolume, hashEntryStates_device, blockCoordinates_device);
	} while (collisionDetected);

}

static std::vector<Vector3s> neighborOffsets = [] { // NOLINT(cert-err58-cpp)
	std::vector<Vector3s> offsets;
	for (short zOffset = -1; zOffset < 2; zOffset++) {
		for (short yOffset = -1; yOffset < 2; yOffset++) {
			for (short xOffset = -1; xOffset < 2; xOffset++) {
				Vector3s neighborOffset(xOffset, yOffset, zOffset);
				offsets.push_back(neighborOffset);
			}
		}
	}
	return offsets;
}();

template<typename TVoxel>
template<typename TVoxelTarget, typename TVoxelSource>
void ITMIndexingEngine<TVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeExpanded(
		ITMVoxelVolume<TVoxelTarget, ITMVoxelBlockHash>* targetVolume,
		ITMVoxelVolume<TVoxelSource, ITMVoxelBlockHash>* sourceVolume) {

	assert(sourceVolume->index.hashEntryCount == targetVolume->index.hashEntryCount);

	int hashEntryCount = targetVolume->index.hashEntryCount;
	ITMHashEntry* targetHashTable = targetVolume->index.GetEntries();
	ITMHashEntry* sourceHashTable = sourceVolume->index.GetEntries();

	HashEntryAllocationState* hashEntryStates_device = targetVolume->index.GetHashEntryAllocationStates();
	Vector3s* blockCoordinates_device = targetVolume->index.GetAllocationBlockCoordinates();

	bool collision_detected;
	do {
		collision_detected = false;
		targetVolume->index.ClearHashEntryAllocationStates();
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(hashEntryCount, sourceHashTable, targetHashTable, hashEntryStates_device, blockCoordinates_device, collision_detected)
#endif
		for (int hashCode = 0; hashCode < hashEntryCount; hashCode++) {
			const ITMHashEntry& hashEntry = sourceHashTable[hashCode];
			// skip empty blocks
			if (hashEntry.ptr < 0) continue;
			for (auto& neighborOffset : neighborOffsets) {
				Vector3s neighborBlockCoordinates = hashEntry.pos + neighborOffset;
				// try to find a corresponding canonical block, and mark it for allocation if not found
				int targetHash = HashCodeFromBlockPosition(neighborBlockCoordinates);
				MarkAsNeedingAllocationIfNotFound(hashEntryStates_device, blockCoordinates_device,
				                                  targetHash, neighborBlockCoordinates, targetHashTable,
				                                  collision_detected);
			}
		}
		ITMIndexingEngine<TVoxelTarget, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
				.AllocateHashEntriesUsingLists(targetVolume, hashEntryStates_device, blockCoordinates_device);
	} while (collision_detected);
}

template<typename TVoxel>
void ITMIndexingEngine<TVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::
AllocateHashEntriesUsingLists(
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene,
		const HashEntryAllocationState* hashEntryStates_device,
		Vector3s* blockCoordinates_device) {
	const int hashEntryCount = scene->index.hashEntryCount;
	int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = scene->index.GetLastFreeExcessListId();
	int* voxelAllocationList = scene->localVBA.GetAllocationList();
	int* excessAllocationList = scene->index.GetExcessAllocationList();
	ITMHashEntry* hashTable = scene->index.GetEntries();

	for (int hashCode = 0; hashCode < hashEntryCount; hashCode++) {
		const HashEntryAllocationState& hashEntryState = hashEntryStates_device[hashCode];
		switch (hashEntryState) {
			case ITMLib::NEEDS_ALLOCATION_IN_ORDERED_LIST:

				if (lastFreeVoxelBlockId >= 0) //there is room in the voxel block array
				{
					ITMHashEntry hashEntry;
					hashEntry.pos = blockCoordinates_device[hashCode];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
					hashTable[hashCode] = hashEntry;
					lastFreeVoxelBlockId--;
				}

				break;
			case NEEDS_ALLOCATION_IN_EXCESS_LIST:

				if (lastFreeVoxelBlockId >= 0 &&
				    lastFreeExcessListId >= 0) //there is room in the voxel block array and excess list
				{
					ITMHashEntry hashEntry;
					hashEntry.pos = blockCoordinates_device[hashCode];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
					int exlOffset = excessAllocationList[lastFreeExcessListId];
					hashTable[hashCode].offset = exlOffset + 1; //connect to child
					hashTable[ORDERED_LIST_SIZE + exlOffset] = hashEntry; //add child to the excess list
					lastFreeVoxelBlockId--;
					lastFreeExcessListId--;
				}
				break;
			default:
				break;
		}
	}
	scene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	scene->index.SetLastFreeExcessListId(lastFreeExcessListId);
}


template<typename TVoxelA>
void ITMIndexingEngine<TVoxelA, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::
AllocateHashEntriesUsingLists_SetVisibility(ITMVoxelVolume<TVoxelA, ITMVoxelBlockHash>* scene,
                                            const ITMLib::HashEntryAllocationState* hashEntryStates_device,
                                            Vector3s* allocationBlockCoordinates_device,
                                            HashBlockVisibility* hashBlockVisibilityTypes_device) {
	int entryCount = scene->index.hashEntryCount;
	int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = scene->index.GetLastFreeExcessListId();
	int* voxelAllocationList = scene->localVBA.GetAllocationList();
	int* excessAllocationList = scene->index.GetExcessAllocationList();
	ITMHashEntry* hashTable = scene->index.GetEntries();

	for (int hash = 0; hash < entryCount; hash++) {
		const HashEntryAllocationState& hashEntryState = hashEntryStates_device[hash];
		switch (hashEntryState) {
			case ITMLib::NEEDS_ALLOCATION_IN_ORDERED_LIST:

				if (lastFreeVoxelBlockId >= 0) //there is room in the voxel block array
				{
					ITMHashEntry hashEntry;
					hashEntry.pos = allocationBlockCoordinates_device[hash];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
					hashTable[hash] = hashEntry;
					lastFreeVoxelBlockId--;
				} else {
					hashBlockVisibilityTypes_device[hash] = INVISIBLE;
				}

				break;
			case NEEDS_ALLOCATION_IN_EXCESS_LIST:

				if (lastFreeVoxelBlockId >= 0 &&
				    lastFreeExcessListId >= 0) //there is room in the voxel block array and excess list
				{
					ITMHashEntry hashEntry;
					hashEntry.pos = allocationBlockCoordinates_device[hash];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
					int exlOffset = excessAllocationList[lastFreeExcessListId];
					hashTable[hash].offset = exlOffset + 1; //connect to child
					hashTable[ORDERED_LIST_SIZE + exlOffset] = hashEntry; //add child to the excess list
					hashBlockVisibilityTypes_device[ORDERED_LIST_SIZE +
					                                exlOffset] = IN_MEMORY_AND_VISIBLE; //make child visible and in memory
					lastFreeVoxelBlockId--;
					lastFreeExcessListId--;
				}
				break;
			default:
				break;
		}
	}
	scene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	scene->index.SetLastFreeExcessListId(lastFreeExcessListId);
}

template<typename TVoxel>
ITMHashEntry
ITMIndexingEngine<TVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::FindHashEntry(const ITMVoxelBlockHash& index,
                                                                              const Vector3s& coordinates) {
	const ITMHashEntry* entries = index.GetEntries();
	int hashCode = FindHashCodeAt(entries, coordinates);
	if (hashCode == -1) {
		return {Vector3s(0, 0, 0), 0, -2};
	} else {
		return entries[hashCode];
	}
}

template<typename TVoxel>
ITMHashEntry
ITMIndexingEngine<TVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::FindHashEntry(const ITMVoxelBlockHash& index,
                                                                              const Vector3s& coordinates,
                                                                              int& hashCode) {
	const ITMHashEntry* entries = index.GetEntries();
	hashCode = FindHashCodeAt(entries, coordinates);
	if (hashCode == -1) {
		return {Vector3s(0, 0, 0), 0, -2};
	} else {
		return entries[hashCode];
	}
}

template<typename TVoxel>
bool ITMIndexingEngine<TVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::AllocateHashBlockAt(
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* volume, Vector3s at, int& hashCode) {

	ITMHashEntry* hashTable = volume->index.GetEntries();
	int lastFreeVoxelBlockId = volume->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = volume->index.GetLastFreeExcessListId();
	int* voxelAllocationList = volume->localVBA.GetAllocationList();
	int* excessAllocationList = volume->index.GetExcessAllocationList();
	ITMHashEntry* entry = nullptr;
	hashCode = -1;
	if (!FindOrAllocateHashEntry(at, hashTable, entry, lastFreeVoxelBlockId, lastFreeExcessListId,
	                             voxelAllocationList, excessAllocationList, hashCode)) {
		return false;
	}
	volume->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	volume->index.SetLastFreeExcessListId(lastFreeExcessListId);
	return true;
}