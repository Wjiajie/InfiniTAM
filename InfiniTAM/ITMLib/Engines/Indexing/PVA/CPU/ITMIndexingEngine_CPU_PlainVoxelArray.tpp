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

#include "ITMIndexingEngine_CPU_PlainVoxelArray.h"
#include "../../../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../../Manipulation/Shared/ITMSceneManipulationEngine_Shared.h"
#include "../../../../Objects/RenderStates/ITMRenderState_VH.h"
#include "../../../Reconstruction/Shared/ITMSceneReconstructionEngine_Shared.h"

using namespace ITMLib;


template<typename TVoxel>
void ITMIndexingEngine<TVoxel, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::AllocateFromDepth(
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, const ITMView* view, const ITMTrackingState* trackingState,
		const ITMRenderState* renderState, bool onlyUpdateVisibleList, bool resetVisibleList) {

}

template<typename TVoxel>
template<typename TVoxelATarget, typename TVoxelASource>
void ITMIndexingEngine<TVoxel, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::AllocateUsingOtherVolume(
		ITMVoxelVolume<TVoxelATarget, ITMPlainVoxelArray>* targetVolume,
		ITMVoxelVolume<TVoxelASource, ITMPlainVoxelArray>* sourceVolume) {

	assert(targetVolume->index.hashEntryCount == sourceVolume->index.hashEntryCount);

	const int hashEntryCount = targetVolume->index.hashEntryCount;

	HashEntryState* hashEntryStates_device = targetVolume->index.GetHashEntryStates();
	Vector3s* blockCoordinates_device = targetVolume->index.GetAllocationBlockCoordinates();
	ITMHashEntry* targetHashEntries = targetVolume->index.GetEntries();
	ITMHashEntry* sourceHashEntries = sourceVolume->index.GetEntries();

	bool collisionDetected;

	do {
		collisionDetected = false;
		//reset target allocation states
		targetVolume->index.ClearHashEntryStates();
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
			int targetHash = hashIndex(sourceHashBlockCoords);

			MarkAsNeedingAllocationIfNotFound(hashEntryStates_device, blockCoordinates_device,
			                                  targetHash, sourceHashBlockCoords, targetHashEntries,
			                                  collisionDetected);
		}

		ITMIndexingEngine<TVoxelATarget, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::Instance()
				.AllocateHashEntriesUsingLists(targetVolume, hashEntryStates_device,
				                               blockCoordinates_device);
	} while (collisionDetected);

}


template<typename TVoxel>
void ITMIndexingEngine<TVoxel, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::
AllocateHashEntriesUsingLists(
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene,
		const HashEntryState* hashEntryStates_device,
		Vector3s* blockCoordinates_device) {
	const int hashEntryCount = scene->index.hashEntryCount;
	int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = scene->index.GetLastFreeExcessListId();
	int* voxelAllocationList = scene->localVBA.GetAllocationList();
	int* excessAllocationList = scene->index.GetExcessAllocationList();
	ITMHashEntry* hashTable = scene->index.GetEntries();

	for (int hashCode = 0; hashCode < hashEntryCount; hashCode++) {
		const HashEntryState& hashEntryState = hashEntryStates_device[hashCode];
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
void ITMIndexingEngine<TVoxelA, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::
AllocateHashEntriesUsingLists_SetVisibility(ITMVoxelVolume<TVoxelA, ITMPlainVoxelArray>* scene,
                                            const ITMLib::HashEntryState* hashEntryStates_device,
                                            Vector3s* allocationBlockCoordinates_device,
                                            uchar* hashBlockVisibilityTypes_device) {
	int entryCount = scene->index.hashEntryCount;
	int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = scene->index.GetLastFreeExcessListId();
	int* voxelAllocationList = scene->localVBA.GetAllocationList();
	int* excessAllocationList = scene->index.GetExcessAllocationList();
	ITMHashEntry* hashTable = scene->index.GetEntries();

	for (int hash = 0; hash < entryCount; hash++) {
		const HashEntryState& hashEntryState = hashEntryStates_device[hash];
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
					hashBlockVisibilityTypes_device[hash] = 0;
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
					                                exlOffset] = 1; //make child visible and in memory
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
ITMHashEntry ITMIndexingEngine<TVoxel, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::FindHashEntry(const ITMPlainVoxelArray& index,
                                                                                            const Vector3s& coordinates) {
	const ITMHashEntry* entries = index.GetEntries();
	int hashCode = FindHashCodeAt(entries, coordinates);
	if (hashCode == -1) {
		return {Vector3s(0, 0, 0), 0, -2};
	} else {
		return entries[hashCode];
	}
}