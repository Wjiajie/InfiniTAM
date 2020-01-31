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

#include "IndexingEngine_CPU_VoxelBlockHash.h"
#include "../../../../Objects/Volume/RepresentationAccess.h"
#include "../../../EditAndCopy/Shared/EditAndCopyEngine_Shared.h"
#include "../../Shared/IndexingEngine_Shared.h"
#include "../../../Common/CheckBlockVisibility.h"
#include "../../../../Utils/Configuration.h"

using namespace ITMLib;

template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::AllocateFromDepth(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
		const Matrix4f& depth_camera_matrix, bool onlyUpdateVisibleList, bool resetVisibleList) {

	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = volume->sceneParams->voxel_size;

	Matrix4f inverted_depth_camera_matrix;
	Vector4f depthCameraProjectionParameters, invertedDepthCameraProjectionParameters;

	if (resetVisibleList) volume->index.SetUtilizedHashBlockCount(0);
	depth_camera_matrix.inv(inverted_depth_camera_matrix);

	depthCameraProjectionParameters = view->calib.intrinsics_d.projectionParamsSimple.all;
	invertedDepthCameraProjectionParameters = depthCameraProjectionParameters;
	invertedDepthCameraProjectionParameters.x = 1.0f / invertedDepthCameraProjectionParameters.x;
	invertedDepthCameraProjectionParameters.y = 1.0f / invertedDepthCameraProjectionParameters.y;

	float mu = volume->sceneParams->narrow_band_half_width;

	float* depth = view->depth->GetData(MEMORYDEVICE_CPU);
	int* voxelAllocationList = volume->localVBA.GetAllocationList();
	ITMHashEntry* hashTable = volume->index.GetEntries();
	HashBlockVisibility* hashBlockVisibilityTypes = volume->index.GetBlockVisibilityTypes();
	int hashEntryCount = volume->index.hashEntryCount;
	int* visibleEntryHashCodes = volume->index.GetUtilizedBlockHashCodes();

	HashEntryAllocationState* hashEntryStates_device = volume->index.GetHashEntryAllocationStates();
	Vector3s* allocationBlockCoordinates = volume->index.GetAllocationBlockCoordinates();

	float oneOverHashBlockSize = 1.0f / (voxelSize * VOXEL_BLOCK_SIZE);//m
	float band_factor = configuration::get().general_voxel_volume_parameters.block_allocation_band_factor;
	float surface_cutoff_distance = band_factor * mu;
	bool collisionDetected;

	bool useSwapping = volume->globalCache != nullptr;

	//reset visibility of formerly "visible" entries, if any
	SetVisibilityToVisibleAtPreviousFrameAndUnstreamed(hashBlockVisibilityTypes, visibleEntryHashCodes,
	                                                   volume->index.GetUtilizedHashBlockCount());
	do {
		volume->index.ClearHashEntryAllocationStates();
		collisionDetected = false;
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(depthImgSize, collisionDetected, volume, surface_cutoff_distance, \
        hashTable, oneOverHashBlockSize, invertedDepthCameraProjectionParameters, inverted_depth_camera_matrix, \
        depth, allocationBlockCoordinates, hashBlockVisibilityTypes, hashEntryStates_device)
#endif
		for (int locId = 0; locId < depthImgSize.x * depthImgSize.y; locId++) {
			int y = locId / depthImgSize.x;
			int x = locId - y * depthImgSize.x;

			buildHashAllocAndVisibleTypePP(hashEntryStates_device, hashBlockVisibilityTypes, x, y,
			                               allocationBlockCoordinates, depth, inverted_depth_camera_matrix,
			                               invertedDepthCameraProjectionParameters, surface_cutoff_distance,
			                               depthImgSize,
			                               oneOverHashBlockSize,
			                               hashTable, volume->sceneParams->near_clipping_distance,
			                               volume->sceneParams->far_clipping_distance, collisionDetected);
		}

		if (onlyUpdateVisibleList) {
			useSwapping = false;
			collisionDetected = false;
		} else {
			AllocateHashEntriesUsingLists_SetVisibility(volume);
		}
	} while (collisionDetected);

	BuildVisibilityList(volume, view, depth_camera_matrix);

	int lastFreeVoxelBlockId = volume->localVBA.lastFreeBlockId;

	//reallocate deleted hash blocks from previous swap operation
	if (useSwapping) {
		for (int hashCode = 0; hashCode < hashEntryCount; hashCode++) {
			if (hashBlockVisibilityTypes[hashCode] > 0 && hashTable[hashCode].ptr == -1) {
				if (lastFreeVoxelBlockId >= 0) {
					hashTable[lastFreeVoxelBlockId].ptr = voxelAllocationList[lastFreeVoxelBlockId];
					lastFreeVoxelBlockId--;
				}
			}
		}
	}
	volume->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
}

template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::AllocateFromDepth(
		VoxelVolume<TVoxel, VoxelBlockHash>* scene, const ITMView* view, const ITMTrackingState* trackingState,
		bool onlyUpdateVisibleList, bool resetVisibleList) {
	AllocateFromDepth(scene, view, trackingState->pose_d->GetM(), onlyUpdateVisibleList, resetVisibleList);
}

template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::AllocateFromDepthAndSdfSpan(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume,
		const RenderState* sourceRenderState,
		const ITMView* view,
		const Matrix4f& depth_camera_matrix,
		bool onlyUpdateAllocatedList, bool resetAllocatedList){

	DIEWITHEXCEPTION_REPORTLOCATION("NOT IMPLEMENTED");

	//TODO
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = volume->sceneParams->voxel_size;

	Matrix4f inverted_depth_camera_matrix;
	Vector4f depthCameraProjectionParameters, invertedDepthCameraProjectionParameters;

	if (resetAllocatedList) volume->index.SetUtilizedHashBlockCount(0);
	depth_camera_matrix.inv(inverted_depth_camera_matrix);

	depthCameraProjectionParameters = view->calib.intrinsics_d.projectionParamsSimple.all;
	invertedDepthCameraProjectionParameters = depthCameraProjectionParameters;
	invertedDepthCameraProjectionParameters.x = 1.0f / invertedDepthCameraProjectionParameters.x;
	invertedDepthCameraProjectionParameters.y = 1.0f / invertedDepthCameraProjectionParameters.y;

	float mu = volume->sceneParams->narrow_band_half_width;

	float* depth = view->depth->GetData(MEMORYDEVICE_CPU);
	int* voxelAllocationList = volume->localVBA.GetAllocationList();
	ITMHashEntry* hashTable = volume->index.GetEntries();
	HashBlockVisibility* hashBlockVisibilityTypes = volume->index.GetBlockVisibilityTypes();
	int hashEntryCount = volume->index.hashEntryCount;
	int* visibleEntryHashCodes = volume->index.GetUtilizedBlockHashCodes();

	HashEntryAllocationState* hashEntryStates_device = volume->index.GetHashEntryAllocationStates();
	Vector3s* allocationBlockCoordinates = volume->index.GetAllocationBlockCoordinates();

	float oneOverHashBlockSize = 1.0f / (voxelSize * VOXEL_BLOCK_SIZE);//m
	float band_factor = configuration::get().general_voxel_volume_parameters.block_allocation_band_factor;
	float surface_cutoff_distance = band_factor * mu;
	bool collisionDetected;

	bool useSwapping = volume->globalCache != nullptr;

	//reset visibility of formerly "visible" entries, if any
	SetVisibilityToVisibleAtPreviousFrameAndUnstreamed(hashBlockVisibilityTypes, visibleEntryHashCodes,
	                                                   volume->index.GetUtilizedHashBlockCount());
	do {
		volume->index.ClearHashEntryAllocationStates();
		collisionDetected = false;
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(depthImgSize, collisionDetected, volume, surface_cutoff_distance, \
        hashTable, oneOverHashBlockSize, invertedDepthCameraProjectionParameters, inverted_depth_camera_matrix, \
        depth, allocationBlockCoordinates, hashBlockVisibilityTypes, hashEntryStates_device)
#endif
		for (int locId = 0; locId < depthImgSize.x * depthImgSize.y; locId++) {
			int y = locId / depthImgSize.x;
			int x = locId - y * depthImgSize.x;

			buildHashAllocAndVisibleTypePP(hashEntryStates_device, hashBlockVisibilityTypes, x, y,
			                               allocationBlockCoordinates, depth, inverted_depth_camera_matrix,
			                               invertedDepthCameraProjectionParameters, surface_cutoff_distance,
			                               depthImgSize,
			                               oneOverHashBlockSize,
			                               hashTable, volume->sceneParams->near_clipping_distance,
			                               volume->sceneParams->far_clipping_distance, collisionDetected);
		}

		if (onlyUpdateAllocatedList) {
			useSwapping = false;
			collisionDetected = false;
		} else {
			AllocateHashEntriesUsingLists_SetVisibility(volume);
		}
	} while (collisionDetected);

	BuildVisibilityList(volume, view, depth_camera_matrix);

	int lastFreeVoxelBlockId = volume->localVBA.lastFreeBlockId;

	//reallocate deleted hash blocks from previous swap operation
	if (useSwapping) {
		for (int hashCode = 0; hashCode < hashEntryCount; hashCode++) {
			if (hashBlockVisibilityTypes[hashCode] > 0 && hashTable[hashCode].ptr == -1) {
				if (lastFreeVoxelBlockId >= 0) {
					hashTable[lastFreeVoxelBlockId].ptr = voxelAllocationList[lastFreeVoxelBlockId];
					lastFreeVoxelBlockId--;
				}
			}
		}
	}
	volume->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
}

template<typename TVoxel>
template<typename TVoxelTarget, typename TVoxelSource>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::AllocateUsingOtherVolume(
		VoxelVolume<TVoxelTarget, VoxelBlockHash>* targetVolume,
		VoxelVolume<TVoxelSource, VoxelBlockHash>* sourceVolume) {

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
#pragma omp parallel for default(none) shared(sourceHashEntries, hashEntryStates_device, blockCoordinates_device, \
        targetHashEntries, collisionDetected)
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

		IndexingEngine<TVoxelTarget, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
				.AllocateHashEntriesUsingLists(targetVolume);
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

template<typename TVoxelTarget, typename TVoxelSource, typename THashBlockMarkProcedure, typename TAllocationProcedure>
void AllocateUsingOtherVolumeExpanded_Generic(VoxelVolume<TVoxelTarget, VoxelBlockHash>* targetVolume,
                                              VoxelVolume<TVoxelSource, VoxelBlockHash>* sourceVolume,
                                              THashBlockMarkProcedure&& hashBlockMarkProcedure,
                                              TAllocationProcedure&& allocationProcedure) {
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
#pragma omp parallel for default(shared)
#endif
		for (int hashCode = 0; hashCode < hashEntryCount; hashCode++) {
			const ITMHashEntry& hashEntry = sourceHashTable[hashCode];
			// skip empty blocks
			if (hashEntry.ptr < 0) continue;
			for (auto& neighborOffset : neighborOffsets) {
				Vector3s neighborBlockCoordinates = hashEntry.pos + neighborOffset;
				// try to find a corresponding canonical block, and mark it for allocation if not found
				int targetHash = HashCodeFromBlockPosition(neighborBlockCoordinates);
				std::forward<THashBlockMarkProcedure>(hashBlockMarkProcedure)(neighborBlockCoordinates, targetHash,
				                                                              collision_detected);
			}
		}
		std::forward<TAllocationProcedure>(allocationProcedure)();
	} while (collision_detected);
}

template<typename TVoxel>
template<typename TVoxelTarget, typename TVoxelSource>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeExpanded(
		VoxelVolume<TVoxelTarget, VoxelBlockHash>* targetVolume,
		VoxelVolume<TVoxelSource, VoxelBlockHash>* sourceVolume) {

	ITMHashEntry* targetHashTable = targetVolume->index.GetEntries();

	HashEntryAllocationState* hashEntryStates_device = targetVolume->index.GetHashEntryAllocationStates();
	Vector3s* blockCoordinates_device = targetVolume->index.GetAllocationBlockCoordinates();

	auto hashBlockMarkProcedure = [&](Vector3s neighborBlockCoordinates, int& targetHash, bool& collision_detected) {
		MarkAsNeedingAllocationIfNotFound(hashEntryStates_device, blockCoordinates_device,
		                                  targetHash, neighborBlockCoordinates, targetHashTable,
		                                  collision_detected);
	};
	auto allocationProcedure = [&]() {
		IndexingEngine<TVoxelTarget, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
				.AllocateHashEntriesUsingLists(targetVolume);
	};
	AllocateUsingOtherVolumeExpanded_Generic(targetVolume, sourceVolume, hashBlockMarkProcedure, allocationProcedure);
}


template<typename TVoxel>
template<typename TVoxelTarget, typename TVoxelSource>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeAndSetVisibilityExpanded(
		VoxelVolume<TVoxelTarget, VoxelBlockHash>* targetVolume,
		VoxelVolume<TVoxelSource, VoxelBlockHash>* sourceVolume,
		ITMView* view, const Matrix4f& depth_camera_matrix) {

	ITMHashEntry* targetHashTable = targetVolume->index.GetEntries();
	int* visibleEntryHashCodes = targetVolume->index.GetUtilizedBlockHashCodes();
	HashBlockVisibility* hashBlockVisibilityTypes_device = targetVolume->index.GetBlockVisibilityTypes();

	HashEntryAllocationState* hashEntryStates_device = targetVolume->index.GetHashEntryAllocationStates();
	Vector3s* blockCoordinates_device = targetVolume->index.GetAllocationBlockCoordinates();

	SetVisibilityToVisibleAtPreviousFrameAndUnstreamed(hashBlockVisibilityTypes_device, visibleEntryHashCodes,
	                                                   targetVolume->index.GetUtilizedHashBlockCount());

	auto hashBlockMarkProcedure = [&](Vector3s neighborBlockCoordinates, int& targetHash, bool& collision_detected) {
		MarkForAllocationAndSetVisibilityTypeIfNotFound(
				hashEntryStates_device, blockCoordinates_device, hashBlockVisibilityTypes_device,
				neighborBlockCoordinates, targetHashTable, collision_detected);
	};
	auto allocationProcedure = [&]() {
		IndexingEngine<TVoxelTarget, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
				.AllocateHashEntriesUsingLists_SetVisibility(targetVolume);
	};
	AllocateUsingOtherVolumeExpanded_Generic(targetVolume, sourceVolume, hashBlockMarkProcedure, allocationProcedure);

	IndexingEngine<TVoxelTarget, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance().BuildVisibilityList(targetVolume,
	                                                                                                  view,
	                                                                                                  depth_camera_matrix);
}

// #define EXCEPT_ON_OUT_OF_SPACE
template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::
AllocateHashEntriesUsingLists(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {

	const HashEntryAllocationState* hashEntryStates_device = volume->index.GetHashEntryAllocationStates();
	Vector3s* blockCoordinates_device = volume->index.GetAllocationBlockCoordinates();

	const int hashEntryCount = volume->index.hashEntryCount;
	int lastFreeVoxelBlockId = volume->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = volume->index.GetLastFreeExcessListId();
	int* voxelAllocationList = volume->localVBA.GetAllocationList();
	int* excessAllocationList = volume->index.GetExcessAllocationList();
	ITMHashEntry* hashTable = volume->index.GetEntries();

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
#ifdef EXCEPT_ON_OUT_OF_SPACE
			else {
				DIEWITHEXCEPTION_REPORTLOCATION("Not enough space in ordered list.");
			}
#endif

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
#ifdef EXCEPT_ON_OUT_OF_SPACE
			else {
				DIEWITHEXCEPTION_REPORTLOCATION("Not enough space in excess list.");
			}
#endif
				break;
			default:
				break;
		}
	}
	volume->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	volume->index.SetLastFreeExcessListId(lastFreeExcessListId);
}


template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::
AllocateHashEntriesUsingLists_SetVisibility(VoxelVolume<TVoxel, VoxelBlockHash>* volume) {

	const HashEntryAllocationState* hashEntryAllocationStates_device = volume->index.GetHashEntryAllocationStates();
	Vector3s* allocationBlockCoordinates_device = volume->index.GetAllocationBlockCoordinates();
	HashBlockVisibility* hashBlockVisibilityTypes_device = volume->index.GetBlockVisibilityTypes();

	int entryCount = volume->index.hashEntryCount;
	int lastFreeVoxelBlockId = volume->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = volume->index.GetLastFreeExcessListId();
	int* voxelAllocationList = volume->localVBA.GetAllocationList();
	int* excessAllocationList = volume->index.GetExcessAllocationList();
	ITMHashEntry* hashTable = volume->index.GetEntries();

	for (int hash = 0; hash < entryCount; hash++) {
		const HashEntryAllocationState& hashEntryState = hashEntryAllocationStates_device[hash];
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
	volume->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	volume->index.SetLastFreeExcessListId(lastFreeExcessListId);
}


template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::BuildVisibilityList(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view, const Matrix4f& depth_camera_matrix) {

	// ** scene data **
	const int hashEntryCount = volume->index.hashEntryCount;
	HashBlockVisibility* hashBlockVisibilityTypes = volume->index.GetBlockVisibilityTypes();
	int* visibleEntryHashCodes = volume->index.GetUtilizedBlockHashCodes();
	ITMHashEntry* hashTable = volume->index.GetEntries();
	bool useSwapping = volume->globalCache != nullptr;
	ITMHashSwapState* swapStates = volume->Swapping() ? volume->globalCache->GetSwapStates(false) : 0;

	// ** view data **
	Vector4f depthCameraProjectionParameters = view->calib.intrinsics_d.projectionParamsSimple.all;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = volume->sceneParams->voxel_size;

	int visibleEntryCount = 0;
	//build visible list
	for (int hashCode = 0; hashCode < hashEntryCount; hashCode++) {
		HashBlockVisibility hashVisibleType = hashBlockVisibilityTypes[hashCode];
		const ITMHashEntry& hashEntry = hashTable[hashCode];

		if (hashVisibleType == 3) {
			bool isVisibleEnlarged, isVisible;

			if (useSwapping) {
				checkBlockVisibility<true>(isVisible, isVisibleEnlarged, hashEntry.pos, depth_camera_matrix,
				                           depthCameraProjectionParameters,
				                           voxelSize, depthImgSize);
				if (!isVisibleEnlarged) hashVisibleType = INVISIBLE;
			} else {
				checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, depth_camera_matrix,
				                            depthCameraProjectionParameters,
				                            voxelSize, depthImgSize);
				if (!isVisible) { hashVisibleType = INVISIBLE; }
			}
			hashBlockVisibilityTypes[hashCode] = hashVisibleType;
		}

		if (useSwapping) {
			if (hashVisibleType > 0 && swapStates[hashCode].state != 2) swapStates[hashCode].state = 1;
		}

		if (hashVisibleType > 0) {
			visibleEntryHashCodes[visibleEntryCount] = hashCode;
			visibleEntryCount++;
		}
	}
	volume->index.SetUtilizedHashBlockCount(visibleEntryCount);
}

template<typename TVoxel>
void IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::SetVisibilityToVisibleAtPreviousFrameAndUnstreamed(
		HashBlockVisibility* hashBlockVisibilityTypes, const int* visibleBlockHashCodes, int visibleHashBlockCount) {
	for (int i_visible_block = 0; i_visible_block < visibleHashBlockCount; i_visible_block++) {
		hashBlockVisibilityTypes[visibleBlockHashCodes[i_visible_block]] = HashBlockVisibility::VISIBLE_AT_PREVIOUS_FRAME_AND_UNSTREAMED;
	}
}

template<typename TVoxel>
ITMHashEntry
IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::FindHashEntry(const VoxelBlockHash& index,
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
IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::FindHashEntry(const VoxelBlockHash& index,
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
bool IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::AllocateHashBlockAt(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3s at, int& hashCode) {

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