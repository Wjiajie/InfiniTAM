//  ================================================================
//  Created by Gregory Kramida on 5/25/18.
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
#include "ITMHashAllocationEngine_CPU.h"
#include "../../../Objects/RenderStates/ITMRenderState_VH.h"
#include "../../../Utils/ITMHashBlockProperties.h"
#include "../Shared/ITMHashAllocationEngine_Shared.h"
#include "../Shared/ITMSceneReconstructionEngine_Shared.h"
#include "../../Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../../Traversal/CPU/ITMSceneTraversal_CPU_VoxelBlockHash.h"
#include "../../Common/ITMCommonFunctors.h"
#include "../../Manipulation/Shared/ITMSceneManipulationEngine_Shared.h"


using namespace ITMLib;

template<typename TVoxel, typename TWarp>
void ITMHashAllocationEngine_CPU<TVoxel, TWarp>::AllocateFromDepth(
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, const ITMView* view, const ITMTrackingState* trackingState,
		const ITMRenderState* renderState, bool onlyUpdateVisibleList, bool resetVisibleList) {
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, invM_d;
	Vector4f projParams_d, invProjParams_d;

	ITMRenderState_VH* renderState_vh = (ITMRenderState_VH*) renderState;
	if (resetVisibleList) renderState_vh->noVisibleEntries = 0;

	M_d = trackingState->pose_d->GetM();
	M_d.inv(invM_d);

	projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all;
	invProjParams_d = projParams_d;
	invProjParams_d.x = 1.0f / invProjParams_d.x;
	invProjParams_d.y = 1.0f / invProjParams_d.y;

	float mu = scene->sceneParams->mu;

	float* depth = view->depth->GetData(MEMORYDEVICE_CPU);
	int* voxelAllocationList = scene->localVBA.GetAllocationList();
	ITMHashEntry* hashTable = scene->index.GetEntries();
	ITMHashSwapState* swapStates = scene->Swapping() ? scene->globalCache->GetSwapStates(false) : 0;
	int* visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
	uchar* hashBlockVisibilityTypes = renderState_vh->GetEntriesVisibleType();
	int hashEntryCount = scene->index.hashEntryCount;

	ORUtils::MemoryBlock<HashEntryState> hashEntryStates(hashEntryCount, MEMORYDEVICE_CPU);
	HashEntryState* hashEntryStates_device = hashEntryStates.GetData(MEMORYDEVICE_CPU);
	ORUtils::MemoryBlock<Vector3s> targetSceneHashBlockCoordinates(hashEntryCount, MEMORYDEVICE_CPU);
	Vector3s* allocationBlockCoordinates = targetSceneHashBlockCoordinates.GetData(MEMORYDEVICE_CPU);

	float oneOverHashEntrySize = 1.0f / (voxelSize * VOXEL_BLOCK_SIZE);//m

	bool collisionDetected;

	bool useSwapping = scene->globalCache != nullptr;
	do {
		memset(hashEntryStates_device, NEEDS_NO_CHANGE, static_cast<size_t>(hashEntryCount));
		collisionDetected = false;
		for (int i = 0; i < renderState_vh->noVisibleEntries; i++)
			hashBlockVisibilityTypes[visibleEntryIDs[i]] = 3; // visible at previous frame and unstreamed

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int locId = 0; locId < depthImgSize.x * depthImgSize.y; locId++) {
			int y = locId / depthImgSize.x;
			int x = locId - y * depthImgSize.x;

			buildHashAllocAndVisibleTypePP(hashEntryStates_device, hashBlockVisibilityTypes, x, y,
			                               allocationBlockCoordinates, depth, invM_d, invProjParams_d, mu, depthImgSize,
			                               oneOverHashEntrySize,
			                               hashTable, scene->sceneParams->viewFrustum_min,
			                               scene->sceneParams->viewFrustum_max, collisionDetected);
		}

		if (onlyUpdateVisibleList) {
			useSwapping = false;
			collisionDetected = false;
		} else {
			AllocateHashEntriesUsingLists_SetVisibility_CPU(scene, hashEntryStates_device,
			                                                allocationBlockCoordinates, hashBlockVisibilityTypes);
		}
	} while (collisionDetected);

	int noVisibleEntries = 0;
	//build visible list
	for (int targetIdx = 0; targetIdx < hashEntryCount; targetIdx++) {
		unsigned char hashVisibleType = hashBlockVisibilityTypes[targetIdx];
		const ITMHashEntry& hashEntry = hashTable[targetIdx];

		if (hashVisibleType == 3) {
			bool isVisibleEnlarged, isVisible;

			if (useSwapping) {
				checkBlockVisibility<true>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d,
				                           voxelSize, depthImgSize);
				if (!isVisibleEnlarged) hashVisibleType = 0;
			} else {
				checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d,
				                            voxelSize, depthImgSize);
				if (!isVisible) { hashVisibleType = 0; }
			}
			hashBlockVisibilityTypes[targetIdx] = hashVisibleType;
		}

		if (useSwapping) {
			if (hashVisibleType > 0 && swapStates[targetIdx].state != 2) swapStates[targetIdx].state = 1;
		}

		if (hashVisibleType > 0) {
			visibleEntryIDs[noVisibleEntries] = targetIdx;
			noVisibleEntries++;
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
	renderState_vh->noVisibleEntries = noVisibleEntries;
	scene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
}


// region =========================== CANONICAL HASH BLOCK ALLOCATION ==================================================

template<typename TVoxel, typename TWarp>
void ITMHashAllocationEngine_CPU<TVoxel, TWarp>::AllocateTSDFVolumeFromTSDFVolume(
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* targetVolume,
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* sourceVolume) {
	AllocateFromVolumeGeneric(targetVolume, sourceVolume);
}


template<typename TVoxel, typename TWarp>
void ITMHashAllocationEngine_CPU<TVoxel, TWarp>::AllocateWarpVolumeFromTSDFVolume(
		ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* targetVolume,
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* sourceVolume) {
	AllocateFromVolumeGeneric(targetVolume, sourceVolume);
}

template<typename TVoxel, typename TWarp>
template<typename TVoxelTarget, typename TVoxelSource>
void ITMHashAllocationEngine_CPU<TVoxel, TWarp>::AllocateFromVolumeGeneric(
		ITMVoxelVolume<TVoxelTarget, ITMVoxelBlockHash>* targetVolume,
		ITMVoxelVolume<TVoxelSource, ITMVoxelBlockHash>* sourceVolume) {

	assert(targetVolume->index.hashEntryCount == sourceVolume->index.hashEntryCount);

	const int hashEntryCount = targetVolume->index.hashEntryCount;
	ORUtils::MemoryBlock<HashEntryState> hashEntryStates(hashEntryCount, MEMORYDEVICE_CPU);
	HashEntryState* hashEntryStates_device = hashEntryStates.GetData(MEMORYDEVICE_CPU);
	ORUtils::MemoryBlock<Vector3s> hashBlockCoordinates(hashEntryCount, MEMORYDEVICE_CPU);
	Vector3s* blockCoordinates_device = hashBlockCoordinates.GetData(MEMORYDEVICE_CPU);
	ITMHashEntry* targetHashEntries = targetVolume->index.GetEntries();
	ITMHashEntry* sourceHashEntries = sourceVolume->index.GetEntries();

	bool collisionDetected;

	do {
		collisionDetected = false;
		//reset target allocation states
		memset(hashEntryStates_device, ITMLib::NEEDS_NO_CHANGE,
		       static_cast<size_t>(hashEntryCount));
#ifdef WITH_OPENMP
#pragma omp parallel for
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
		AllocateHashEntriesUsingLists_CPU(targetVolume, hashEntryStates_device,
		                                  blockCoordinates_device);
	} while (collisionDetected);

}

// region ===================================== ALLOCATIONS FOR INTERPOLATION ==========================================

/**
 * \brief method which looks at voxel grid with warps and an SDF voxel grid and allocates all hash blocks in the
 * SDF grid where warp vectors are pointing to (if not already allocated).
 * \details scans each (allocated) voxel in the SDF voxel grid, checks the warp vector at the corresponding location,
 * finds the voxel where the warp vector is pointing to, and, if the hash block for that voxel is not yet allocated,
 * allocates it.
 * \param warpField voxel grid where each voxel has a .warp Vector3f field defined
 * \param sourceTsdf sdf grid whose hash blocks to allocate if needed
 * \param sourceSdfIndex index of the sdf / flag field to use in the sdfScene
 * \tparam TWarpType the type of warp vector to use
 */
template<typename TVoxel, typename TWarp>
template<WarpType TWarpType>
void ITMHashAllocationEngine_CPU<TVoxel, TWarp>::AllocateFromWarpedVolume(
		ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField,
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* sourceTSDF,
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* targetTSDF) {
	allocateFromWarpedVolume_common<TVoxel,TWarp, TWarpType, ITMLibSettings::DEVICE_CPU>(warpField,sourceTSDF,targetTSDF);
}


// endregion ==================================== END CANONICAL HASH BLOCK ALLOCATION ==================================


