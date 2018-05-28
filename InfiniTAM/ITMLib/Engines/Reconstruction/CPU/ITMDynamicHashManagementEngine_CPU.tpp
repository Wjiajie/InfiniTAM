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
#include "ITMDynamicHashManagementEngine_CPU.h"
#include "../../../Objects/RenderStates/ITMRenderState_VH.h"
#include "../../../Utils/ITMHashBlockProperties.h"
#include "../Shared/ITMDynamicHashManagementEngine_Shared.h"
#include "../Shared/ITMSceneReconstructionEngine_Shared.h"
#include "../../../Objects/Scene/ITMSceneManipulation.h"
#include "../../../Objects/Scene/ITMSceneTraversal.h"


using namespace ITMLib;

// region ================================== CONSTRUCTORS / DESTRUCTORS ================================================

template<typename TVoxelCanonical, typename TVoxelLive>
ITMDynamicHashManagementEngine_CPU<TVoxelCanonical, TVoxelLive >::ITMDynamicHashManagementEngine_CPU() :
		canonicalEntryAllocationTypes(
				new ORUtils::MemoryBlock<unsigned char>(ITMVoxelBlockHash::noTotalEntries, MEMORYDEVICE_CPU)),
		liveEntryAllocationTypes(
				new ORUtils::MemoryBlock<unsigned char>(ITMVoxelBlockHash::noTotalEntries, MEMORYDEVICE_CPU)),
		allocationBlockCoordinates(
				new ORUtils::MemoryBlock<Vector3s>(ITMVoxelBlockHash::noTotalEntries, MEMORYDEVICE_CPU)) {}


template<typename TVoxelCanonical, typename TVoxelLive>
ITMDynamicHashManagementEngine_CPU<TVoxelCanonical, TVoxelLive >::~ITMDynamicHashManagementEngine_CPU() {
	delete canonicalEntryAllocationTypes;
	delete liveEntryAllocationTypes;
	delete allocationBlockCoordinates;
}

// endregion ===========================================================================================================

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMDynamicHashManagementEngine_CPU<TVoxelCanonical, TVoxelLive >::AllocateLiveSceneFromDepth(
		ITMScene<TVoxelLive, ITMVoxelBlockHash>* scene, const ITMView* view, const ITMTrackingState* trackingState,
		const ITMRenderState* renderState, bool onlyUpdateVisibleList, bool resetVisibleList){
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
	ITMHashSwapState* swapStates = scene->globalCache != nullptr ? scene->globalCache->GetSwapStates(false) : 0;
	int* visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
	uchar* entriesVisibleType = renderState_vh->GetEntriesVisibleType();
	uchar* liveEntryAllocationTypes = this->liveEntryAllocationTypes->GetData(MEMORYDEVICE_CPU);
	Vector3s* allocationBlockCoordinates = this->allocationBlockCoordinates->GetData(MEMORYDEVICE_CPU);
	int noTotalEntries = scene->index.noTotalEntries;

	bool useSwapping = scene->globalCache != nullptr;

	float oneOverHashEntrySize = 1.0f / (voxelSize * SDF_BLOCK_SIZE);//m

	int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = scene->index.GetLastFreeExcessListId();

	int noVisibleEntries = 0;

	memset(liveEntryAllocationTypes, 0, static_cast<size_t>(noTotalEntries));

	for (int i = 0; i < renderState_vh->noVisibleEntries; i++)
		entriesVisibleType[visibleEntryIDs[i]] = 3; // visible at previous frame and unstreamed

	//build hashVisibility
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int locId = 0; locId < depthImgSize.x * depthImgSize.y; locId++) {
		int y = locId / depthImgSize.x;
		int x = locId - y * depthImgSize.x;
		buildHashAllocAndVisibleTypePP(liveEntryAllocationTypes, entriesVisibleType, x, y, allocationBlockCoordinates,
		                                   depth, invM_d,
		                                   invProjParams_d, mu, depthImgSize, oneOverHashEntrySize, hashTable,
		                                   scene->sceneParams->viewFrustum_min,
		                                   scene->sceneParams->viewFrustum_max);
	}

	if (onlyUpdateVisibleList) useSwapping = false;
	//TODO: replace with call to AllocateHashEntriesUsingLists_CPU in SceneManipulation
	if (!onlyUpdateVisibleList) {
		AllocateHashEntriesUsingLists_CPU(scene, liveEntryAllocationTypes, allocationBlockCoordinates,ITMLib::STABLE);
	}

	//build visible list
	for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++) {
		unsigned char hashVisibleType = entriesVisibleType[targetIdx];
		const ITMHashEntry& hashEntry = hashTable[targetIdx];

		if (hashVisibleType == 3) {
			bool isVisibleEnlarged, isVisible;

			if (useSwapping) {
				checkBlockVisibility<true>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d,
				                               voxelSize,depthImgSize);
				if (!isVisibleEnlarged) hashVisibleType = 0;
			} else {
				checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d,
				                                voxelSize,depthImgSize);
				if (!isVisible) { hashVisibleType = 0; }
			}
			entriesVisibleType[targetIdx] = hashVisibleType;
		}

		if (useSwapping) {
			if (hashVisibleType > 0 && swapStates[targetIdx].state != 2) swapStates[targetIdx].state = 1;
		}

		if (hashVisibleType > 0) {
			visibleEntryIDs[noVisibleEntries] = targetIdx;
			noVisibleEntries++;
		}
	}

	//reallocate deleted ones from previous swap operation
	if (useSwapping) {
		for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++) {
			int vbaIdx;
			ITMHashEntry hashEntry = hashTable[targetIdx];

			if (entriesVisibleType[targetIdx] > 0 && hashEntry.ptr == -1) {
				vbaIdx = lastFreeVoxelBlockId;
				lastFreeVoxelBlockId--;
				if (vbaIdx >= 0) hashTable[targetIdx].ptr = voxelAllocationList[vbaIdx];
				else lastFreeVoxelBlockId++; // Avoid leaks
			}
		}
	}

	renderState_vh->noVisibleEntries = noVisibleEntries;
	scene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	scene->index.SetLastFreeExcessListId(lastFreeExcessListId);
}


// region =========================== CANONICAL HASH BLOCK ALLOCATION ==================================================
/**
 * \brief Allocates new hash blocks in the canonical (reference) frame at each frame to accommodate potential new
 * data arising from sensor motion and discovery of previously unseen surfaces
 * \tparam TVoxelCanonical type of voxels in the canonical scene
 * \tparam TVoxelLive type of voxels in the live scene
 * \tparam TIndex index used in scenes
 * \param canonicalScene the canonical (reference) scene
 * \param liveScene the live (target) scene
 */
template<typename TVoxelCanonical, typename TVoxelLive>
void
ITMDynamicHashManagementEngine_CPU<TVoxelCanonical, TVoxelLive>
::ExpandAllocatedCanonicalStableRegion(ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene) {

	uchar* entriesAllocType = this->canonicalEntryAllocationTypes->GetData(MEMORYDEVICE_CPU);
	Vector3s* allocationBlockCoords = this->allocationBlockCoordinates->GetData(MEMORYDEVICE_CPU);
	ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();

	// traverse canonical allocated blocks, if a block is stable, mark it's neighbors for allocation as boundaries
	const short neighborhoodSize = 3;//must be an odd positive integer greater than 1.
	const short neighborhoodRangeStart = -neighborhoodSize / 2;
	const short neighborhoodRangeEnd = neighborhoodSize / 2 + 1;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int canonicalHashBlockIndex = 0; canonicalHashBlockIndex < ITMVoxelBlockHash::noTotalEntries;
	     canonicalHashBlockIndex++) {
		const ITMHashEntry& currentCanonicalHashBlock = canonicalHashTable[canonicalHashBlockIndex];
		if (currentCanonicalHashBlock.ptr < 0) continue; //hash block not allocated in live, continue
		Vector3s hashBlockCoords = currentCanonicalHashBlock.pos;
		Vector3s currentBlockLocation = hashBlockCoords;
		int hashIdx = hashIndex(currentBlockLocation);
		int iNeighbor = 0;
		unsigned char entryAllocType = entriesAllocType[hashIdx];

		if (entryAllocType == ITMLib::STABLE) {
			//for all stable voxels, check the neighborhood
			for (short x = neighborhoodRangeStart; x < neighborhoodRangeEnd; x++) {
				for (short y = neighborhoodRangeStart; y < neighborhoodRangeEnd; y++) {
					for (short z = neighborhoodRangeStart; z < neighborhoodRangeEnd; z++) {
						//compute neighbor hash block position
						if (x == 0 && y == 0 && z == 0) continue;
						currentBlockLocation = hashBlockCoords + Vector3s(x, y, z);
						//compute index in hash table
						hashIdx = hashIndex(currentBlockLocation);
						MarkAsNeedingAllocationIfNotFound(entriesAllocType, allocationBlockCoords, hashIdx,
						                                  currentBlockLocation, canonicalHashTable);
						iNeighbor++;
					}
				}
			}
		}
	}
	AllocateHashEntriesUsingLists_CPU(canonicalScene, entriesAllocType, allocationBlockCoords, ITMLib::BOUNDARY);
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMDynamicHashManagementEngine_CPU<TVoxelCanonical, TVoxelLive>::AllocateCanonicalFromLive(
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene,
		ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene) {

	uchar* canonicalEntryAllocationTypes = this->canonicalEntryAllocationTypes->GetData(MEMORYDEVICE_CPU);
	Vector3s* allocationBlockCoordinates = this->allocationBlockCoordinates->GetData(MEMORYDEVICE_CPU);
	ITMHashEntry* canonicalHashEntries = canonicalScene->index.GetEntries();
	ITMHashEntry* liveHashEntries = liveScene->index.GetEntries();
	// at frame zero, allocate all the same blocks as in live frame
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int liveHashBlockIndex = 0; liveHashBlockIndex < ITMVoxelBlockHash::noTotalEntries; liveHashBlockIndex++) {
		const ITMHashEntry& currentLiveHashBlock = liveHashEntries[liveHashBlockIndex];
		//skip unfilled live blocks
		if (currentLiveHashBlock.ptr < 0) {
			continue;
		}
		Vector3s liveHashBlockCoords = currentLiveHashBlock.pos;

		//try to find a corresponding canonical block, and mark it for allocation if not found
		int canonicalBlockIndex = hashIndex(liveHashBlockCoords);
		MarkAsNeedingAllocationIfNotFound(canonicalEntryAllocationTypes, allocationBlockCoordinates,
		                                  canonicalBlockIndex,
		                                  liveHashBlockCoords, canonicalHashEntries);
	}
	AllocateHashEntriesUsingLists_CPU(canonicalScene, canonicalEntryAllocationTypes, allocationBlockCoordinates,
	                                  ITMLib::STABLE);
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMDynamicHashManagementEngine_CPU<TVoxelCanonical, TVoxelLive >::ChangeCanonicalHashEntryState(
		int hash, ITMLib::HashBlockState state) {
	this->canonicalEntryAllocationTypes->GetData(MEMORYDEVICE_CPU)[hash] = state;
}



template<typename TVoxelWarpSource, typename TVoxelSdfSource>
struct WarpBasedAllocationMarkerFunctor {
	WarpBasedAllocationMarkerFunctor(
			ITMScene<TVoxelSdfSource, ITMVoxelBlockHash>* sceneToAllocate,
			Vector3s* allocationBlockCoords,
			uchar* warpedEntryAllocationTypes,
			int flagIndex) :

			sdfScene(sceneToAllocate),
			sdfVoxels(sceneToAllocate->localVBA.GetVoxelBlocks()),
			sdfHashEntries(sceneToAllocate->index.GetEntries()),
			sdfCache(),

			allocationBlockCoords(allocationBlockCoords),
			warpedEntryAllocationTypes(warpedEntryAllocationTypes),

			flagIndex(flagIndex) {}

	void operator()(TVoxelWarpSource& voxel, Vector3i voxelPosition, Vector3s hashBlockPosition) {
		Vector3f warpedPosition = voxelPosition.toFloat() + voxel.gradient0;
		Vector3i warpedPositionTruncated = warpedPosition.toInt();
		// perform lookup
		int vmIndex;
		const TVoxelSdfSource& sdfVoxelAtWarp = readVoxel(sdfVoxels, sdfHashEntries, warpedPositionTruncated,
		                                                  vmIndex, sdfCache);
		// skip truncated voxels in raw/old live
		if (sdfVoxelAtWarp.flag_values[flagIndex] != ITMLib::VOXEL_NONTRUNCATED) return;

		int liveBlockHash = hashIndex(hashBlockPosition);
		MarkAsNeedingAllocationIfNotFound(warpedEntryAllocationTypes, allocationBlockCoords, liveBlockHash,
		                                  hashBlockPosition, sdfHashEntries);

	}

private:
	ITMScene<TVoxelSdfSource, ITMVoxelBlockHash>* sdfScene;
	TVoxelSdfSource* sdfVoxels;
	ITMHashEntry* sdfHashEntries;
	ITMVoxelBlockHash::IndexCache sdfCache;


	Vector3s* allocationBlockCoords;
	uchar* warpedEntryAllocationTypes;
	const int flagIndex;
};
/**
 * \brief Helper method which looks at voxel grid with warps and an SDF voxel grid and allocates all hash blocks in the
 * SDF grid where warp vectors are pointing to (if not already allocated).
 * \details scans each (allocated) voxel in the SDF voxel grid, checks the warp vector at the corresponding location,
 * finds the voxel where the warp vector is pointing to, and, if the hash block for that voxel is not yet allocated,
 * allocates it.
 * \param warpSourceScene voxel grid where each voxel has a .warp Vector3f field defined
 * \param sdfScene sdf grid whose hash blocks to allocate if needed
 * \param fieldIndex index of the sdf / flag field to use in the sdfScene
 */
template<typename TVoxelCanonical, typename TVoxelLive>
void ITMDynamicHashManagementEngine_CPU<TVoxelCanonical, TVoxelLive >::AllocateWarpedLive(
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* warpSourceScene,
		ITMScene<TVoxelLive, ITMVoxelBlockHash>* sdfScene, int fieldIndex) {
	int entryCount = ITMVoxelBlockHash::noTotalEntries;
	Vector3s* allocationBlockCoords = this->allocationBlockCoordinates->GetData(MEMORYDEVICE_CPU);
	uchar* liveEntryAllocationTypes = this->liveEntryAllocationTypes->GetData(MEMORYDEVICE_CPU);
	//reset allocation types
	memset(liveEntryAllocationTypes, (unsigned char) 0, static_cast<size_t>(entryCount));

	//Mark up hash entries in the target scene that will need allocation
	WarpBasedAllocationMarkerFunctor<TVoxelCanonical, TVoxelLive>
			hashMarkerFunctor(sdfScene, allocationBlockCoords, liveEntryAllocationTypes, fieldIndex);
	VoxelAndHashBlockPositionTraversal_CPU(*warpSourceScene, hashMarkerFunctor);

	//Allocate the hash entries that will potentially have any data
	AllocateHashEntriesUsingLists_CPU(sdfScene, liveEntryAllocationTypes, allocationBlockCoords,
	                                  ITMLib::STABLE);
}

// endregion ==================================== END CANONICAL HASH BLOCK ALLOCATION ==================================


