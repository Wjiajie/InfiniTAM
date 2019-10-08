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
#include "../Shared/ITMDynamicHashManagementEngine_Shared.h"
#include "../Shared/ITMSceneReconstructionEngine_Shared.h"
#include "../../Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../../Traversal/CPU/ITMSceneTraversal_CPU_VoxelBlockHash.h"
#include "../../Common/ITMCommonFunctors.h"
#include "../../Manipulation/Shared/ITMSceneManipulationEngine_Shared.h"


using namespace ITMLib;

// region ================================== CONSTRUCTORS / DESTRUCTORS ================================================

template<typename TVoxelCanonical, typename TVoxelLive>
ITMHashAllocationEngine_CPU<TVoxelCanonical, TVoxelLive>::ITMHashAllocationEngine_CPU() :
		targetSceneHashBlockStates(
				new ORUtils::MemoryBlock<unsigned char>(ITMVoxelBlockHash::noTotalEntries, MEMORYDEVICE_CPU)),
		sourceSceneHashBlockStates(
				new ORUtils::MemoryBlock<unsigned char>(ITMVoxelBlockHash::noTotalEntries, MEMORYDEVICE_CPU)),
		targetSceneHashBlockCoordinates(
				new ORUtils::MemoryBlock<Vector3s>(ITMVoxelBlockHash::noTotalEntries, MEMORYDEVICE_CPU)) {}


template<typename TVoxelCanonical, typename TVoxelLive>
ITMHashAllocationEngine_CPU<TVoxelCanonical, TVoxelLive>::~ITMHashAllocationEngine_CPU() {
	delete targetSceneHashBlockStates;
	delete sourceSceneHashBlockStates;
	delete targetSceneHashBlockCoordinates;
}

// endregion ===========================================================================================================
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
	uchar* liveEntryAllocationTypes = this->sourceSceneHashBlockStates->GetData(MEMORYDEVICE_CPU);
	Vector3s* allocationBlockCoordinates = this->targetSceneHashBlockCoordinates->GetData(MEMORYDEVICE_CPU);
	int noTotalEntries = scene->index.noTotalEntries;

	float oneOverHashEntrySize = 1.0f / (voxelSize * SDF_BLOCK_SIZE);//m

	bool collisionDetected;

	bool useSwapping = scene->globalCache != nullptr;
	do {
		memset(liveEntryAllocationTypes, (uint) NEEDS_NO_CHANGE, static_cast<size_t>(noTotalEntries));
		collisionDetected = false;
		for (int i = 0; i < renderState_vh->noVisibleEntries; i++)
			hashBlockVisibilityTypes[visibleEntryIDs[i]] = 3; // visible at previous frame and unstreamed

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int locId = 0; locId < depthImgSize.x * depthImgSize.y; locId++) {
			int y = locId / depthImgSize.x;
			int x = locId - y * depthImgSize.x;

			buildHashAllocAndVisibleTypePP(liveEntryAllocationTypes, hashBlockVisibilityTypes, x, y,
			                               allocationBlockCoordinates, depth, invM_d, invProjParams_d, mu, depthImgSize,
			                               oneOverHashEntrySize,
			                               hashTable, scene->sceneParams->viewFrustum_min,
			                               scene->sceneParams->viewFrustum_max, collisionDetected);
		}

		if (onlyUpdateVisibleList) {
			useSwapping = false;
			collisionDetected = false;
		} else {
			AllocateHashEntriesUsingLists_SetVisibility_CPU(scene, liveEntryAllocationTypes,
			                                                allocationBlockCoordinates, hashBlockVisibilityTypes);
		}
	} while (collisionDetected);

	int noVisibleEntries = 0;
	//build visible list
	for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++) {
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
		for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++) {
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

	uchar* targetSceneHashBlockStateData = this->targetSceneHashBlockStates->GetData(MEMORYDEVICE_CPU);

	Vector3s* targetSceneHashBlockCoordinateData = this->targetSceneHashBlockCoordinates->GetData(MEMORYDEVICE_CPU);
	ITMHashEntry* targetHashEntries = targetVolume->index.GetEntries();
	ITMHashEntry* sourceHashEntries = sourceVolume->index.GetEntries();

	bool collisionDetected;

	do {
		collisionDetected = false;
		//reset target allocation states
		memset(targetSceneHashBlockStateData, ITMLib::NEEDS_NO_CHANGE,
		       static_cast<size_t>(ITMVoxelBlockHash::noTotalEntries));
		// at frame zero, allocate all the same blocks as in live frame
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int sourceHash = 0; sourceHash < ITMVoxelBlockHash::noTotalEntries; sourceHash++) {

			const ITMHashEntry& currentSourceHashBlock = sourceHashEntries[sourceHash];
			//skip unfilled live blocks
			if (currentSourceHashBlock.ptr < 0) {
				continue;
			}
			Vector3s sourceHashBlockCoords = currentSourceHashBlock.pos;

			//try to find a corresponding canonical block, and mark it for allocation if not found
			int targetHash = hashIndex(sourceHashBlockCoords);

			MarkAsNeedingAllocationIfNotFound(targetSceneHashBlockStateData, targetSceneHashBlockCoordinateData,
			                                  targetHash, sourceHashBlockCoords, targetHashEntries,
			                                  collisionDetected);
		}
		AllocateHashEntriesUsingLists_CPU(targetVolume, targetSceneHashBlockStateData,
		                                  targetSceneHashBlockCoordinateData);
	} while (collisionDetected);

}


template<typename TVoxelWarpSource, typename TVoxelTSDF, typename TLookupPositionFunctor>
struct WarpBasedAllocationMarkerFunctor {
	WarpBasedAllocationMarkerFunctor(
			ITMVoxelVolume<TVoxelTSDF, ITMVoxelBlockHash>* sourceVolume,
			ITMVoxelVolume<TVoxelTSDF, ITMVoxelBlockHash>* volumeToAllocate,
			Vector3s* allocationBlockCoords,
			uchar* warpedEntryAllocationStates) :

			targetTSDFScene(volumeToAllocate),
			targetTSDFVoxels(volumeToAllocate->localVBA.GetVoxelBlocks()),
			targetTSDFHashEntries(volumeToAllocate->index.GetEntries()),
			targetTSDFCache(),

			sourceTSDFScene(sourceVolume),
			sourceTSDFVoxels(sourceVolume->localVBA.GetVoxelBlocks()),
			sourceTSDFHashEntries(sourceVolume->index.GetEntries()),
			sourceTSDFCache(),

			allocationBlockCoords(allocationBlockCoords),
			warpedEntryAllocationStates(warpedEntryAllocationStates) {}

	void operator()(TVoxelWarpSource& voxel, Vector3i voxelPosition, Vector3s hashBlockPosition) {
		Vector3f warpedPosition = TLookupPositionFunctor::GetWarpedPosition(voxel, voxelPosition);
		Vector3i warpedPositionTruncated = warpedPosition.toInt();
		// perform lookup in source volume
		int vmIndex;
		const TVoxelTSDF& sourceTSDFVoxelAtWarp = readVoxel(sourceTSDFVoxels, sourceTSDFHashEntries,
		                                                    warpedPositionTruncated,
		                                                    vmIndex, sourceTSDFCache);

		voxelCount++;
		// skip truncated voxels in source scene
		if (sourceTSDFVoxelAtWarp.flags != ITMLib::VOXEL_NONTRUNCATED) return;
		nontruncatedCount++;

		int liveBlockHash = hashIndex(hashBlockPosition);
		bool collisionDetected = false;
		if (MarkAsNeedingAllocationIfNotFound(warpedEntryAllocationStates, allocationBlockCoords, liveBlockHash,
		                                      hashBlockPosition, targetTSDFHashEntries, collisionDetected)) {
			countToAllocate++;
		}
	}

	//_DEBUG
	int countToAllocate = 0;
	int voxelCount = 0;
	int nontruncatedCount = 0;
private:
	ITMVoxelVolume<TVoxelTSDF, ITMVoxelBlockHash>* targetTSDFScene;
	TVoxelTSDF* targetTSDFVoxels;
	ITMHashEntry* targetTSDFHashEntries;
	ITMVoxelBlockHash::IndexCache targetTSDFCache;

	ITMVoxelVolume<TVoxelTSDF, ITMVoxelBlockHash>* sourceTSDFScene;
	TVoxelTSDF* sourceTSDFVoxels;
	ITMHashEntry* sourceTSDFHashEntries;
	ITMVoxelBlockHash::IndexCache sourceTSDFCache;

	Vector3s* allocationBlockCoords;
	uchar* warpedEntryAllocationStates;
};



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
	int entryCount = ITMVoxelBlockHash::noTotalEntries;
	Vector3s* allocationBlockCoordinatesData = this->targetSceneHashBlockCoordinates->GetData(MEMORYDEVICE_CPU);
	uchar* liveEntryAllocationStates = this->sourceSceneHashBlockStates->GetData(MEMORYDEVICE_CPU);
	//reset allocation types
	memset(liveEntryAllocationStates, (unsigned char) 0, static_cast<size_t>(entryCount));

	//Mark up hash entries in the target scene that will need allocation
	WarpBasedAllocationMarkerFunctor<TWarp, TVoxel, WarpVoxelStaticFunctor<TWarp, TWarpType>>
			hashMarkerFunctor(sourceTSDF, targetTSDF, allocationBlockCoordinatesData, liveEntryAllocationStates);
	ITMSceneTraversalEngine<TWarp, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::VoxelAndHashBlockPositionTraversal(
			warpField, hashMarkerFunctor);

	//Allocate the hash entries that will potentially have any data
	AllocateHashEntriesUsingLists_CPU(targetTSDF, liveEntryAllocationStates, allocationBlockCoordinatesData);
}


// endregion ==================================== END CANONICAL HASH BLOCK ALLOCATION ==================================


