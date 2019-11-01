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
#pragma once

#include "../../../../ORUtils/PlatformIndependence.h"
#include "../../../Utils/ITMMath.h"
#include "../../../Objects/Scene/ITMVoxelBlockHash.h"
#include "../../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../../Engines/Common/ITMCommonFunctors.h"
#include "../../../Engines/Manipulation/Shared/ITMSceneManipulationEngine_Shared.h"
#include "../../Traversal/CPU/ITMSceneTraversal_CPU_VoxelBlockHash.h"

#ifdef __CUDACC__
#include "../../Traversal/CUDA/ITMSceneTraversal_CUDA_VoxelBlockHash.h"
#endif

using namespace ITMLib;

template<typename TWarp, typename TVoxel, typename TLookupPositionFunctor>
struct WarpBasedAllocationMarkerFunctor {
	WarpBasedAllocationMarkerFunctor(
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* sourceVolume,
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* volumeToAllocate,
			Vector3s* allocationBlockCoords,
			HashEntryState* warpedEntryAllocationStates) :

			collisionDetected(false),

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

	_CPU_AND_GPU_CODE_
	inline
	void operator()(TWarp& voxel, Vector3i voxelPosition, Vector3s hashBlockPosition) {
		Vector3f warpedPosition = TLookupPositionFunctor::GetWarpedPosition(voxel, voxelPosition);
		Vector3i warpedPositionTruncated = warpedPosition.toInt();
		// perform lookup in source volume
		int vmIndex;
#if !defined(__CUDACC__) &&!defined(WITH_OPENMP)
		const TVoxel& sourceTSDFVoxelAtWarp = readVoxel(sourceTSDFVoxels, sourceTSDFHashEntries,
		                                                warpedPositionTruncated,
		                                                vmIndex, sourceTSDFCache);
#else //don't use cache when multithreading!
		const TVoxel& sourceTSDFVoxelAtWarp = readVoxel(sourceTSDFVoxels, sourceTSDFHashEntries,
		                                                warpedPositionTruncated,
		                                                vmIndex);
#endif
		// skip truncated voxels in source scene
		if (sourceTSDFVoxelAtWarp.flags != ITMLib::VOXEL_NONTRUNCATED) return;


		int targetBlockHash = hashIndex(hashBlockPosition);

		MarkAsNeedingAllocationIfNotFound(warpedEntryAllocationStates, allocationBlockCoords, targetBlockHash,
		                                  hashBlockPosition, targetTSDFHashEntries, collisionDetected);
	}

	bool collisionDetected;

private:


	ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* targetTSDFScene;
	TVoxel* targetTSDFVoxels;
	ITMHashEntry* targetTSDFHashEntries;
	ITMVoxelBlockHash::IndexCache targetTSDFCache;

	ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* sourceTSDFScene;
	TVoxel* sourceTSDFVoxels;
	ITMHashEntry* sourceTSDFHashEntries;
	ITMVoxelBlockHash::IndexCache sourceTSDFCache;

	Vector3s* allocationBlockCoords;
	HashEntryState* warpedEntryAllocationStates;
};


template<typename TVoxel, typename TWarp, WarpType TWarpType, ITMLibSettings::DeviceType TDeviceType>
inline
void allocateFromWarpedVolume_common(
		ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField,
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* sourceTSDF,
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* targetTSDF) {

	assert(warpField->index.hashEntryCount == sourceTSDF->index.hashEntryCount &&
	       sourceTSDF->index.hashEntryCount == targetTSDF->index.hashEntryCount);

	int hashEntryCount = warpField->index.hashEntryCount;

	MemoryDeviceType memoryDeviceType = TDeviceType == ITMLibSettings::DEVICE_CPU ? MEMORYDEVICE_CPU : MEMORYDEVICE_CUDA;

	ORUtils::MemoryBlock<HashEntryState> hashEntryStates(hashEntryCount, memoryDeviceType);
	HashEntryState* hashEntryStates_device = hashEntryStates.GetData(memoryDeviceType);
	ORUtils::MemoryBlock<Vector3s> blockCoordinates(hashEntryCount, memoryDeviceType);
	Vector3s* blockCoordinates_device = blockCoordinates.GetData(memoryDeviceType);

	//Mark up hash entries in the target scene that will need allocation
	WarpBasedAllocationMarkerFunctor<TWarp, TVoxel, WarpVoxelStaticFunctor<TWarp, TWarpType>>
	hashMarkerFunctor(sourceTSDF, targetTSDF, blockCoordinates_device, hashEntryStates_device);
	do{
		//reset allocation flags
		hashEntryStates.Clear(NEEDS_NO_CHANGE);
		hashMarkerFunctor.collisionDetected = false;
		ITMSceneTraversalEngine<TWarp, ITMVoxelBlockHash, TDeviceType>::VoxelAndHashBlockPositionTraversal(
				warpField, hashMarkerFunctor);

		//Allocate the hash entries that will potentially have any data
		//FIXME (should be different for CUDA)
		AllocateHashEntriesUsingLists_CPU(targetTSDF, hashEntryStates_device, blockCoordinates_device);
	}while(hashMarkerFunctor.collisionDetected);
}