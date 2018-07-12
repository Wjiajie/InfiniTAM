//  ================================================================
//  Created by Gregory Kramida on 11/5/17.
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

#include "ITMSceneManipulation.h"
#include "../../Objects/Scene/ITMScene.h"
#include "../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../Objects/Scene/ITMVoxelBlockHash.h"
#include "../../Objects/Scene/ITMPlainVoxelArray.h"
#include "../../Utils/ITMLibSettings.h"
#include "../Reconstruction/ITMDynamicSceneReconstructionEngineFactory.h"
#include "../Reconstruction/Interface/ITMSceneReconstructionEngine.h"
#include "../Reconstruction/ITMSceneReconstructionEngineFactory.h"


namespace ITMLib {

template<typename TVoxelSource, typename TVoxelDestination>
void
ITMTwoSceneManipulationEngine_CPU<TVoxelSource, TVoxelDestination, ITMVoxelBlockHash>::
CopySceneSDFandFlagsWithOffset_CPU(
		ITMScene<TVoxelDestination, ITMVoxelBlockHash>* destination,
		ITMScene<TVoxelSource, ITMVoxelBlockHash>* source,
		Vector3i offset) {

	ITMSceneManipulationEngine_CPU<TVoxelDestination, ITMVoxelBlockHash>::ResetScene(destination);

	TVoxelSource* originalVoxels = source->localVBA.GetVoxelBlocks();
	const ITMHashEntry* originalHashTable = source->index.GetEntries();
	int noTotalEntries = source->index.noTotalEntries;

	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		const ITMHashEntry& currentOriginalHashEntry = originalHashTable[entryId];
		if (currentOriginalHashEntry.ptr < 0) continue;

		//position of the current entry in 3D space (in voxel units)
		Vector3i canonicalHashEntryPosition = currentOriginalHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		TVoxelSource* localVoxelBlock = &(originalVoxels[currentOriginalHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3i originalPosition = canonicalHashEntryPosition + Vector3i(x, y, z);
					Vector3i offsetPosition = originalPosition + offset;
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxelSource& voxelSource = localVoxelBlock[locId];
					TVoxelDestination voxelDestination;
					voxelDestination.sdf = voxelSource.sdf;
					voxelDestination.flags = voxelSource.flags;
					ITMSceneManipulationEngine_CPU<TVoxelDestination, ITMVoxelBlockHash>
					::SetVoxel(destination, offsetPosition, voxelDestination);
				}
			}
		}
	}

}

template<typename TVoxel>
inline static int
ComputeLinearIndexFromPosition_PlainVoxelArray(ITMScene<TVoxel, ITMPlainVoxelArray>* scene, Vector3i position) {
	return position.z * (scene->index.getVolumeSize().z * scene->index.getVolumeSize().y)
	                       + position.y * (scene->index.getVolumeSize().x + position.x);
};

template<typename TVoxel>
inline static int
ComputeLinearIndexFromPosition_PlainVoxelArray(ITMScene<TVoxel, ITMPlainVoxelArray>* scene, int x, int y, int z) {
	return z * (scene->index.getVolumeSize().z * scene->index.getVolumeSize().y)
	                       + y * (scene->index.getVolumeSize().x + x);
};


template<typename TVoxelSource, typename TVoxelDestination>
void
ITMTwoSceneManipulationEngine_CPU<TVoxelSource, TVoxelDestination, ITMPlainVoxelArray>::CopySceneSDFandFlagsWithOffset_CPU(
		ITMScene<TVoxelDestination, ITMPlainVoxelArray>* destination,
		ITMScene<TVoxelSource, ITMPlainVoxelArray>* source, Vector3i offset) {
	ITMSceneManipulationEngine_CPU<TVoxelDestination, ITMPlainVoxelArray>::ResetScene(destination);

	const TVoxelSource* originalVoxels = source->localVBA.GetVoxelBlocks();
	TVoxelDestination* destinationVoxels = destination->localVBA.GetVoxelBlocks();

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int sourceIndex = 0; sourceIndex < source->index.getVolumeSize().x * source->index.getVolumeSize().y *
	                                        source->index.getVolumeSize().z; ++sourceIndex) {
		int z = sourceIndex / (source->index.getVolumeSize().x * source->index.getVolumeSize().y);
		int tmp = sourceIndex - z * source->index.getVolumeSize().x * source->index.getVolumeSize().y;
		int y = tmp / source->index.getVolumeSize().x;
		int x = tmp - y * source->index.getVolumeSize().x;
		int destX = x + offset.x;
		int destY = y + offset.y;
		int destZ = z + offset.z;

		if (destX < 0 || destX > destination->index.getVolumeSize().x ||
		    destY < 0 || destY > destination->index.getVolumeSize().y ||
		    destZ < 0 || destZ > destination->index.getVolumeSize().z)
			continue; //out-of-bounds

		int destinationIndex = ComputeLinearIndexFromPosition_PlainVoxelArray(destination, destX, destY, destZ);
		const TVoxelSource& sourceVoxel = originalVoxels[sourceIndex];
		TVoxelDestination& destinationVoxel = destinationVoxels[destinationIndex];
		destinationVoxel.sdf = sourceVoxel.sdf;
		destinationVoxel.flags = sourceVoxel.flags;

	}

}


template<typename TVoxel>
void ITMLib::ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>::ResetScene(
		ITMLib::ITMScene<TVoxel, ITMLib::ITMVoxelBlockHash>* scene) {
	int numBlocks = scene->index.getNumAllocatedVoxelBlocks();
	int blockSize = scene->index.getVoxelBlockSize();

	TVoxel* voxelBlocks_ptr = scene->localVBA.GetVoxelBlocks();
	for (int i = 0; i < numBlocks * blockSize; ++i) voxelBlocks_ptr[i] = TVoxel();
	int* vbaAllocationList_ptr = scene->localVBA.GetAllocationList();
	for (int i = 0; i < numBlocks; ++i) vbaAllocationList_ptr[i] = i;
	scene->localVBA.lastFreeBlockId = numBlocks - 1;

	ITMHashEntry tmpEntry;
	memset(&tmpEntry, 0, sizeof(ITMHashEntry));
	tmpEntry.ptr = -2;
	ITMHashEntry* hashEntry_ptr = scene->index.GetEntries();
	for (int i = 0; i < scene->index.noTotalEntries; ++i) hashEntry_ptr[i] = tmpEntry;
	int* excessList_ptr = scene->index.GetExcessAllocationList();
	for (int i = 0; i < SDF_EXCESS_LIST_SIZE; ++i) excessList_ptr[i] = i;

	scene->index.SetLastFreeExcessListId(SDF_EXCESS_LIST_SIZE - 1);
}

template<typename TVoxel>
bool ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>::SetVoxel(ITMScene<TVoxel, ITMVoxelBlockHash>* scene,
                                                                         Vector3i at, TVoxel voxel) {
	int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = scene->index.GetLastFreeExcessListId();
	ITMHashEntry* hashTable = scene->index.GetEntries();
	ITMHashEntry* entry = nullptr;
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	int* voxelAllocationList = scene->localVBA.GetAllocationList();
	int* excessAllocationList = scene->index.GetExcessAllocationList();
	Vector3i blockPos;
	int linearIdx = pointToVoxelBlockPos(at, blockPos);
	int hash;
	if (AllocateHashEntry_CPU(TO_SHORT_FLOOR3(blockPos), hashTable, entry, lastFreeVoxelBlockId, lastFreeExcessListId,
	                          voxelAllocationList, excessAllocationList, hash)) {
		TVoxel* localVoxelBlock = &(voxels[entry->ptr * (SDF_BLOCK_SIZE3)]);
		localVoxelBlock[linearIdx] = voxel;
	} else {
		return false;
	}
	scene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	scene->index.SetLastFreeExcessListId(lastFreeExcessListId);
	return true;
}


template<typename TVoxel>
bool ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::SetVoxel(ITMScene<TVoxel, ITMPlainVoxelArray>* scene,
                                                                          Vector3i at, TVoxel voxel) {
	int arrayIndex = ComputeLinearIndexFromPosition_PlainVoxelArray(scene, at);
	scene->localVBA.GetVoxelBlocks()[arrayIndex] = voxel;
}


template<typename TVoxel>
TVoxel ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>::ReadVoxel(ITMScene<TVoxel, ITMVoxelBlockHash>* scene,
                                                                            Vector3i at) {
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	ITMHashEntry* hashTable = scene->index.GetEntries();
	int vmIndex;
	return readVoxel(voxels, hashTable, at, vmIndex);
}

template<typename TVoxel>
TVoxel ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>::ReadVoxel(ITMScene<TVoxel, ITMVoxelBlockHash>* scene,
                                                                            Vector3i at,
                                                                            ITMVoxelBlockHash::IndexCache& cache) {
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	ITMHashEntry* hashTable = scene->index.GetEntries();
	int vmIndex;
	return readVoxel(voxels, hashTable, at, vmIndex, cache);
}

template<typename TVoxel>
TVoxel
ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::ReadVoxel(ITMScene<TVoxel, ITMPlainVoxelArray>* scene,
                                                                      Vector3i at) {
	int index = ComputeLinearIndexFromPosition_PlainVoxelArray(scene, at);
	return scene->localVBA.GetVoxelBlocks()[index];
}

template<typename TVoxel>
TVoxel
ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::ReadVoxel(ITMScene<TVoxel, ITMPlainVoxelArray>* scene,
                                                                      Vector3i at,
                                                                      ITMPlainVoxelArray::IndexCache& cache) {
	int index = ComputeLinearIndexFromPosition_PlainVoxelArray(scene, at);
	return scene->localVBA.GetVoxelBlocks()[index];
}


template<typename TVoxel, typename TIndex, bool hasWarpInformation>
struct OffsetWarpsFunctor;

template<typename TVoxel>
struct OffsetWarpsFunctor<TVoxel, ITMVoxelBlockHash, true> {
	static void OffsetWarps(ITMScene<TVoxel, ITMVoxelBlockHash>* scene, Vector3f offset) {
		TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
		const ITMHashEntry* hashTable = scene->index.GetEntries();
		int noTotalEntries = scene->index.noTotalEntries;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int entryId = 0; entryId < noTotalEntries; entryId++) {
			const ITMHashEntry& currentHashEntry = hashTable[entryId];
			if (currentHashEntry.ptr < 0) continue;
			TVoxel* localVoxelBlock = &(voxels[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
			for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
				for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
					for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
						int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
						localVoxelBlock[locId].warp += offset;
					}
				}
			}
		}
	}
};


template<typename TVoxel>
struct OffsetWarpsFunctor<TVoxel, ITMPlainVoxelArray, true> {
	static void OffsetWarps(ITMScene<TVoxel, ITMPlainVoxelArray>* scene, Vector3f offset) {
		TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int linearArrayIndex = 0;
		     linearArrayIndex < scene->index.getVolumeSize().x * scene->index.getVolumeSize().y *
		                        scene->index.getVolumeSize().z; ++linearArrayIndex) {
			voxels[linearArrayIndex].warp += offset;
		}
	}
};

template<typename TVoxel>
struct OffsetWarpsFunctor<TVoxel, ITMVoxelBlockHash, false> {
	static void OffsetWarps(ITMScene<TVoxel, ITMVoxelBlockHash>* scene, Vector3f offset) {
		DIEWITHEXCEPTION_REPORTLOCATION("Warps not defined for scene of using this voxel type.");
	}
};

template<typename TVoxel>
struct OffsetWarpsFunctor<TVoxel, ITMPlainVoxelArray, false> {
	static void OffsetWarps(ITMScene<TVoxel, ITMPlainVoxelArray>* scene, Vector3f offset) {
		DIEWITHEXCEPTION_REPORTLOCATION("Warps not defined for scene of using this voxel type.");
	}
};

template<typename TVoxel>
void
ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::OffsetWarps(ITMScene<TVoxel, ITMPlainVoxelArray>* scene,
                                                                        Vector3f offset) {
	OffsetWarpsFunctor<TVoxel, ITMPlainVoxelArray, TVoxel::hasWarpInformation>::OffsetWarps(scene, offset);
}


template<typename TVoxel>
void ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>::OffsetWarps(ITMScene<TVoxel, ITMVoxelBlockHash>* scene,
                                                                            Vector3f offset) {
	OffsetWarpsFunctor<TVoxel, ITMVoxelBlockHash, TVoxel::hasWarpInformation>::OffsetWarps(scene, offset);
}

template<typename TVoxel>
bool ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>::CopySceneSlice(
		ITMScene<TVoxel, ITMVoxelBlockHash>* destination, ITMScene<TVoxel, ITMVoxelBlockHash>* source,
		Vector6i bounds) {

	// prep destination scene
	ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>::ResetScene(destination);

	//temporary stuff
	ORUtils::MemoryBlock<unsigned char>* entryAllocationTypes
			= new ORUtils::MemoryBlock<unsigned char>(ITMVoxelBlockHash::noTotalEntries, MEMORYDEVICE_CPU);
	ORUtils::MemoryBlock<Vector3s>* blockCoords = new ORUtils::MemoryBlock<Vector3s>(ITMVoxelBlockHash::noTotalEntries,
	                                                                                 MEMORYDEVICE_CPU);
	uchar* entriesAllocType = entryAllocationTypes->GetData(MEMORYDEVICE_CPU);
	Vector3s* allocationBlockCoords = blockCoords->GetData(MEMORYDEVICE_CPU);

	TVoxel* sourceVoxels = source->localVBA.GetVoxelBlocks();
	const ITMHashEntry* sourceHashTable = source->index.GetEntries();
	int totalHashEntryCount = source->index.noTotalEntries;
	ITMHashEntry* destinationHashTable = destination->index.GetEntries();
	TVoxel* destinationVoxels = destination->localVBA.GetVoxelBlocks();


	for (int sourceHash = 0; sourceHash < totalHashEntryCount; sourceHash++) {
		const ITMHashEntry& currentOriginalHashEntry = sourceHashTable[sourceHash];
		if (currentOriginalHashEntry.ptr < 0) continue;
		Vector3i originalHashBlockPosition = currentOriginalHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		if (IsHashBlockFullyInRange(originalHashBlockPosition, bounds) ||
		    IsHashBlockPartiallyInRange(originalHashBlockPosition, bounds)) {
			int destinationHash = hashIndex(currentOriginalHashEntry.pos);
			bool collisionDetected = false;
			MarkAsNeedingAllocationIfNotFound(entriesAllocType, allocationBlockCoords, destinationHash,
			                                  currentOriginalHashEntry.pos, destinationHashTable, collisionDetected);
		}
	}

	AllocateHashEntriesUsingLists_CPU(destination, entriesAllocType, allocationBlockCoords);

	delete blockCoords;
	delete entryAllocationTypes;

	bool newSceneContainsVoxels = false;

	for (int sourceHash = 0; sourceHash < totalHashEntryCount; sourceHash++) {
		const ITMHashEntry& sourceHashEntry = sourceHashTable[sourceHash];

		if (sourceHashEntry.ptr < 0) continue;
		int destinationHash;
		FindHashAtPosition(destinationHash, sourceHashEntry.pos, destinationHashTable);
		const ITMHashEntry& destinationHashEntry = destinationHashTable[destinationHash];

		//position of the current entry in 3D space (in voxel units)
		Vector3i sourceHashBlockPositionVoxels = sourceHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		TVoxel* localSourceVoxelBlock = &(sourceVoxels[sourceHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		TVoxel* localDestinationVoxelBlock = &(destinationVoxels[destinationHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		if (IsHashBlockFullyInRange(sourceHashBlockPositionVoxels, bounds)) {
			//we can safely copy the whole block
			memcpy(localDestinationVoxelBlock, localSourceVoxelBlock, sizeof(TVoxel) * SDF_BLOCK_SIZE3);
			newSceneContainsVoxels = true;
		} else if (IsHashBlockPartiallyInRange(sourceHashBlockPositionVoxels, bounds)) {
			int zRangeStart, zRangeEnd, yRangeStart, yRangeEnd, xRangeStart, xRangeEnd;
			ComputeCopyRanges(xRangeStart, xRangeEnd, yRangeStart, yRangeEnd, zRangeStart, zRangeEnd,
			                  sourceHashBlockPositionVoxels, bounds);
			for (int z = zRangeStart; z < zRangeEnd; z++) {
				for (int y = yRangeStart; y < yRangeEnd; y++) {
					for (int x = xRangeStart; x < xRangeEnd; x++) {
						int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
						memcpy(&localDestinationVoxelBlock[locId], &localSourceVoxelBlock[locId], sizeof(TVoxel));
					}
				}
			}
			newSceneContainsVoxels = true;
		}

	}
	return newSceneContainsVoxels;
}


template<typename TVoxel>
bool ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::CopySceneSlice(
		ITMScene<TVoxel, ITMPlainVoxelArray>* destination, ITMScene<TVoxel, ITMPlainVoxelArray>* source,
		Vector6i bounds) {

	if (source->index.getVolumeSize() != destination->index.getVolumeSize()) {
		DIEWITHEXCEPTION_REPORTLOCATION("The two scenes must have equal size");
	}

	// prep destination scene
	ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::ResetScene(destination);


	TVoxel* sourceVoxels = source->localVBA.GetVoxelBlocks();
	TVoxel* destinationVoxels = destination->localVBA.GetVoxelBlocks();

	for (int z = bounds.min_z; z < bounds.max_z; z++) {
		for (int y = bounds.min_y; y < bounds.max_y; y++) {
			for (int x = bounds.min_x; x < bounds.max_x; x++) {
				int linearIndex = ComputeLinearIndexFromPosition_PlainVoxelArray(source, x, y, z);
				memcpy(&destinationVoxels[linearIndex], &sourceVoxels[linearIndex], sizeof(TVoxel));
			}
		}
	}
	return true;
}


template<typename TVoxel>
void ITMLib::ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::ResetScene(
		ITMLib::ITMScene<TVoxel, ITMLib::ITMPlainVoxelArray>* scene) {
	int numBlocks = scene->index.getNumAllocatedVoxelBlocks();
	int blockSize = scene->index.getVoxelBlockSize();

	TVoxel* voxelBlocks_ptr = scene->localVBA.GetVoxelBlocks();
	for (int i = 0; i < numBlocks * blockSize; ++i) voxelBlocks_ptr[i] = TVoxel();
	int* vbaAllocationList_ptr = scene->localVBA.GetAllocationList();
	for (int i = 0; i < numBlocks; ++i) vbaAllocationList_ptr[i] = i;
	scene->localVBA.lastFreeBlockId = numBlocks - 1;
}


}//namespace ITMLib