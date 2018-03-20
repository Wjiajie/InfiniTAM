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
#include "ITMScene.h"
#include "ITMRepresentationAccess.h"
#include "../../Utils/ITMLibSettings.h"
#include "../../Engines/Reconstruction/ITMSceneReconstructionEngineFactory.h"


namespace ITMLib {


template<class TVoxel, class TIndex>
void CopySceneWithOffset_CPU(ITMScene<TVoxel, TIndex>& destination, ITMScene<TVoxel, TIndex>& source, Vector3i offset) {
	ITMSceneReconstructionEngine<TVoxel, TIndex>* reconstructionEngine =
			ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxel, TIndex>(
					ITMLibSettings::DEVICE_CPU);

	reconstructionEngine->ResetScene(&destination);

	TVoxel* originalVoxels = source.localVBA.GetVoxelBlocks();
	const ITMHashEntry* originalHashTable = source.index.GetEntries();
	int noTotalEntries = source.index.noTotalEntries;

	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		const ITMHashEntry& currentOriginalHashEntry = originalHashTable[entryId];
		if (currentOriginalHashEntry.ptr < 0) continue;

		//position of the current entry in 3D space (in voxel units)
		Vector3i canonicalHashEntryPosition = currentOriginalHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		TVoxel* localVoxelBlock = &(originalVoxels[currentOriginalHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3i originalPosition = canonicalHashEntryPosition + Vector3i(x, y, z);
					Vector3i offsetPosition = originalPosition + offset;
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					SetVoxel_CPU(destination, offsetPosition, localVoxelBlock[locId]);
				}
			}
		}
	}

}

template<class TVoxel, class TIndex>
bool SetVoxel_CPU(ITMScene<TVoxel, TIndex>& scene, Vector3i at, TVoxel voxel) {
	int lastFreeVoxelBlockId = scene.localVBA.lastFreeBlockId;
	int lastFreeExcessListId = scene.index.GetLastFreeExcessListId();
	ITMHashEntry* hashTable = scene.index.GetEntries();
	ITMHashEntry* entry = NULL;
	TVoxel* voxels = scene.localVBA.GetVoxelBlocks();
	int* voxelAllocationList = scene.localVBA.GetAllocationList();
	int* excessAllocationList = scene.index.GetExcessAllocationList();
	Vector3i blockPos;
	int linearIdx = pointToVoxelBlockPos(at, blockPos);
	if (AllocateHashEntry_CPU(TO_SHORT_FLOOR3(blockPos), hashTable, entry,
	                          lastFreeVoxelBlockId, lastFreeExcessListId, voxelAllocationList,
	                          excessAllocationList)) {
		TVoxel* localVoxelBlock = &(voxels[entry->ptr * (SDF_BLOCK_SIZE3)]);
		localVoxelBlock[linearIdx] = voxel;
	} else {
		return false;
	}
	scene.localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	scene.index.SetLastFreeExcessListId(lastFreeExcessListId);
	return true;
};


template<class TVoxel, class TIndex>
void OffsetWarps(ITMScene<TVoxel, TIndex>& scene, Vector3f offset) {
	TVoxel* voxels = scene.localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashTable = scene.index.GetEntries();
	int noTotalEntries = scene.index.noTotalEntries;
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
					localVoxelBlock[locId].warp_t_update = offset;
					localVoxelBlock[locId].warp_t += offset;
				}
			}
		}
	}
}

template<class TVoxel, class TIndex>
TVoxel ReadVoxel(ITMScene<TVoxel, TIndex>& scene, Vector3i at) {
	TVoxel* voxels = scene.localVBA.GetVoxelBlocks();
	ITMHashEntry* hashTable = scene.index.GetEntries();
	int vmIndex;
	return readVoxel(voxels, hashTable, at, vmIndex);
}


template<class TVoxel, class TIndex>
void CopySceneSlice_CPU(ITMScene<TVoxel, TIndex>* destination, ITMScene<TVoxel, TIndex>* source,
                        Vector3i extremum1, Vector3i extremum2) {

	// prep destination scene
	ITMSceneReconstructionEngine<TVoxel, TIndex>* reconstructionEngine =
			ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxel, TIndex>(
					ITMLibSettings::DEVICE_CPU);
	reconstructionEngine->ResetScene(destination);

	// ** set min/max **
	Vector3i minPoint, maxPoint;
	for (int iValue = 0; iValue < 3; iValue++) {
		if (extremum1.values[iValue] > extremum2.values[iValue]) {
			minPoint.values[iValue] = extremum2.values[iValue];
			maxPoint.values[iValue] = extremum1.values[iValue];
		} else {
			minPoint.values[iValue] = extremum1.values[iValue];
			maxPoint.values[iValue] = extremum2.values[iValue];
		}
	}

	//temporary stuff that won't fit on the stack (possibly)
	ORUtils::MemoryBlock<unsigned char>* entryAllocationTypes
			= new ORUtils::MemoryBlock<unsigned char>(TIndex::noTotalEntries,MEMORYDEVICE_CPU);
	ORUtils::MemoryBlock<Vector3s>* blockCoords = new ORUtils::MemoryBlock<Vector3s>(TIndex::noTotalEntries, MEMORYDEVICE_CPU);
	uchar* entriesAllocType = entryAllocationTypes->GetData(MEMORYDEVICE_CPU);
	Vector3s* allocationBlockCoords = blockCoords->GetData(MEMORYDEVICE_CPU);

	auto isHashBlockInRange = [&](Vector3i hashBlockPositionVoxels) {
		return hashBlockPositionVoxels.x + SDF_BLOCK_SIZE - 1 <= maxPoint.x && hashBlockPositionVoxels.x >= minPoint.x &&
		       hashBlockPositionVoxels.y + SDF_BLOCK_SIZE - 1 <= maxPoint.y && hashBlockPositionVoxels.y >= minPoint.y &&
		       hashBlockPositionVoxels.z + SDF_BLOCK_SIZE - 1 <= maxPoint.z && hashBlockPositionVoxels.z >= minPoint.z;
	};

	TVoxel* sourceVoxels = source->localVBA.GetVoxelBlocks();
	const ITMHashEntry* sourceHashTable = source->index.GetEntries();
	int noTotalEntries = source->index.noTotalEntries;
	ITMHashEntry* destinationHashTable = destination->index.GetEntries();
	TVoxel* destinationVoxels = destination->localVBA.GetVoxelBlocks();

	for (int hash = 0; hash < noTotalEntries; hash++) {
		const ITMHashEntry& currentOriginalHashEntry = sourceHashTable[hash];
		if (currentOriginalHashEntry.ptr < 0) continue;
		Vector3i originalHashBlockPosition = currentOriginalHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		if (!isHashBlockInRange(originalHashBlockPosition)) continue;
		MarkAsNeedingAllocationIfNotFound(entriesAllocType, allocationBlockCoords, hash,
		                                  currentOriginalHashEntry.pos,destinationHashTable);
	}

	AllocateHashEntriesUsingLists_CPU(destination, entriesAllocType, allocationBlockCoords,ITMLib::STABLE);

	delete blockCoords;
	delete entryAllocationTypes;

	for (int hash = 0; hash < noTotalEntries; hash++) {
		const ITMHashEntry& currentOriginalHashEntry = sourceHashTable[hash];
		if (currentOriginalHashEntry.ptr < 0) continue;

		//position of the current entry in 3D space (in voxel units)
		Vector3i originalHashBlockPosition = currentOriginalHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		if (!isHashBlockInRange(originalHashBlockPosition)) continue;

		TVoxel* localSourceVoxelBlock = &(sourceVoxels[currentOriginalHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		TVoxel* localDestinationVoxelBlock = &(sourceVoxels[currentOriginalHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		memcpy(localDestinationVoxelBlock,localSourceVoxelBlock,sizeof(TVoxel)*SDF_BLOCK_SIZE3);

	}

};

}