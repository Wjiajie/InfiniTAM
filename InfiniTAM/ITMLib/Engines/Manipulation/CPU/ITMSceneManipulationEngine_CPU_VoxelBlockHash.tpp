//  ================================================================
//  Created by Gregory Kramida on 9/26/19.
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


#include "ITMSceneManipulationEngine_CPU.h"
#include "../../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../Reconstruction/ITMDynamicSceneReconstructionEngineFactory.h"
#include "ITMSceneTraversal_CPU_AuxilaryFunctions.h"
#include "../Shared/ITMSceneManipulationEngine_Shared.h"

namespace ITMLib {

// region ==================================== Voxel Hash Scene Manipulation Engine ====================================

template<typename TVoxel>
void ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>::ResetScene(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene) {
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
bool
ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>::SetVoxel(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene,
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
	if (FindOrAllocateHashEntry(TO_SHORT_FLOOR3(blockPos), hashTable, entry, lastFreeVoxelBlockId, lastFreeExcessListId,
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
bool ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>::SetVoxelNoAllocation(
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene,
		Vector3i at, TVoxel voxel) {
	ITMHashEntry* hashTable = scene->index.GetEntries();
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	Vector3i blockPos;
	int linearIdx = pointToVoxelBlockPos(at, blockPos);
	int hash_index;
	if (FindHashAtPosition(hash_index, blockPos.toShortFloor(), hashTable)) {
		TVoxel* localVoxelBlock = &(voxels[hash_index]);
		localVoxelBlock[linearIdx] = voxel;
	} else {
		return false;
	}
	return true;
}


template<typename TVoxel>
TVoxel
ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>::ReadVoxel(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene,
                                                                     Vector3i at) {
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	ITMHashEntry* hashTable = scene->index.GetEntries();
	int vmIndex;
	return readVoxel(voxels, hashTable, at, vmIndex);
}

template<typename TVoxel>
TVoxel
ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>::ReadVoxel(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene,
                                                                     Vector3i at,
                                                                     ITMVoxelBlockHash::IndexCache& cache) {
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	ITMHashEntry* hashTable = scene->index.GetEntries();
	int vmIndex;
	return readVoxel(voxels, hashTable, at, vmIndex, cache);
}

template<typename TVoxel>
void ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>::OffsetWarps(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, Vector3f offset){
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented!");
}

template<typename TVoxel>
bool ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>::CopySceneSlice(
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* destination, ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* source,
		Vector6i bounds, const Vector3i& offset) {

	//temporary stuff
	auto entryAllocationTypes
			= new ORUtils::MemoryBlock<unsigned char>(ITMVoxelBlockHash::noTotalEntries, MEMORYDEVICE_CPU);
	auto blockCoords = new ORUtils::MemoryBlock<Vector3s>(ITMVoxelBlockHash::noTotalEntries, MEMORYDEVICE_CPU);
	uchar* entriesAllocType = entryAllocationTypes->GetData(MEMORYDEVICE_CPU);
	Vector3s* allocationBlockCoords = blockCoords->GetData(MEMORYDEVICE_CPU);

	TVoxel* sourceVoxels = source->localVBA.GetVoxelBlocks();
	const ITMHashEntry* sourceHashTable = source->index.GetEntries();
	int totalHashEntryCount = source->index.noTotalEntries;
	ITMHashEntry* destinationHashTable = destination->index.GetEntries();
	TVoxel* destinationVoxels = destination->localVBA.GetVoxelBlocks();

	bool voxelsWereCopied = false;

	if (offset == Vector3i(0)) {
		// *** allocate missing entries in target hash table
		// traverse source hash blocks, see which ones are at least partially inside the specified bounds
		for (int sourceHash = 0; sourceHash < totalHashEntryCount; sourceHash++) {
			const ITMHashEntry& currentSourceHashEntry = sourceHashTable[sourceHash];
			if (currentSourceHashEntry.ptr < 0) continue;
			Vector3i originalHashBlockPosition = currentSourceHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
			if (IsHashBlockFullyInRange(originalHashBlockPosition, bounds) ||
			    IsHashBlockPartiallyInRange(originalHashBlockPosition, bounds)) {
				int destinationHash = hashIndex(currentSourceHashEntry.pos);
				bool collisionDetected = false;
				MarkAsNeedingAllocationIfNotFound(entriesAllocType, allocationBlockCoords, destinationHash,
				                                  currentSourceHashEntry.pos, destinationHashTable, collisionDetected);
			}
		}

		AllocateHashEntriesUsingLists_CPU(destination, entriesAllocType, allocationBlockCoords);

		//iterate over source hash blocks & fill in the target hash blocks
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
				voxelsWereCopied = true;
			} else if (IsHashBlockPartiallyInRange(sourceHashBlockPositionVoxels, bounds)) {
				//we have to copy only parts of the scene that are within bounds
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
				voxelsWereCopied = true;
			}

		}
	} else {
		//non-zero-offset case

		//We need to allocated needed blocks in the destination hash
		//This can be done by using the destination bounds
		Vector6i destination_bounds(bounds.min_x + offset.x, bounds.min_y + offset.y, bounds.min_z + offset.z,
		                            bounds.max_x + offset.x, bounds.max_y + offset.y, bounds.max_z + offset.z);

		Vector3i destinationMinPoint(destination_bounds.min_x, destination_bounds.min_y, destination_bounds.min_z);
		Vector3i destinationMaxPoint(destination_bounds.max_x, destination_bounds.max_y, destination_bounds.max_z);
		Vector3i minPointBlock, maxPointBlock;
		pointToVoxelBlockPos(destinationMinPoint, minPointBlock);
		pointToVoxelBlockPos(destinationMaxPoint, maxPointBlock);
		for (int block_z = minPointBlock.z; block_z < maxPointBlock.z; block_z++) {
			for (int block_y = minPointBlock.y; block_y < maxPointBlock.y; block_y++) {
				for (int block_x = minPointBlock.x; block_x < maxPointBlock.x; block_x++) {
					Vector3i blockPos(block_x, block_y, block_x);
					int destinationHash = hashIndex(blockPos);
					bool collisionDetected = false;
					MarkAsNeedingAllocationIfNotFound(entriesAllocType, allocationBlockCoords, destinationHash,
					                                  blockPos.toShortFloor(), destinationHashTable, collisionDetected);
				}
			}
		}
		AllocateHashEntriesUsingLists_CPU(destination, entriesAllocType, allocationBlockCoords);
		ITMVoxelBlockHash::IndexCache source_cache;

		for (int source_z = bounds.min_z; source_z < bounds.max_z; source_z++) {
			for (int source_y = bounds.min_y; source_y < bounds.min_y; source_y++) {
				for (int source_x = bounds.min_x; source_x < bounds.max_x; source_x++) {
					Vector3i source_point(source_x, source_y, source_z);
					Vector3i destination_point = source_point + offset;
					TVoxel source_voxel =
							ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>::
							ReadVoxel(source, source_point, source_cache);
					ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>::
					SetVoxelNoAllocation(destination, destination_point, source_voxel);
					voxelsWereCopied = true;
				}
			}
		}
	}
	delete blockCoords;
	delete entryAllocationTypes;

	return voxelsWereCopied;
}

template<typename TVoxel>
bool ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>::CopyScene(
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* destination, ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* source,
		const Vector3i& offset) {

	//reset destination scene
	ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>::ResetScene(destination);

	//temporary stuff
	auto entryAllocationTypes
			= new ORUtils::MemoryBlock<unsigned char>(ITMVoxelBlockHash::noTotalEntries, MEMORYDEVICE_CPU);
	auto blockCoords = new ORUtils::MemoryBlock<Vector3s>(ITMVoxelBlockHash::noTotalEntries, MEMORYDEVICE_CPU);

	uchar* entriesAllocType = entryAllocationTypes->GetData(MEMORYDEVICE_CPU);
	Vector3s* allocationBlockCoords = blockCoords->GetData(MEMORYDEVICE_CPU);

	TVoxel* sourceVoxels = source->localVBA.GetVoxelBlocks();
	const ITMHashEntry* sourceHashTable = source->index.GetEntries();
	int totalHashEntryCount = source->index.noTotalEntries;
	ITMHashEntry* destinationHashTable = destination->index.GetEntries();
	TVoxel* destinationVoxels = destination->localVBA.GetVoxelBlocks();

	bool voxelsWereCopied = false;

	if (offset == Vector3i(0)) {
		// traverse source hash blocks, see which ones need to be allocated in destination
		for (int sourceHash = 0; sourceHash < totalHashEntryCount; sourceHash++) {
			const ITMHashEntry& currentSourceHashEntry = sourceHashTable[sourceHash];
			if (currentSourceHashEntry.ptr < 0) continue;
			Vector3i originalHashBlockPosition = currentSourceHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
			int destinationHash = hashIndex(currentSourceHashEntry.pos);
			bool collisionDetected = false;
			MarkAsNeedingAllocationIfNotFound(entriesAllocType, allocationBlockCoords, destinationHash,
			                                  currentSourceHashEntry.pos, destinationHashTable, collisionDetected);
		}

		AllocateHashEntriesUsingLists_CPU(destination, entriesAllocType, allocationBlockCoords);

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
			//we can safely copy the whole block
			memcpy(localDestinationVoxelBlock, localSourceVoxelBlock, sizeof(TVoxel) * SDF_BLOCK_SIZE3);
			voxelsWereCopied = true;
		}
	} else {
		// traverse source hash blocks
		for (int sourceHash = 0; sourceHash < totalHashEntryCount; sourceHash++) {
			const ITMHashEntry& currentSourceHashEntry = sourceHashTable[sourceHash];
			if (currentSourceHashEntry.ptr < 0) continue;

			Vector3i originalHashBlockPosition = currentSourceHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
			Vector3i sourceBlockPos;
			sourceBlockPos.x = currentSourceHashEntry.pos.x;
			sourceBlockPos.y = currentSourceHashEntry.pos.y;
			sourceBlockPos.z = currentSourceHashEntry.pos.z;
			sourceBlockPos *= SDF_BLOCK_SIZE;

			TVoxel* localSourceVoxelBlock = &(sourceVoxels[currentSourceHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

			for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
				for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
					for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
						int locId;
						Vector3i sourcePoint(sourceBlockPos.x + x, sourceBlockPos.y + y, sourceBlockPos.z + z);
						Vector3i destinationPoint = sourcePoint + offset;
						locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
						TVoxel sourceVoxel = localSourceVoxelBlock[locId];
						ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>::
						SetVoxel(destination, destinationPoint, sourceVoxel);
						voxelsWereCopied = true;
					}
				}
			}
		}
	}
	delete blockCoords;
	delete entryAllocationTypes;

	return voxelsWereCopied;
}

// endregion ===========================================================================================================


}//namespace ITMLib
