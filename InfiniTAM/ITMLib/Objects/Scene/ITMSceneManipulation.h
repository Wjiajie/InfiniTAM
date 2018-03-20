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
#pragma once

#include "ITMScene.h"
#include "../../ITMLibDefines.h"
#include "../../Utils/ITMHashBlockProperties.h"

//TODO: Make GPU versions -Greg (GitHub: Algomorph)

namespace ITMLib {


template<typename TFunctor, typename TVoxel, typename TIndex>
inline
void VoxelTraversal_CPU(ITMScene<TVoxel, TIndex>& scene, TFunctor& functor) {
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
					TVoxel& voxel = localVoxelBlock[locId];
					functor(voxel);
				}
			}
		}
	}
};

template<typename TFunctor, typename TVoxel, typename TIndex>
inline
void VoxelPositionTraversal_CPU(ITMScene<TVoxel, TIndex>& scene, TFunctor& functor) {
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
		//position of the current entry in 3D space (in voxel units)
		Vector3i hashEntryPosition = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					Vector3i voxelPosition = hashEntryPosition + Vector3i(x, y, z);
					TVoxel& voxel = localVoxelBlock[locId];
					functor(voxel, voxelPosition);
				}
			}
		}
	}
};

template<typename TStaticFunctor, typename TVoxel, typename TIndex>
inline
void StaticVoxelTraversal_CPU(ITMScene<TVoxel, TIndex>& scene) {
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
					TVoxel& voxel = localVoxelBlock[locId];
					TStaticFunctor::run(voxel);
				}
			}
		}
	}
};

template<typename TStaticFunctor, typename TVoxel, typename TIndex>
inline
void StaticVoxelPositionTraversal_CPU(ITMScene<TVoxel, TIndex>& scene) {
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
		//position of the current entry in 3D space (in voxel units)
		Vector3i hashEntryPosition = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3i voxelPosition = hashEntryPosition + Vector3i(x, y, z);
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxel& voxel = localVoxelBlock[locId];
					TStaticFunctor::run(voxel, voxelPosition);
				}
			}
		}
	}
};

//======================================================================================================================
//=========================================== HELPER ROUTINES FOR SCENE OPTIMIZATION PREP ==============================
//======================================================================================================================
/**
 * \brief Determines whether the hash block at the specified block position needs it's voxels to be allocated, as well
 * as whether they should be allocated in the excess list or the ordered list of the hash table.
 * If any of these are true, marks the corresponding entry in \param entryAllocationTypes
 * \param[in,out] entryAllocationTypes  array where to set the allocation type at final hashIdx index
 * \param[in,out] hashBlockCoordinates  array block coordinates for the new hash blocks at final hashIdx index
 * \param[in,out] hashIdx  takes in original index assuming coords, i.e. \refitem hashIndex(\param desiredHashBlockPosition),
 * returns final index of the hash block to be allocated (may be updated based on hash closed chaining)
 * \param[in] desiredHashBlockPosition  position of the hash block to check / allocate
 * \param[in] hashTable  hash table with existing blocks
 * \return true if the block needs allocation, false otherwise
 */
_CPU_AND_GPU_CODE_
inline bool MarkAsNeedingAllocationIfNotFound(DEVICEPTR(uchar)* entryAllocationTypes,
                                              DEVICEPTR(Vector3s)* hashBlockCoordinates,
                                              THREADPTR(int)& hashIdx,
                                              const CONSTPTR(Vector3s)& desiredHashBlockPosition,
                                              const CONSTPTR(ITMHashEntry)* hashTable) {

	ITMHashEntry hashEntry = hashTable[hashIdx];
	//check if hash table contains entry

	if (!(IS_EQUAL3(hashEntry.pos, desiredHashBlockPosition) && hashEntry.ptr >= -1)) {
		if (hashEntry.ptr >= -1) {
			//search excess list only if there is no room in ordered part
			while (hashEntry.offset >= 1) {
				hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
				hashEntry = hashTable[hashIdx];

				if (IS_EQUAL3(hashEntry.pos, desiredHashBlockPosition) && hashEntry.ptr >= -1) {
					return false;
				}
			}
			entryAllocationTypes[hashIdx] = ITMLib::NEEDS_ALLOC_IN_EXCESS_LIST;
			hashBlockCoordinates[hashIdx] = desiredHashBlockPosition;
			return true;
		}
		entryAllocationTypes[hashIdx] = ITMLib::NEEDS_ALLOC_IN_ORDERED_LIST;
		hashBlockCoordinates[hashIdx] = desiredHashBlockPosition;
		return true;
	}
	// already have hash block, no allocation needed
	return false;

};

template<typename TVoxel, typename TIndex>
inline
void AllocateHashEntriesUsingLists_CPU( ITMScene<TVoxel,TIndex>* scene, uchar* entryAllocationTypes,
                                        Vector3s* allocationBlockCoordinates, const HashBlockState& newEntryState) {
	int entryCount = TIndex::noTotalEntries;
	int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = scene->index.GetLastFreeExcessListId();
	int* voxelAllocationList = scene->localVBA.GetAllocationList();
	int* excessAllocationList = scene->index.GetExcessAllocationList();
	ITMHashEntry* liveHashEntries = scene->index.GetEntries();

	for (int iTargetHashBlock = 0; iTargetHashBlock < entryCount; iTargetHashBlock++) {
		unsigned char entryAllocType = entryAllocationTypes[iTargetHashBlock];
		switch (entryAllocType) {
			case ITMLib::NEEDS_ALLOC_IN_ORDERED_LIST:

				if (lastFreeVoxelBlockId >= 0) //there is room in the voxel block array
				{
					ITMHashEntry hashEntry;
					hashEntry.pos = allocationBlockCoordinates[iTargetHashBlock];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
					liveHashEntries[iTargetHashBlock] = hashEntry;
					entryAllocationTypes[iTargetHashBlock] = newEntryState;
					lastFreeVoxelBlockId--;
				}

				break;
			case NEEDS_ALLOC_IN_EXCESS_LIST:

				if (lastFreeVoxelBlockId >= 0 &&
				    lastFreeExcessListId >= 0) //there is room in the voxel block array and excess list
				{
					ITMHashEntry hashEntry;
					hashEntry.pos = allocationBlockCoordinates[iTargetHashBlock];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
					int exlOffset = excessAllocationList[lastFreeExcessListId];
					liveHashEntries[iTargetHashBlock].offset = exlOffset + 1; //connect to child
					liveHashEntries[SDF_BUCKET_NUM + exlOffset] = hashEntry; //add child to the excess list
					entryAllocationTypes[iTargetHashBlock] = newEntryState;
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

//======================================================================================================================
bool AllocateHashEntry_CPU(const Vector3s& hashEntryPosition,
                           ITMHashEntry* hashTable,
                           ITMHashEntry*& resultEntry,
                           int& lastFreeVoxelBlockId,
                           int& lastFreeExcessListId,
                           const int* voxelAllocationList,
                           const int* excessAllocationList);


template<class TVoxel, class TIndex>
void CopySceneWithOffset_CPU(ITMScene<TVoxel, TIndex>& destination,
                             ITMScene<TVoxel, TIndex>& source,
                             Vector3i offset);

//TODO -make this suitable for source/dest scenes with different voxel types somehow -Greg (Github: Algomorph)
void CopySceneWithOffset_CPU(ITMScene<ITMVoxelLive, ITMVoxelIndex>& destination,
                             ITMScene<ITMVoxelCanonical, ITMVoxelIndex>& source,
                             Vector3i offset);

void CopySceneWithOffset_CPU(ITMScene<ITMVoxelCanonical, ITMVoxelIndex>& destination,
                             ITMScene<ITMVoxelLive, ITMVoxelIndex>& source,
                             Vector3i offset);

template<class TVoxel, class TIndex>
void CopySceneSlice_CPU(ITMScene<TVoxel, TIndex>& destination,
                        ITMScene<TVoxel, TIndex>& source, Vector3i extremum1, Vector3i extremum2);


template<class TVoxel, class TIndex>
TVoxel ReadVoxel(ITMScene<TVoxel, TIndex>& scene, Vector3i at);

int FindHashBlock(const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData)* voxelIndex, const THREADPTR(Vector3s)& at);


template<class TVoxel, class TIndex>
bool SetVoxel_CPU(ITMScene<TVoxel, TIndex>& scene, Vector3i at, TVoxel voxel);


template<class TVoxel, class TIndex>
void
CopySceneWithOffset_CPU(ITMScene<TVoxel, TIndex>& destination, ITMScene<TVoxel, TIndex>& source, Vector3i offset);

void CopySceneWithOffset_CPU(ITMScene<ITMVoxelLive, ITMVoxelIndex>& destination,
                             ITMScene<ITMVoxel, ITMVoxelIndex>& source, Vector3i offset);
void CopySceneWithOffset_CPU(ITMScene<ITMVoxelCanonical, ITMVoxelIndex>& destination,
                             ITMScene<ITMVoxelLive, ITMVoxelIndex>& source, Vector3i offset);

template<class TVoxel, class TIndex>
void OffsetWarps(ITMScene<TVoxel, TIndex>& scene, Vector3f offset);

template<class TVoxel, class TIndex>
TVoxel ReadVoxel(ITMScene<TVoxel, TIndex>& scene, Vector3i at);


template<class TVoxel, class TIndex>
bool SetVoxel_CPU(ITMScene<TVoxel, TIndex>& scene, Vector3i at, TVoxel voxel);
}//namespace ITMLib