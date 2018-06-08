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

#include "../../Objects/Scene/ITMScene.h"
#include "../../ITMLibDefines.h"
#include "../../Utils/ITMHashBlockProperties.h"
#include "../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../Utils/ITMPrintHelpers.h"

//TODO: Make GPU versions -Greg (GitHub: Algomorph)

namespace ITMLib {

template<typename TVoxel, typename TIndex>
class ITMSceneManipulationEngine_CPU{};

template<typename TVoxel>
class ITMSceneManipulationEngine_CPU<TVoxel,ITMVoxelBlockHash>{
public:
	static void ResetScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene);
};

template<typename TVoxel>
class ITMSceneManipulationEngine_CPU<TVoxel,ITMPlainVoxelArray>{
public:
	static void ResetScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene);
};

// region ==================================== GENERAL HASH MANAGEMENT =================================================
/**
 * \brief Look for the hash index of the hash entry with the specified position
 * \param hashIdx [out] the index of the hash entry corresponding to the specified position
 * \param hashBlockPosition [in] spacial position of the sough-after hash entry (in hash blocks)
 * \param hashTable [in] the hash table to search
 * \return true if hash block is allocated, false otherwise
 */
inline bool FindHashAtPosition(THREADPTR(int)& hashIdx,
                               const CONSTPTR(Vector3s)& hashBlockPosition,
                               const CONSTPTR(ITMHashEntry)* hashTable) {
	hashIdx = hashIndex(hashBlockPosition);
	ITMHashEntry hashEntry = hashTable[hashIdx];
	if (!(IS_EQUAL3(hashEntry.pos, hashBlockPosition) && hashEntry.ptr >= -1)) {
		if (hashEntry.ptr >= -1) {
			//search excess list only if there is no room in ordered part
			while (hashEntry.offset >= 1) {
				hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
				hashEntry = hashTable[hashIdx];

				if (IS_EQUAL3(hashEntry.pos, hashBlockPosition) && hashEntry.ptr >= -1) {
					return true;
				}
			}
			return false;
		}
		return false;
	}
	return true;
}


bool AllocateHashEntry_CPU(const Vector3s& hashEntryPosition, ITMHashEntry* hashTable, ITMHashEntry*& resultEntry,
                           int& lastFreeVoxelBlockId, int& lastFreeExcessListId, const int* voxelAllocationList,
                           const int* excessAllocationList, int& hash);
// endregion ===========================================================================================================
// endregion ===========================================================================================================
// region =================================== HELPER ROUTINES FOR SCENE OPTIMIZATION PREP ==============================
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
			entryAllocationTypes[hashIdx] = ITMLib::NEEDS_ALLOCATION_IN_EXCESS_LIST;
			hashBlockCoordinates[hashIdx] = desiredHashBlockPosition;
			return true;
		}
		entryAllocationTypes[hashIdx] = ITMLib::NEEDS_ALLOCATION_IN_ORDERED_LIST;
		hashBlockCoordinates[hashIdx] = desiredHashBlockPosition;
		return true;
	}
	// already have hash block, no allocation needed
	return false;

};


template<typename TVoxel, typename TIndex>
inline
void AllocateHashEntriesUsingLists_CPU(ITMScene<TVoxel, TIndex>* scene, uchar* entryAllocationTypes,
                                       Vector3s* allocationBlockCoordinates) {
	int entryCount = TIndex::noTotalEntries;
	int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = scene->index.GetLastFreeExcessListId();
	int* voxelAllocationList = scene->localVBA.GetAllocationList();
	int* excessAllocationList = scene->index.GetExcessAllocationList();
	ITMHashEntry* hashTable = scene->index.GetEntries();

	for (int hash = 0; hash < entryCount; hash++) {
		unsigned char entryAllocType = entryAllocationTypes[hash];
		switch (entryAllocType) {
			case ITMLib::NEEDS_ALLOCATION_IN_ORDERED_LIST:

				if (lastFreeVoxelBlockId >= 0) //there is room in the voxel block array
				{
					ITMHashEntry hashEntry;
					hashEntry.pos = allocationBlockCoordinates[hash];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
					hashTable[hash] = hashEntry;
					lastFreeVoxelBlockId--;
				}

				break;
			case NEEDS_ALLOCATION_IN_EXCESS_LIST:

				if (lastFreeVoxelBlockId >= 0 &&
				    lastFreeExcessListId >= 0) //there is room in the voxel block array and excess list
				{
					ITMHashEntry hashEntry;
					hashEntry.pos = allocationBlockCoordinates[hash];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
					int exlOffset = excessAllocationList[lastFreeExcessListId];
					hashTable[hash].offset = exlOffset + 1; //connect to child
					hashTable[SDF_BUCKET_NUM + exlOffset] = hashEntry; //add child to the excess list
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


template<typename TVoxel, typename TIndex>
inline
void AllocateHashEntriesUsingLists_SetVisibility_CPU(ITMScene<TVoxel, TIndex>* scene, uchar* entryAllocationTypes,
                                       Vector3s* allocationBlockCoordinates, uchar* entriesVisibleType) {
	int entryCount = TIndex::noTotalEntries;
	int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = scene->index.GetLastFreeExcessListId();
	int* voxelAllocationList = scene->localVBA.GetAllocationList();
	int* excessAllocationList = scene->index.GetExcessAllocationList();
	ITMHashEntry* hashTable = scene->index.GetEntries();

	for (int hash = 0; hash < entryCount; hash++) {
		if(hash == 0 || hash == 1924){
			int i = 42;
		}
		unsigned char entryAllocType = entryAllocationTypes[hash];
		switch (entryAllocType) {
			case ITMLib::NEEDS_ALLOCATION_IN_ORDERED_LIST:

				if (lastFreeVoxelBlockId >= 0) //there is room in the voxel block array
				{
					ITMHashEntry hashEntry;
					hashEntry.pos = allocationBlockCoordinates[hash];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
					hashTable[hash] = hashEntry;
					lastFreeVoxelBlockId--;
				}else{
					entriesVisibleType[hash] = 0;
				}

				break;
			case NEEDS_ALLOCATION_IN_EXCESS_LIST:

				if (lastFreeVoxelBlockId >= 0 &&
				    lastFreeExcessListId >= 0) //there is room in the voxel block array and excess list
				{
					ITMHashEntry hashEntry;
					hashEntry.pos = allocationBlockCoordinates[hash];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
					int exlOffset = excessAllocationList[lastFreeExcessListId];
					hashTable[hash].offset = exlOffset + 1; //connect to child
					hashTable[SDF_BUCKET_NUM + exlOffset] = hashEntry; //add child to the excess list
					entriesVisibleType[SDF_BUCKET_NUM + exlOffset] = 1; //make child visible and in memory
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

// endregion ===========================================================================================================
// region ======================================== HELPER RANGE COMPUTATION / CHECK ROUTINES ===========================
// =====================================================================================================================
inline
void MinMaxFromExtrema(Vector3i& minPoint, Vector3i& maxPoint, const Vector3i& extremum1, const Vector3i& extremum2) {
	// ** set min/max **
	for (int iValue = 0; iValue < 3; iValue++) {
		if (extremum1.values[iValue] > extremum2.values[iValue]) {
			minPoint.values[iValue] = extremum2.values[iValue];
			maxPoint.values[iValue] = extremum1.values[iValue];
		} else {
			minPoint.values[iValue] = extremum1.values[iValue];
			maxPoint.values[iValue] = extremum2.values[iValue];
		}
	}
}

inline
bool
IsHashBlockFullyInRange(const Vector3i& hashBlockPositionVoxels, const Vector3i& minPoint, const Vector3i& maxPoint) {
	return hashBlockPositionVoxels.x + SDF_BLOCK_SIZE - 1 <= maxPoint.x && hashBlockPositionVoxels.x >= minPoint.x &&
	       hashBlockPositionVoxels.y + SDF_BLOCK_SIZE - 1 <= maxPoint.y && hashBlockPositionVoxels.y >= minPoint.y &&
	       hashBlockPositionVoxels.z + SDF_BLOCK_SIZE - 1 <= maxPoint.z && hashBlockPositionVoxels.z >= minPoint.z;
}

inline
bool IsHashBlockPartiallyInRange(const Vector3i& hashBlockPositionVoxels, const Vector3i& minPoint,
                                 const Vector3i& maxPoint) {
	//@formatter:off
	return ((hashBlockPositionVoxels.x + SDF_BLOCK_SIZE - 1 >= maxPoint.x && hashBlockPositionVoxels.x <= maxPoint.x)
	     || (hashBlockPositionVoxels.x + SDF_BLOCK_SIZE - 1 >= minPoint.x && hashBlockPositionVoxels.x <= minPoint.x)) &&
	       ((hashBlockPositionVoxels.y + SDF_BLOCK_SIZE - 1 >= maxPoint.y && hashBlockPositionVoxels.y <= maxPoint.y)
	     || (hashBlockPositionVoxels.y + SDF_BLOCK_SIZE - 1 >= minPoint.y && hashBlockPositionVoxels.y <= minPoint.y)) &&
	       ((hashBlockPositionVoxels.z + SDF_BLOCK_SIZE - 1 >= maxPoint.z && hashBlockPositionVoxels.z <= maxPoint.z)
	     || (hashBlockPositionVoxels.z + SDF_BLOCK_SIZE - 1 >= minPoint.z && hashBlockPositionVoxels.z <= minPoint.z));
	//@formatter:on
}

inline
bool
ComputeCopyRanges(int& xRangeStart, int& xRangeEnd, int& yRangeStart, int& yRangeEnd, int& zRangeStart, int& zRangeEnd,
                  const Vector3i& hashBlockPositionVoxels, const Vector3i& minPoint, const Vector3i& maxPoint) {
	zRangeStart = std::max(0, minPoint.z - hashBlockPositionVoxels.z);
	zRangeEnd = std::min(SDF_BLOCK_SIZE, maxPoint.z - hashBlockPositionVoxels.z + 1);
	yRangeStart = std::max(0, minPoint.y - hashBlockPositionVoxels.y);
	yRangeEnd = std::min(SDF_BLOCK_SIZE, maxPoint.y - hashBlockPositionVoxels.y + 1);
	xRangeStart = std::max(0, minPoint.x - hashBlockPositionVoxels.x);
	xRangeEnd = std::min(SDF_BLOCK_SIZE, maxPoint.x - hashBlockPositionVoxels.x + 1);
}

// endregion ===========================================================================================================




template<typename TVoxelSource, typename TVoxelDesitnation, typename TIndex >
void CopySceneSDFandFlagsWithOffset_CPU(ITMScene<TVoxelDesitnation, TIndex>* destination,
                                        ITMScene<TVoxelSource, TIndex>* source,
                                        Vector3i offset);

template<class TVoxel, class TIndex >
bool CopySceneSlice_CPU(ITMScene<TVoxel, TIndex>* destination,
                        ITMScene<TVoxel, TIndex>* source,
                        Vector3i minPoint, Vector3i maxPoint);


template<class TVoxel, class TIndex>
TVoxel ReadVoxel(ITMScene<TVoxel, TIndex>& scene, Vector3i at);

int FindHashBlock(const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData)* voxelIndex, const THREADPTR(Vector3s)& at);

void GetVoxelHashLocals(int& vmIndex, int& locId, int& xInBlock, int& yInBlock, int& zInBlock,
                        const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData)* hashEntries,
                        ITMLib::ITMVoxelBlockHash::IndexCache& cache,
                        const CONSTPTR(Vector3i)& at);


template<class TVoxel, class TIndex>
bool SetVoxel_CPU(ITMScene <TVoxel, TIndex>* scene, Vector3i at, TVoxel voxel);


template<class TVoxel, class TIndex>
void OffsetWarps(ITMScene<TVoxel, TIndex>& scene, Vector3f offset);

template<class TVoxel, class TIndex>
TVoxel ReadVoxel(ITMScene<TVoxel, TIndex>& scene, Vector3i at);


template<class TVoxel, class TIndex>
bool SetVoxel_CPU(ITMScene <TVoxel, TIndex>* scene, Vector3i at, TVoxel voxel);

}//namespace ITMLib