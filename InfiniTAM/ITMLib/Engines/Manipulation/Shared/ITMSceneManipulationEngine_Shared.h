//  ================================================================
//  Created by Gregory Kramida on 9/25/19.
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


#include "../../../Utils/ITMMath.h"
#include "../../../Objects/Scene/ITMVoxelBlockHash.h"
#include "../../../Objects/Scene/ITMRepresentationAccess.h"


struct AllocationTempData {
	int noAllocatedVoxelEntries;
	int noAllocatedExcessEntries;
	int noBlocksToCopy;
	bool success;
};

struct CopyHashBlockPairInfo {
	int sourceHash;
	int destinationHash;
	bool fullyInBounds;
};

struct OffsetCopyHashBlockInfo{
	int destinationHash;
	bool fullyInBounds;
};

_CPU_AND_GPU_CODE_
inline
bool FindOrAllocateHashEntry(const Vector3s& hashEntryPosition, ITMHashEntry* hashTable, ITMHashEntry*& resultEntry,
                             int& lastFreeVoxelBlockId, int& lastFreeExcessListId, const int* voxelAllocationList,
                             const int* excessAllocationList, int& hash) {
	hash = hashIndex(hashEntryPosition);
	ITMHashEntry hashEntry = hashTable[hash];
	if (!IS_EQUAL3(hashEntry.pos, hashEntryPosition) || hashEntry.ptr < -1) {
		bool isExcess = false;
		//search excess list only if there is no room in ordered part
		if (hashEntry.ptr >= -1) {
			while (hashEntry.offset >= 1) {
				hash = SDF_BUCKET_NUM + hashEntry.offset - 1;
				hashEntry = hashTable[hash];
				if (IS_EQUAL3(hashEntry.pos, hashEntryPosition) && hashEntry.ptr >= -1) {
					resultEntry = &hashTable[hash];
					return true;
				}
			}
			isExcess = true;

		}
		//still not found, allocate
		if (isExcess && lastFreeVoxelBlockId >= 0 && lastFreeExcessListId >= 0) {
			//there is room in the voxel block array and excess list
			ITMHashEntry newHashEntry;
			newHashEntry.pos = hashEntryPosition;
			newHashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
			newHashEntry.offset = 0;
			int exlOffset = excessAllocationList[lastFreeExcessListId];
			hashTable[hash].offset = exlOffset + 1; //connect to child
			hashTable[SDF_BUCKET_NUM +
			          exlOffset] = newHashEntry; //add child to the excess list
			resultEntry = &hashTable[SDF_BUCKET_NUM +
			                         exlOffset];
			lastFreeVoxelBlockId--;
			lastFreeExcessListId--;
			return true;
		} else if (lastFreeVoxelBlockId >= 0) {
			//there is room in the voxel block array
			ITMHashEntry newHashEntry;
			newHashEntry.pos = hashEntryPosition;
			newHashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
			newHashEntry.offset = 0;
			hashTable[hash] = newHashEntry;
			resultEntry = &hashTable[hash];
			lastFreeVoxelBlockId--;
			return true;
		} else {
			return false;
		}
	} else {
		//HashEntry already exists, return the pointer to it
		resultEntry = &hashTable[hash];
		return true;
	}
}

_CPU_AND_GPU_CODE_
inline
bool
IsHashBlockFullyInRange(const Vector3i& hashBlockPositionVoxels, const Vector6i& bounds) {
	return hashBlockPositionVoxels.x + SDF_BLOCK_SIZE - 1 <= bounds.max_x &&
	       hashBlockPositionVoxels.x >= bounds.min_x &&
	       hashBlockPositionVoxels.y + SDF_BLOCK_SIZE - 1 <= bounds.max_y &&
	       hashBlockPositionVoxels.y >= bounds.min_y &&
	       hashBlockPositionVoxels.z + SDF_BLOCK_SIZE - 1 <= bounds.max_z && hashBlockPositionVoxels.z >= bounds.min_z;
}

_CPU_AND_GPU_CODE_
inline
bool IsHashBlockPartiallyInRange(const Vector3i& hashBlockPositionVoxels, const Vector6i& bounds) {
	//@formatter:off
	return ((hashBlockPositionVoxels.x + SDF_BLOCK_SIZE - 1 >= bounds.max_x && hashBlockPositionVoxels.x <= bounds.max_x)
	     || (hashBlockPositionVoxels.x + SDF_BLOCK_SIZE - 1 >= bounds.min_x && hashBlockPositionVoxels.x <= bounds.min_x)) &&
	       ((hashBlockPositionVoxels.y + SDF_BLOCK_SIZE - 1 >= bounds.max_y && hashBlockPositionVoxels.y <= bounds.max_y)
	     || (hashBlockPositionVoxels.y + SDF_BLOCK_SIZE - 1 >= bounds.min_y && hashBlockPositionVoxels.y <= bounds.min_y)) &&
	       ((hashBlockPositionVoxels.z + SDF_BLOCK_SIZE - 1 >= bounds.max_z && hashBlockPositionVoxels.z <= bounds.max_z)
	     || (hashBlockPositionVoxels.z + SDF_BLOCK_SIZE - 1 >= bounds.min_z && hashBlockPositionVoxels.z <= bounds.min_z));
	//@formatter:on
}

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
inline bool MarkAsNeedingAllocationIfNotFound(uchar* entryAllocationTypes, Vector3s* hashBlockCoordinates, int& hashIdx,
                                              const CONSTPTR(Vector3s)& desiredHashBlockPosition,
                                              const CONSTPTR(ITMHashEntry)* hashTable, bool& collisionDetected) {

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
			if (entryAllocationTypes[hashIdx] != ITMLib::NEEDS_NO_CHANGE) {
				collisionDetected = true;
				return false;
			} else {
				entryAllocationTypes[hashIdx] = ITMLib::NEEDS_ALLOCATION_IN_EXCESS_LIST;
				hashBlockCoordinates[hashIdx] = desiredHashBlockPosition;
				return true;
			}

		}
		if (entryAllocationTypes[hashIdx] != ITMLib::NEEDS_NO_CHANGE) {
			collisionDetected = true;
			return false;
		} else {
			entryAllocationTypes[hashIdx] = ITMLib::NEEDS_ALLOCATION_IN_ORDERED_LIST;
			hashBlockCoordinates[hashIdx] = desiredHashBlockPosition;
			return true;
		}

	}
	// already have hash block, no allocation needed
	return false;

};

 /**
  * \brief find the hash block at the specified spatial coordinates (in blocks, not voxels!) and return its hash
  * \param voxelIndex
  * \param at
  * \return
  */
_CPU_AND_GPU_CODE_
inline int
FindHashBlock(const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData)* voxelIndex, const THREADPTR(Vector3s)& at) {
	int hash = hashIndex(at);
	while (true) {
		ITMHashEntry hashEntry = voxelIndex[hash];

		if (IS_EQUAL3(hashEntry.pos, at) && hashEntry.ptr >= 0) {
			return hash;
		}

		if (hashEntry.offset < 1) break;
		hash = SDF_BUCKET_NUM + hashEntry.offset - 1;
	}
	return -1;
}

_CPU_AND_GPU_CODE_
inline void 
ComputeVoxelBlockOffsetRange(const CONSTPTR(Vector3i)& offset,
                             THREADPTR(Vector6i)& offsetRange){
#define  ITM_CMP(a,b)    (((a) > (b)) - ((a) < (b)))
#define  ITM_SIGN(a)     ITM_CMP((a),0)

	int xA = offset.x / SDF_BLOCK_SIZE;
	int xB = xA + ITM_SIGN(offset.x % SDF_BLOCK_SIZE);
	int yA = offset.y / SDF_BLOCK_SIZE;
	int yB = yA + ITM_SIGN(offset.y % SDF_BLOCK_SIZE);
	int zA = offset.z / SDF_BLOCK_SIZE;
	int zB = zA + ITM_SIGN(offset.z % SDF_BLOCK_SIZE);
	int tmp;
	if (xA > xB) {
		tmp = xA;
		xA = xB;
		xB = tmp;
	}
	if (yA > yB) {
		tmp = yA;
		yA = yB;
		yB = tmp;
	}
	if (zA > zB) {
		tmp = zA;
		zA = zB;
		zB = tmp;
	}
	offsetRange.min_x = xA;
	offsetRange.min_y = yA;
	offsetRange.min_z = zA;
	offsetRange.max_x = xB + 1;
	offsetRange.max_y = yB + 1;
	offsetRange.max_z = zB + 1;
#undef ITM_CMP
#undef ITM_SIGN
}

/**
 * \brief Determine if hash blocks spanning any part of the volume offset from the given block by a given vector are allocated
 * \param targetIndex Target index where to check the blocks being allocated
 * \param blockCoord coordinate of the given block (before offset)
 * \param offset offset, in voxels (not blocks!)
 * \return true if any of the said blocks are allocated, false otherwise
 */
_CPU_AND_GPU_CODE_
inline bool
HashBlockAllocatedAtOffset(const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData)* targetIndex,
                           const THREADPTR(Vector3s)& blockCoord,
                           const CONSTPTR(Vector3i)& offset) {
	Vector6i blockRange;
	ComputeVoxelBlockOffsetRange(offset, blockRange);

	bool allocated = false;
	for (int z = blockCoord.z + blockRange.min_x; z < blockCoord.z + blockRange.max_x; z++){
		for (int y = blockCoord.y + blockRange.min_y; y < blockCoord.y + blockRange.max_y; y++){
			for (int x = blockCoord.x + blockRange.min_z; x < blockCoord.x + blockRange.max_z; x++){
				if(FindHashBlock(targetIndex, Vector3s(x,y,z)) != -1){
					allocated = true;
				}
			}
		}
	}
	return allocated;
}

/**
 * \brief Same as above, except with block ranges spanning the offset volume of the given block precomputed and provided
 * \param targetIndex Target index where to check the blocks being allocated
 * \param blockCoord coordinate of the given block (before offset)
 * \param offsetBlockRange block range to check (in blocks!) - each value of the range is an offset from block coord
 * \return  true if any of the blocks in blockCoord + [values within offset range] are allocated, false otherwise
 */
_CPU_AND_GPU_CODE_
inline bool
HashBlockAllocatedAtOffset(const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData)* targetIndex,
                           const THREADPTR(Vector3s)& blockCoord,
                           const CONSTPTR(Vector6i)& offsetBlockRange) {

	bool allocated = false;
	for (int z = blockCoord.z + offsetBlockRange.min_x; z < blockCoord.z + offsetBlockRange.max_x; z++){
		for (int y = blockCoord.y + offsetBlockRange.min_y; y < blockCoord.y + offsetBlockRange.max_y; y++){
			for (int x = blockCoord.x + offsetBlockRange.min_z; x < blockCoord.x + offsetBlockRange.max_z; x++){
				if(FindHashBlock(targetIndex, Vector3s(x,y,z)) != -1){
					allocated = true;
				}
			}
		}
	}
	return allocated;
}