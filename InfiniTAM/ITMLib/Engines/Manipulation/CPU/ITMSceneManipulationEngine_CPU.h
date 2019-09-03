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

#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "../../../ITMLibDefines.h"
#include "../../../Utils/ITMHashBlockProperties.h"
#include "../../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../../Utils/ITMPrintHelpers.h"

//TODO: Make GPU versions -Greg (GitHub: Algomorph)

namespace ITMLib {

// region ==================== SCENE MANIPULATION ENGINE ===============================================================

template<typename TVoxel, typename TIndex>
class ITMSceneManipulationEngine_CPU {

};


template<typename TVoxel>
class ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash> {
public:
	/**
	 * \brief Clear out scene and reset the index
	 * \param scene
	 */
	static void ResetScene(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene);
	static bool SetVoxel(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, Vector3i at, TVoxel voxel);
	static bool SetVoxelNoAllocation(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, Vector3i at, TVoxel voxel);
	static TVoxel ReadVoxel(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, Vector3i at);
	static TVoxel ReadVoxel(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, Vector3i at, ITMVoxelBlockHash::IndexCache& cache);


	/**
	 * \brief offset warps by a fixed amount in each direction
	 * \param scene the scene to modify
	 * \param offset the offset vector to use
	 */
	static void OffsetWarps(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, Vector3f offset);
	/**
	 * \brief Copies the slice (box-like window) specified by points extremum1 and extremum2 from the source scene into a
	 * destination scene. Clears the destination scene before copying.
	 * \tparam TVoxel type of voxel
	 * \tparam TIndex type of voxel index
	 * \param destination destination voxel grid (can be uninitialized)
	 * \param source source voxel grid
	 * \param bounds minimum point in the desired slice (inclusive), i.e. minimum x, y, and z coordinates
	 * \param maxPoint maximum point in the desired slice (inclusive), i.e. maximum x, y, and z coordinates
	 * \return true on success (destination scene contains the slice), false on failure (there are no allocated hash blocks
	 */
	static bool
	CopySceneSlice(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* destination, ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* source,
	               Vector6i bounds, const Vector3i& offset = Vector3i(0));
	static bool CopyScene(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* destination,
	                      ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* source,
	                      const Vector3i& offset = Vector3i(0));

};

template<typename TVoxel>
class ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray> {
public:
	static void ResetScene(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene);
	static bool SetVoxel(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, Vector3i at, TVoxel voxel);
	static TVoxel ReadVoxel(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, Vector3i at);
	static TVoxel
	ReadVoxel(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, Vector3i at, ITMPlainVoxelArray::IndexCache& cache);
	static bool IsPointInBounds(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, const Vector3i& at);
	static void OffsetWarps(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, Vector3f offset);
	/**
	 * \brief Copies the slice (box-like window) specified by points extremum1 and extremum2 from the source scene into a
	 * destination scene. Clears the destination scene before copying.
	 * \tparam TVoxel type of voxel
	 * \tparam TIndex type of voxel index
	 * \param destination destination voxel grid (must be initialized)
	 * \param source source voxel grid
	 * \param bounds a vector representing the minimum point in the desired slice (inclusive), i.e. minimum x, y,
	 * and z coordinates, and maximum point in the desired slice (inclusive), i.e. maximum x, y, and z coordinates
	 * \param offset if non-zero, the slice will be copied at the specified offset in the space of the destination scene
	 * from the source slice in the source scene.
	 * \return true on success (destination scene contains the slice), false on failure (there are no allocated hash blocks
	 */
	static bool CopySceneSlice(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* destination,
	                           ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* source,
	                           Vector6i bounds, const Vector3i& offset = Vector3i(0));

	/**
	 * \brief Copies the whole scene
	 * \param destination destination voxel grid (may be uninitialized, will be erased)
	 * \param source source voxel grid
	 * \param offset copies voxels at the specified offset -- leaves the leftover margin as default values, the source
	 * values that go out-of-range due to the offset will be omitted
	 * \return true on success, false on failure
	 */
	static bool CopyScene(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* destination,
	                      ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* source,
	                      const Vector3i& offset = Vector3i(0));

};

// endregion ================= SCENE MANIPULATION ENGINE ===============================================================

// region ==================================== GENERAL HASH MANAGEMENT =================================================

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


template<typename TVoxel, typename TIndex>
inline
void AllocateHashEntriesUsingLists_CPU(ITMVoxelVolume<TVoxel, TIndex>* scene, uchar* entryAllocationTypes,
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
void AllocateHashEntriesUsingLists_SetVisibility_CPU(ITMVoxelVolume<TVoxel, TIndex>* scene, uchar* entryAllocationTypes,
                                                     Vector3s* allocationBlockCoordinates, uchar* entriesVisibleType) {
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
				} else {
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
void BoundsFromExtrema(Vector6i& bounds, const Vector3i& extremum1, const Vector3i& extremum2) {
	// ** set min/max **
	if (extremum1.x > extremum2.x) {
		bounds.min_x = extremum2.x;
		bounds.max_x = extremum1.x;
	} else {
		bounds.min_x = extremum1.x;
		bounds.max_x = extremum2.x;
	}
	if (extremum1.y > extremum2.y) {
		bounds.min_y = extremum2.y;
		bounds.max_y = extremum1.y;
	} else {
		bounds.min_y = extremum1.y;
		bounds.max_y = extremum2.y;
	}
	if (extremum1.z > extremum2.z) {
		bounds.min_z = extremum2.z;
		bounds.max_z = extremum1.z;
	} else {
		bounds.min_z = extremum1.z;
		bounds.max_z = extremum2.z;
	}
}

inline
bool
IsHashBlockFullyInRange(const Vector3i& hashBlockPositionVoxels, const Vector6i& bounds) {
	return hashBlockPositionVoxels.x + SDF_BLOCK_SIZE - 1 <= bounds.max_x &&
	       hashBlockPositionVoxels.x >= bounds.min_x &&
	       hashBlockPositionVoxels.y + SDF_BLOCK_SIZE - 1 <= bounds.max_y &&
	       hashBlockPositionVoxels.y >= bounds.min_y &&
	       hashBlockPositionVoxels.z + SDF_BLOCK_SIZE - 1 <= bounds.max_z && hashBlockPositionVoxels.z >= bounds.min_z;
}

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

inline
void
ComputeCopyRanges(int& xRangeStart, int& xRangeEnd, int& yRangeStart, int& yRangeEnd, int& zRangeStart, int& zRangeEnd,
                  const Vector3i& hashBlockPositionVoxels, const Vector6i& bounds) {
	zRangeStart = MAX(0, bounds.min_z - hashBlockPositionVoxels.z);
	zRangeEnd = MIN(SDF_BLOCK_SIZE, bounds.max_z - hashBlockPositionVoxels.z + 1);
	yRangeStart = MAX(0, bounds.min_y - hashBlockPositionVoxels.y);
	yRangeEnd = MIN(SDF_BLOCK_SIZE, bounds.max_y - hashBlockPositionVoxels.y + 1);
	xRangeStart = MAX(0, bounds.min_x - hashBlockPositionVoxels.x);
	xRangeEnd = MIN(SDF_BLOCK_SIZE, bounds.max_x - hashBlockPositionVoxels.x + 1);
}

// endregion ===========================================================================================================

template<typename TVoxelSource, typename TVoxelDestination, typename TIndex>
class ITMTwoSceneManipulationEngine_CPU {
};

template<typename TVoxelSource, typename TVoxelDestination>
class ITMTwoSceneManipulationEngine_CPU<TVoxelSource, TVoxelDestination, ITMVoxelBlockHash> {
public:
	/**
	 * \brief Copies all the sdf & flag values from the source scene to the destination scene with the desired offset
	 * \param destination destination scene
	 * \param source source scene
	 * \param offset offset to use when copying the values
	 */
	static void CopySceneSDFandFlagsWithOffset_CPU(ITMVoxelVolume<TVoxelDestination, ITMVoxelBlockHash>* destination,
	                                               ITMVoxelVolume<TVoxelSource, ITMVoxelBlockHash>* source,
	                                               Vector3i offset);


};


template<typename TVoxelSource, typename TVoxelDestination>
class ITMTwoSceneManipulationEngine_CPU<TVoxelSource, TVoxelDestination, ITMPlainVoxelArray> {
public:
	/**
	 * \brief Copies all the sdf & flag values from the source scene to the destination scene with the desired offset
	 * \param destination destination scene
	 * \param source source scene
	 * \param offset offset to use when copying the values
	 */
	static void CopySceneSDFandFlagsWithOffset_CPU(ITMVoxelVolume<TVoxelDestination, ITMPlainVoxelArray>* destination,
	                                               ITMVoxelVolume<TVoxelSource, ITMPlainVoxelArray>* source,
	                                               Vector3i offset);
};


int FindHashBlock(const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData)* voxelIndex, const THREADPTR(Vector3s)& at);

void GetVoxelHashLocals(int& vmIndex, int& locId, int& xInBlock, int& yInBlock, int& zInBlock,
                        const CONSTPTR(ITMLib::ITMPlainVoxelArray::IndexData)* indexData,
                        ITMLib::ITMPlainVoxelArray::IndexCache& cache,
                        const CONSTPTR(Vector3i)& at);
void GetVoxelHashLocals(int& vmIndex, int& locId, int& xInBlock, int& yInBlock, int& zInBlock,
                        const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData)* hashEntries,
                        ITMLib::ITMVoxelBlockHash::IndexCache& cache,
                        const CONSTPTR(Vector3i)& at);

}//namespace ITMLib