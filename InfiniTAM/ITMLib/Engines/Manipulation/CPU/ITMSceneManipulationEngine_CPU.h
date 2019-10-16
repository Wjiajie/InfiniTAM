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
#include "../Interface/ITMSceneManipulationEngine.h"


//TODO: Make GPU versions -Greg (GitHub: Algomorph)

namespace ITMLib {

// region ==================== SCENE MANIPULATION ENGINE ===============================================================

template<typename TVoxel, typename TIndex>
class ITMSceneManipulationEngine_CPU : public ITMSceneManipulationEngine<TVoxel, TIndex> {
};


template<typename TVoxel>
class ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>
		: public ITMSceneManipulationEngine<TVoxel, ITMVoxelBlockHash> {
public:
	//can be used as a singleton, but doesn't HAVE TO be
	static ITMSceneManipulationEngine_CPU& Inst() {
		static ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash> instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	void ResetScene(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene) override;
	bool SetVoxel(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, Vector3i at, TVoxel voxel) override;
	bool SetVoxelNoAllocation(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, Vector3i at, TVoxel voxel);
	TVoxel ReadVoxel(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, Vector3i at) override;
	TVoxel ReadVoxel(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, Vector3i at,
	                 ITMVoxelBlockHash::IndexCache& cache) override;
	void OffsetWarps(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, Vector3f offset) override;
	bool
	CopySceneSlice(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* destination,
	               ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* source,
	               Vector6i bounds, const Vector3i& offset = Vector3i(0)) override;
	bool CopyScene(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* destination,
	               ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* source,
	               const Vector3i& offset = Vector3i(0)) override;
	virtual ~ITMSceneManipulationEngine_CPU() = default;
};

template<typename TVoxel>
class ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray> :
		public ITMSceneManipulationEngine<TVoxel, ITMPlainVoxelArray> {
public:
	// can use this as singleton, but don't have to
	static ITMSceneManipulationEngine_CPU& Inst() {
		static ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray> instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	void ResetScene(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene) override;
	bool SetVoxel(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, Vector3i at, TVoxel voxel) override;
	TVoxel ReadVoxel(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, Vector3i at) override;
	TVoxel ReadVoxel(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, Vector3i at,
	                 ITMPlainVoxelArray::IndexCache& cache) override;
	bool IsPointInBounds(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, const Vector3i& at);
	void OffsetWarps(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, Vector3f offset) override;
	bool CopySceneSlice(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* destination,
	                    ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* source,
	                    Vector6i bounds, const Vector3i& offset = Vector3i(0)) override;
	bool CopyScene(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* destination,
	               ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* source,
	               const Vector3i& offset = Vector3i(0)) override;

};

// endregion ================= SCENE MANIPULATION ENGINE ===============================================================

// region =================================== HELPER ROUTINES FOR SCENE OPTIMIZATION PREP ==============================
//======================================================================================================================



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

typedef ITMSceneManipulationEngine_CPU<ITMVoxel, ITMPlainVoxelArray> ManipulationEngine_CPU_PVA_Voxel;
typedef ITMSceneManipulationEngine_CPU<ITMVoxel, ITMVoxelBlockHash> ManipulationEngine_CPU_VBH_Voxel;
typedef ITMSceneManipulationEngine_CPU<ITMWarp, ITMPlainVoxelArray> ManipulationEngine_CPU_PVA_Warp;
typedef ITMSceneManipulationEngine_CPU<ITMWarp, ITMVoxelBlockHash> ManipulationEngine_CPU_VBH_Warp;

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


}//namespace ITMLib