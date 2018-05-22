//  ================================================================
//  Created by Gregory Kramida on 5/22/18.
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

#include "ITMScene.h"
#include "ITMSceneManipulation.h"

namespace ITMLib {

// region ================================ DYNAMIC SINGLE-SCENE TRAVERSAL ==============================================

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
void VoxelPositionTraversal_CPU(ITMScene<TVoxel, TIndex>* scene, TFunctor& functor) {
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


template<typename TFunctor, typename TVoxel, typename TIndex>
inline
void VoxelAndHashBlockPositionTraversal_CPU(ITMScene<TVoxel, TIndex>& scene, TFunctor& functor) {
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
					functor(voxel, voxelPosition, currentHashEntry.pos);
				}
			}
		}
	}
};

// endregion ===========================================================================================================

// region ================================ STATIC SINGLE-SCENE TRAVERSAL ===============================================

template<typename TStaticFunctor, typename TVoxel, typename TIndex>
inline void StaticVoxelTraversal_CPU(ITMScene<TVoxel, TIndex>* scene) {
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
					TVoxel& voxel = localVoxelBlock[locId];
					TStaticFunctor::run(voxel);
				}
			}
		}
	}
};

template<typename TStaticFunctor, typename TVoxel, typename TIndex>
inline void StaticVoxelPositionTraversal_CPU(ITMScene<TVoxel, TIndex>* scene) {
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
// endregion
// region ================================ DYNAMIC TWO-SCENE TRAVERSAL =================================================

template<typename TVoxelSecondary, typename TIndex>
struct AllocateHashOnMissFunctor {
	static void run(ITMScene<TVoxelSecondary, TIndex>* secondaryScene, uchar* entriesAllocType,
	                ITMHashEntry& currentCanonicalHashEntry, const Vector3s& blockPosition, int& secondaryHash,
	                const bool& report) {
		ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();
		int lastFreeVoxelBlockId = secondaryScene->localVBA.lastFreeBlockId;
		int lastFreeExcessListId = secondaryScene->index.GetLastFreeExcessListId();
		int* voxelAllocationList = secondaryScene->localVBA.GetAllocationList();
		int* excessAllocationList = secondaryScene->index.GetExcessAllocationList();
		ITMHashEntry* newEntry;
		if (AllocateHashEntry_CPU(blockPosition, secondaryHashTable, newEntry, lastFreeExcessListId,
		                          lastFreeExcessListId, voxelAllocationList, excessAllocationList,
		                          secondaryHash)) {
			currentCanonicalHashEntry = *newEntry;

			entriesAllocType[secondaryHash] = ITMLib::BOUNDARY;
			if (report) {
				std::cout << bright_red << "Allocating hash entry in secondary scene during dual traversal at: "
				          << blockPosition << reset << std::endl;
			}
		} else {
			DIEWITHEXCEPTION_REPORTLOCATION("Could not allocate hash entry...");
		}
		secondaryScene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
		secondaryScene->index.SetLastFreeExcessListId(lastFreeExcessListId);
	}
};

template<typename TVoxelSecondary, typename TIndex>
struct DieOnMissFunctor {
	static void run(ITMScene<TVoxelSecondary, TIndex>* secondaryScene, uchar* entriesAllocType,
	                ITMHashEntry& currentCanonicalHashEntry, const Vector3s& blockPosition, int& secondaryHash,
	                const bool& report) {
		std::stringstream stream;
		stream << "Could not find corresponding secondary scene block at postion " << blockPosition
		       << ". " << __FILE__ << ": " << __LINE__;
		DIEWITHEXCEPTION(stream.str());
	}
};


template<typename TFunctor, typename TBehaviorOnMissFunctor, typename TVoxelPrimary, typename TVoxelSecondary, typename TIndex>
inline void DualVoxelPositionTraversal_CPU(
		ITMScene<TVoxelPrimary, TIndex>* primaryScene,
		ITMScene<TVoxelSecondary, TIndex>* secondaryScene,
		unsigned char* entriesAllocType,
		TFunctor& functor, bool reportMisses = true) {

// *** traversal vars
	TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();
	typename TIndex::IndexCache secondaryCache;


	TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();
	int noTotalEntries = primaryScene->index.noTotalEntries;
	typename TIndex::IndexCache primaryCache;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int hash = 0; hash < noTotalEntries; hash++) {
		ITMHashEntry& currentLiveHashEntry = primaryHashTable[hash];
		if (currentLiveHashEntry.ptr < 0) continue;
		ITMHashEntry& currentCanonicalHashEntry = secondaryHashTable[hash];

		// the rare case where we have different positions for primary & secondary voxel block with the same index:
		// we have a hash bucket miss, find the secondary voxel with the matching coordinates
		if (currentCanonicalHashEntry.pos != currentLiveHashEntry.pos) {
			int secondaryHash;
			if (!FindHashAtPosition(secondaryHash, currentLiveHashEntry.pos, secondaryHashTable)) {
				TBehaviorOnMissFunctor::run(secondaryScene, entriesAllocType, currentCanonicalHashEntry,
				                            currentLiveHashEntry.pos, secondaryHash, reportMisses);
			} else {
				currentCanonicalHashEntry = secondaryHashTable[secondaryHash];
			}
		}
		// position of the current entry in 3D space in voxel units
		Vector3i hashBlockPosition = currentLiveHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		TVoxelPrimary* localLiveVoxelBlock = &(primaryVoxels[currentLiveHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		TVoxelSecondary* localCanonicalVoxelBlock = &(secondaryVoxels[currentCanonicalHashEntry.ptr *
		                                                              (SDF_BLOCK_SIZE3)]);
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					Vector3i voxelPosition = hashBlockPosition + Vector3i(x, y, z);
					TVoxelPrimary& primaryVoxel = localLiveVoxelBlock[locId];
					TVoxelSecondary& secondaryVoxel = localCanonicalVoxelBlock[locId];
					functor(primaryVoxel, secondaryVoxel, voxelPosition);
				}
			}
		}
	}
};

template<typename TFunctor, typename TBehaviorOnMissFunctor, typename TVoxelPrimary, typename TVoxelSecondary, typename TIndex>
inline void DualVoxelPositionTraversal_CPU_SingleThreaded(
		ITMScene<TVoxelPrimary, TIndex>* primaryScene,
		ITMScene<TVoxelSecondary, TIndex>* secondaryScene,
		unsigned char* entriesAllocType,
		TFunctor& functor, bool reportMisses = true) {

// *** traversal vars
	TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();
	typename TIndex::IndexCache secondaryCache;


	TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();
	int noTotalEntries = primaryScene->index.noTotalEntries;
	typename TIndex::IndexCache primaryCache;

	for (int hash = 0; hash < noTotalEntries; hash++) {
		ITMHashEntry& currentLiveHashEntry = primaryHashTable[hash];
		if (currentLiveHashEntry.ptr < 0) continue;
		ITMHashEntry& currentCanonicalHashEntry = secondaryHashTable[hash];

		// the rare case where we have different positions for primary & secondary voxel block with the same index:
		// we have a hash bucket miss, find the secondary voxel with the matching coordinates
		if (currentCanonicalHashEntry.pos != currentLiveHashEntry.pos) {
			int secondaryHash;
			if (!FindHashAtPosition(secondaryHash, currentLiveHashEntry.pos, secondaryHashTable)) {
				TBehaviorOnMissFunctor::run(secondaryScene, entriesAllocType, currentCanonicalHashEntry,
				                            currentLiveHashEntry.pos, secondaryHash, reportMisses);
			} else {
				currentCanonicalHashEntry = secondaryHashTable[secondaryHash];
			}
		}
		// position of the current entry in 3D space in voxel units
		Vector3i hashBlockPosition = currentLiveHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		TVoxelPrimary* localLiveVoxelBlock = &(primaryVoxels[currentLiveHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		TVoxelSecondary* localCanonicalVoxelBlock = &(secondaryVoxels[currentCanonicalHashEntry.ptr *
		                                                              (SDF_BLOCK_SIZE3)]);
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					Vector3i voxelPosition = hashBlockPosition + Vector3i(x, y, z);
					TVoxelPrimary& primaryVoxel = localLiveVoxelBlock[locId];
					TVoxelSecondary& secondaryVoxel = localCanonicalVoxelBlock[locId];
					functor(primaryVoxel, secondaryVoxel, voxelPosition);
				}
			}
		}
	}
};

template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary, typename TIndex>
inline void DualVoxelPositionTraversal_DieOnMiss_CPU(ITMScene<TVoxelPrimary, TIndex>* primaryScene,
                                                     ITMScene<TVoxelSecondary, TIndex>* secondaryScene,
                                                     TFunctor& functor) {

	DualVoxelPositionTraversal_CPU<TFunctor,
			DieOnMissFunctor<TVoxelSecondary, TIndex>, TVoxelPrimary, TVoxelSecondary, TIndex>(
			primaryScene, secondaryScene, nullptr, functor, false);
};


template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary, typename TIndex>
inline
void DualVoxelPositionTraversal_AllocateSecondaryOnMiss_CPU(ITMScene<TVoxelPrimary, TIndex>* primaryScene,
                                                            ITMScene<TVoxelSecondary, TIndex>* secondaryScene,
                                                            ORUtils::MemoryBlock<unsigned char>* entryAllocationTypes,
                                                            TFunctor& functor, bool reportAllocations = true) {
	uchar* entriesAllocType = entryAllocationTypes->GetData(MEMORYDEVICE_CPU);
	DualVoxelPositionTraversal_CPU<TFunctor,
			AllocateHashOnMissFunctor<TVoxelSecondary, TIndex>, TVoxelPrimary, TVoxelSecondary, TIndex>(
			primaryScene, secondaryScene, entriesAllocType, functor, reportAllocations);
};

template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary, typename TIndex>
inline
void DualVoxelPositionTraversal_AllocateSecondaryOnMiss_CPU_SingleThreaded(ITMScene<TVoxelPrimary, TIndex>* primaryScene,
                                                            ITMScene<TVoxelSecondary, TIndex>* secondaryScene,
                                                            ORUtils::MemoryBlock<unsigned char>* entryAllocationTypes,
                                                            TFunctor& functor, bool reportAllocations = true) {
	uchar* entriesAllocType = entryAllocationTypes->GetData(MEMORYDEVICE_CPU);
	DualVoxelPositionTraversal_CPU_SingleThreaded<TFunctor,
			AllocateHashOnMissFunctor<TVoxelSecondary, TIndex>, TVoxelPrimary, TVoxelSecondary, TIndex>(
			primaryScene, secondaryScene, entriesAllocType, functor, reportAllocations);
};

// endregion ===========================================================================================================

}//namespace ITMLib