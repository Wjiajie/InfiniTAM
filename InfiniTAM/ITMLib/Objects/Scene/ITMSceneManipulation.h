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

//TODO: Make GPU versions -Greg (GitHub: Algomorph)

namespace ITMLib {


template <typename TFunctor, typename TVoxel, typename TIndex>
inline
void VoxelTraversal_CPU(ITMScene<TVoxel, TIndex>& scene, TFunctor& functor){
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

template <typename TFunctor, typename TVoxel, typename TIndex>
inline
void VoxelPositionTraversal_CPU(ITMScene<TVoxel, TIndex>& scene, TFunctor& functor){
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
					functor(voxel,voxelPosition);
				}
			}
		}
	}
};

template <typename TStaticFunctor, typename TVoxel, typename TIndex>
inline
void StaticVoxelTraversal_CPU(ITMScene<TVoxel, TIndex>& scene){
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

template <typename TStaticFunctor, typename TVoxel, typename TIndex>
inline
void StaticVoxelPositionTraversal_CPU(ITMScene<TVoxel, TIndex>& scene){
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
					TStaticFunctor::run(voxel,voxelPosition);
				}
			}
		}
	}
};


bool AllocateHashEntry_CPU(const Vector3s& hashEntryPosition,
                           ITMHashEntry* hashTable,
                           ITMHashEntry*& resultEntry,
                           int& lastFreeVoxelBlockId,
                           int& lastFreeExcessListId,
                           const int* voxelAllocationList,
                           const int* excessAllocationList);

template<class TVoxel, class TIndex>
void CopySceneWithOffset_CPU(ITMScene <TVoxel, TIndex>& destination,
                             ITMScene <TVoxel, TIndex>& source,
                             Vector3i offset);

//TODO -make this suitable for source/dest scenes with different voxel types somehow -Greg (Github: Algomorph)
void CopySceneWithOffset_CPU(ITMScene <ITMVoxelLive, ITMVoxelIndex>& destination,
                             ITMScene <ITMVoxelCanonical, ITMVoxelIndex>& source,
                             Vector3i offset);

void CopySceneWithOffset_CPU(ITMScene <ITMVoxelCanonical, ITMVoxelIndex>& destination,
                             ITMScene <ITMVoxelLive, ITMVoxelIndex>& source,
                             Vector3i offset);

template<class TVoxel, class TIndex>
TVoxel ReadVoxel(ITMScene <TVoxel, TIndex>& scene, Vector3i at);

int FindHashBlock(const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData)* voxelIndex, const THREADPTR(Vector3s)& at);


template<class TVoxel, class TIndex>
bool SetVoxel_CPU(ITMScene <TVoxel, TIndex>& scene, Vector3i at, TVoxel voxel);


template<class TVoxel, class TIndex>
void
CopySceneWithOffset_CPU(ITMScene <TVoxel, TIndex>& destination, ITMScene <TVoxel, TIndex>& source, Vector3i offset);

void CopySceneWithOffset_CPU(ITMScene <ITMVoxelLive, ITMVoxelIndex>& destination,
                             ITMScene <ITMVoxel, ITMVoxelIndex>& source,Vector3i offset);
void CopySceneWithOffset_CPU(ITMScene <ITMVoxelCanonical, ITMVoxelIndex>& destination,
                             ITMScene <ITMVoxelLive, ITMVoxelIndex>& source, Vector3i offset);

template<class TVoxel, class TIndex>
void OffsetWarps(ITMScene <TVoxel, TIndex>& scene, Vector3f offset);

template<class TVoxel, class TIndex>
TVoxel ReadVoxel(ITMScene <TVoxel, TIndex>& scene, Vector3i at);


template<class TVoxel, class TIndex>
bool SetVoxel_CPU(ITMScene <TVoxel, TIndex>& scene, Vector3i at, TVoxel voxel);
}//namespace ITMLib