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
#include "../../Engines/Manipulation/ITMSceneManipulation.h"

namespace ITMLib {


//======================================================================================================================
//                              TRAVERSAL METHODS FOR SCENES USING ITMVoxelBlockHash FOR INDEXING
//======================================================================================================================
// region ================================ DYNAMIC SINGLE-SCENE TRAVERSAL ==============================================

template<typename TFunctor, typename TVoxel>
inline
void VoxelTraversal_CPU(ITMScene<TVoxel, ITMVoxelBlockHash>* scene, TFunctor& functor) {
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
					functor(voxel);
				}
			}
		}
	}
};


template<typename TFunctor, typename TVoxel>
inline
void VoxelPositionTraversal_CPU(ITMScene<TVoxel, ITMVoxelBlockHash>* scene, TFunctor& functor) {
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


template<typename TFunctor, typename TVoxel>
inline
void VoxelAndHashBlockPositionTraversal_CPU(ITMScene<TVoxel, ITMVoxelBlockHash>* scene, TFunctor& functor) {
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
					functor(voxel, voxelPosition, currentHashEntry.pos);
				}
			}
		}
	}
};

inline bool HashBlockDoesNotIntersectBounds(const Vector3i& hashEntryMinPoint, const Vector3i& hashEntryMaxPoint,
                                            const Vector6i& bounds) {
	return hashEntryMaxPoint.x < bounds.min_x ||
	       hashEntryMaxPoint.y < bounds.min_y ||
	       hashEntryMaxPoint.z < bounds.min_z ||
	       hashEntryMinPoint.x >= bounds.max_x ||
	       hashEntryMinPoint.y >= bounds.max_y ||
	       hashEntryMinPoint.z >= bounds.max_z;
}

inline
Vector6i computeLocalBounds(const Vector3i& hashEntryMinPoint, const Vector3i& hashEntryMaxPoint,
                            const Vector6i& bounds) {
	return Vector6i(std::max(0, bounds.min_x - hashEntryMinPoint.x),
	                std::max(0, bounds.min_y - hashEntryMinPoint.y),
	                std::max(0, bounds.min_z - hashEntryMinPoint.z),
	                std::min(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE - (hashEntryMaxPoint.x - bounds.max_x)),
	                std::min(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE - (hashEntryMaxPoint.y - bounds.max_y)),
	                std::min(SDF_BLOCK_SIZE, SDF_BLOCK_SIZE - (hashEntryMaxPoint.z - bounds.max_z)));
}

template<typename TFunctor, typename TVoxel>
inline void
VoxelTraversalWithinBounds_CPU(ITMScene<TVoxel, ITMVoxelBlockHash>* scene, TFunctor& functor, Vector6i bounds) {
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noTotalEntries;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		const ITMHashEntry& currentHashEntry = hashTable[entryId];
		if (currentHashEntry.ptr < 0) continue;
		Vector3i hashEntryMinPoint = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		Vector3i hashEntryMaxPoint = hashEntryMinPoint + Vector3i(SDF_BLOCK_SIZE);
		if (HashBlockDoesNotIntersectBounds(hashEntryMinPoint, hashEntryMaxPoint, bounds)) {
			continue;
		}
		Vector6i localBounds = computeLocalBounds(hashEntryMinPoint, hashEntryMaxPoint, bounds);
		TVoxel* localVoxelBlock = &(voxels[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		for (int z = localBounds.min_z; z < localBounds.max_z; z++) {
			for (int y = localBounds.min_y; y < localBounds.max_y; y++) {
				for (int x = localBounds.min_x; x < localBounds.max_x; x++) {
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxel& voxel = localVoxelBlock[locId];
					functor(voxel);
				}
			}
		}
	}
};


template<typename TFunctor, typename TVoxel>
inline void
VoxelPositionTraversalWithinBounds_CPU(ITMScene<TVoxel, ITMVoxelBlockHash>* scene, TFunctor& functor, Vector6i bounds) {
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noTotalEntries;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		const ITMHashEntry& currentHashEntry = hashTable[entryId];
		if (currentHashEntry.ptr < 0) continue;
		Vector3i hashEntryMinPoint = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		Vector3i hashEntryMaxPoint = hashEntryMinPoint + Vector3i(SDF_BLOCK_SIZE);
		if (HashBlockDoesNotIntersectBounds(hashEntryMinPoint, hashEntryMaxPoint, bounds)) {
			continue;
		}
		Vector6i localBounds = computeLocalBounds(hashEntryMinPoint, hashEntryMaxPoint, bounds);
		TVoxel* localVoxelBlock = &(voxels[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		//position of the current entry in 3D space (in voxel units)
		Vector3i hashEntryPosition = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		for (int z = localBounds.min_z; z < localBounds.max_z; z++) {
			for (int y = localBounds.min_y; y < localBounds.max_y; y++) {
				for (int x = localBounds.min_x; x < localBounds.max_x; x++) {
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					Vector3i voxelPosition = hashEntryPosition + Vector3i(x, y, z);
					TVoxel& voxel = localVoxelBlock[locId];
					functor(voxel, voxelPosition);
				}
			}
		}
	}
};


template<typename TFunctor, typename TVoxel>
inline void
VoxelPositionAndHashEntryTraversalWithinBounds_CPU(ITMScene<TVoxel, ITMVoxelBlockHash>* scene, TFunctor& functor,
                                                   Vector6i bounds) {
	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noTotalEntries;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		const ITMHashEntry& currentHashEntry = hashTable[entryId];
		if (currentHashEntry.ptr < 0) continue;
		Vector3i hashEntryMinPoint = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		Vector3i hashEntryMaxPoint = hashEntryMinPoint + Vector3i(SDF_BLOCK_SIZE);
		if (HashBlockDoesNotIntersectBounds(hashEntryMinPoint, hashEntryMaxPoint, bounds)) {
			continue;
		}
		Vector6i localBounds = computeLocalBounds(hashEntryMinPoint, hashEntryMaxPoint, bounds);

		functor.processHashEntry(currentHashEntry);

		TVoxel* localVoxelBlock = &(voxels[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		//position of the current entry in 3D space (in voxel units)
		Vector3i hashEntryPosition = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		for (int z = localBounds.min_z; z < localBounds.max_z; z++) {
			for (int y = localBounds.min_y; y < localBounds.max_y; y++) {
				for (int x = localBounds.min_x; x < localBounds.max_x; x++) {
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					Vector3i voxelPosition = hashEntryPosition + Vector3i(x, y, z);
					TVoxel& voxel = localVoxelBlock[locId];
					functor(voxel, voxelPosition);
				}
			}
		}
	}
};

// endregion ===========================================================================================================

// region ================================ STATIC SINGLE-SCENE TRAVERSAL ===============================================

template<typename TStaticFunctor, typename TVoxel>
inline void StaticVoxelTraversal_CPU(ITMScene<TVoxel, ITMVoxelBlockHash>* scene) {
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

template<typename TStaticFunctor, typename TVoxel>
inline void StaticVoxelPositionTraversal_CPU(ITMScene<TVoxel, ITMVoxelBlockHash>* scene) {
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
template<typename TStaticFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
inline void StaticDualVoxelTraversal_CPU(
		ITMScene<TVoxelPrimary, ITMVoxelBlockHash>* primaryScene,
		ITMScene<TVoxelSecondary, ITMVoxelBlockHash>* secondaryScene) {

// *** traversal vars
	TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();
	typename ITMVoxelBlockHash::IndexCache secondaryCache;


	TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();
	int noTotalEntries = primaryScene->index.noTotalEntries;
	typename ITMVoxelBlockHash::IndexCache primaryCache;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int hash = 0; hash < noTotalEntries; hash++) {
		const ITMHashEntry& currentLiveHashEntry = primaryHashTable[hash];
		if (currentLiveHashEntry.ptr < 0) continue;
		ITMHashEntry currentCanonicalHashEntry = secondaryHashTable[hash];

		// the rare case where we have different positions for primary & secondary voxel block with the same index:
		// we have a hash bucket miss, find the secondary voxel with the matching coordinates
		if (currentCanonicalHashEntry.pos != currentLiveHashEntry.pos) {
			int secondaryHash;
			if (!FindHashAtPosition(secondaryHash, currentLiveHashEntry.pos, secondaryHashTable)) {
				std::stringstream stream;
				stream << "Could not find corresponding secondary scene block at postion " << currentLiveHashEntry.pos
				       << ". " << __FILE__ << ": " << __LINE__;
				DIEWITHEXCEPTION(stream.str());
			} else {
				currentCanonicalHashEntry = secondaryHashTable[secondaryHash];
			}
		}
		TVoxelPrimary* localLiveVoxelBlock = &(primaryVoxels[currentLiveHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		TVoxelSecondary* localCanonicalVoxelBlock = &(secondaryVoxels[currentCanonicalHashEntry.ptr *
		                                                              (SDF_BLOCK_SIZE3)]);
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxelPrimary& primaryVoxel = localLiveVoxelBlock[locId];
					TVoxelSecondary& secondaryVoxel = localCanonicalVoxelBlock[locId];
					TStaticFunctor::run(primaryVoxel, secondaryVoxel);
				}
			}
		}
	}
};


template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
inline void DualVoxelTraversal_CPU(
		ITMScene<TVoxelPrimary, ITMVoxelBlockHash>* primaryScene,
		ITMScene<TVoxelSecondary, ITMVoxelBlockHash>* secondaryScene,
		TFunctor& functor) {

// *** traversal vars
	TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();
	typename ITMVoxelBlockHash::IndexCache secondaryCache;


	TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();
	int noTotalEntries = primaryScene->index.noTotalEntries;
	typename ITMVoxelBlockHash::IndexCache primaryCache;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int hash = 0; hash < noTotalEntries; hash++) {

		const ITMHashEntry& currentLiveHashEntry = primaryHashTable[hash];
		if (currentLiveHashEntry.ptr < 0) continue;
		ITMHashEntry currentCanonicalHashEntry = secondaryHashTable[hash];

		// the rare case where we have different positions for primary & secondary voxel block with the same index:
		// we have a hash bucket miss, find the secondary voxel with the matching coordinates
		if (currentCanonicalHashEntry.pos != currentLiveHashEntry.pos) {
			int secondaryHash;
			if (!FindHashAtPosition(secondaryHash, currentLiveHashEntry.pos, secondaryHashTable)) {
				std::stringstream stream;
				stream << "Could not find corresponding secondary scene block at postion " << currentLiveHashEntry.pos
				       << ". " << __FILE__ << ": " << __LINE__;
				DIEWITHEXCEPTION(stream.str());
			} else {
				currentCanonicalHashEntry = secondaryHashTable[secondaryHash];
			}
		}
		TVoxelPrimary* localLiveVoxelBlock = &(primaryVoxels[currentLiveHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		TVoxelSecondary* localCanonicalVoxelBlock = &(secondaryVoxels[currentCanonicalHashEntry.ptr *
		                                                              (SDF_BLOCK_SIZE3)]);
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxelPrimary& primaryVoxel = localLiveVoxelBlock[locId];
					TVoxelSecondary& secondaryVoxel = localCanonicalVoxelBlock[locId];
					functor(primaryVoxel, secondaryVoxel);
				}
			}
		}
	}
};


template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
inline void DualVoxelPositionTraversal_CPU(
		ITMScene<TVoxelPrimary, ITMVoxelBlockHash>* primaryScene,
		ITMScene<TVoxelSecondary, ITMVoxelBlockHash>* secondaryScene,
		TFunctor& functor) {

// *** traversal vars
	TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();
	typename ITMVoxelBlockHash::IndexCache secondaryCache;


	TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();
	int noTotalEntries = primaryScene->index.noTotalEntries;
	typename ITMVoxelBlockHash::IndexCache primaryCache;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int hash = 0; hash < noTotalEntries; hash++) {
		const ITMHashEntry& currentLiveHashEntry = primaryHashTable[hash];
		if (currentLiveHashEntry.ptr < 0) continue;
		ITMHashEntry currentCanonicalHashEntry = secondaryHashTable[hash];

		// the rare case where we have different positions for primary & secondary voxel block with the same index:
		// we have a hash bucket miss, find the secondary voxel with the matching coordinates
		if (currentCanonicalHashEntry.pos != currentLiveHashEntry.pos) {
			int secondaryHash;

			if (!FindHashAtPosition(secondaryHash, currentLiveHashEntry.pos, secondaryHashTable)) {
				std::stringstream stream;
				stream << "Could not find corresponding secondary scene block at postion " << currentLiveHashEntry.pos
				       << ". " << __FILE__ << ": " << __LINE__;
				DIEWITHEXCEPTION(stream.str());
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


template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
inline void DualVoxelPositionTraversal_DefaultForMissingSecondary_CPU(
		ITMScene<TVoxelPrimary, ITMVoxelBlockHash>* primaryScene,
		ITMScene<TVoxelSecondary, ITMVoxelBlockHash>* secondaryScene,
		TFunctor& functor) {

// *** traversal vars
	TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();
	typename ITMVoxelBlockHash::IndexCache secondaryCache;


	TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();
	int noTotalEntries = primaryScene->index.noTotalEntries;
	typename ITMVoxelBlockHash::IndexCache primaryCache;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int hash = 0; hash < noTotalEntries; hash++) {
		const ITMHashEntry& currentLiveHashEntry = primaryHashTable[hash];
		if (currentLiveHashEntry.ptr < 0) continue;
		ITMHashEntry currentCanonicalHashEntry = secondaryHashTable[hash];

		// the rare case where we have different positions for primary & secondary voxel block with the same index:
		// we have a hash bucket miss, find the secondary voxel with the matching coordinates
		if (currentCanonicalHashEntry.pos != currentLiveHashEntry.pos) {
			int secondaryHash;
			if (!FindHashAtPosition(secondaryHash, currentLiveHashEntry.pos, secondaryHashTable)) {
				TVoxelPrimary* localLiveVoxelBlock = &(primaryVoxels[currentLiveHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
				Vector3i hashBlockPosition = currentLiveHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
				TVoxelSecondary secondaryVoxel;
				for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
					for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
						for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
							int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
							Vector3i voxelPosition = hashBlockPosition + Vector3i(x, y, z);
							TVoxelPrimary& primaryVoxel = localLiveVoxelBlock[locId];
							functor(primaryVoxel, secondaryVoxel, voxelPosition);
						}
					}
				}
				continue;
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


template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
inline void DualVoxelPositionTraversal_CPU_SingleThreaded(
		ITMScene<TVoxelPrimary, ITMVoxelBlockHash>* primaryScene,
		ITMScene<TVoxelSecondary, ITMVoxelBlockHash>* secondaryScene,
		TFunctor& functor) {

// *** traversal vars
	TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();
	typename ITMVoxelBlockHash::IndexCache secondaryCache;


	TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();
	int noTotalEntries = primaryScene->index.noTotalEntries;
	typename ITMVoxelBlockHash::IndexCache primaryCache;

	for (int hash = 0; hash < noTotalEntries; hash++) {
		const ITMHashEntry& currentLiveHashEntry = primaryHashTable[hash];
		if (currentLiveHashEntry.ptr < 0) continue;
		ITMHashEntry currentCanonicalHashEntry = secondaryHashTable[hash];

		// the rare case where we have different positions for primary & secondary voxel block with the same index:
		// we have a hash bucket miss, find the secondary voxel with the matching coordinates
		if (currentCanonicalHashEntry.pos != currentLiveHashEntry.pos) {
			int secondaryHash;
			if (!FindHashAtPosition(secondaryHash, currentLiveHashEntry.pos, secondaryHashTable)) {
				std::stringstream stream;
				stream << "Could not find corresponding secondary scene block at postion " << currentLiveHashEntry.pos
				       << ". " << __FILE__ << ": " << __LINE__;
				DIEWITHEXCEPTION(stream.str());
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

// endregion ===========================================================================================================

}//namespace ITMLib