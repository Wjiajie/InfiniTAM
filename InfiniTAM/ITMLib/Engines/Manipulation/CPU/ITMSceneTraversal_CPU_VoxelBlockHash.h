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

//local
#include "../Interface/ITMSceneTraversal.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "ITMSceneManipulationEngine_CPU.h"
#include "ITMSceneTraversal_CPU_AuxilaryFunctions.h"

namespace ITMLib {


//======================================================================================================================
//                            CONTAINS TRAVERSAL METHODS FOR SCENES USING ITMVoxelBlockHash FOR INDEXING
//======================================================================================================================

//static-member-only classes are used here instead of namespaces to utilize template specialization (and maximize code reuse)
template<typename TVoxel>
class ITMSceneTraversalEngine<TVoxel, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU> {
public:
// region ================================ DYNAMIC SINGLE-SCENE TRAVERSAL ==============================================

	template<typename TFunctor>
	inline static void
	VoxelTraversal(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, TFunctor& functor) {
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
	}

	template<typename TFunctor>
	inline static void
	VoxelPositionTraversal(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, TFunctor& functor) {
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
	}

	template<typename TFunctor>
	inline static void
	VoxelAndHashBlockPositionTraversal(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, TFunctor& functor) {
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
	}

	template<typename TFunctor>
	inline static void
	VoxelTraversalWithinBounds(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, TFunctor& functor, Vector6i bounds) {
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
	}


	template<typename TFunctor>
	inline static void
	VoxelPositionTraversalWithinBounds(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, TFunctor& functor,
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
	}

	template<typename TFunctor>
	inline static void
	VoxelPositionAndHashEntryTraversalWithinBounds(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, TFunctor& functor,
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
	}

// endregion ===========================================================================================================
// region ================================ STATIC SINGLE-SCENE TRAVERSAL ===============================================
	template<typename TStaticFunctor>
	inline static void StaticVoxelTraversal(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene) {
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
	}

	template<typename TStaticFunctor>
	inline static void StaticVoxelPositionTraversal(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene) {
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
	}
// endregion
};


// ====================== FOR TWO SCENES WITH DIFFERING VOXEL TYPES ====================================================

template<typename TVoxelPrimary, typename TVoxelSecondary>
class ITMDualSceneTraversalEngine<TVoxelPrimary, TVoxelSecondary, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU> {
public:
// region ================================ STATIC TWO-SCENE TRAVERSAL =================================================
	template<typename TStaticFunctor>
	inline static void StaticDualVoxelTraversal(
			ITMVoxelVolume<TVoxelPrimary, ITMVoxelBlockHash>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, ITMVoxelBlockHash>* secondaryScene) {

// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();
		int noTotalEntries = primaryScene->index.noTotalEntries;

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
					stream << "Could not find corresponding secondary scene block at postion "
					       << currentLiveHashEntry.pos
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
	}

	// endregion
	// region ================================ STATIC TWO-SCENE TRAVERSAL WITH VOXEL POSITION ==========================
	template<typename TStaticFunctor>
	inline static void StaticDualVoxelPositionTraversal(
			ITMVoxelVolume<TVoxelPrimary, ITMVoxelBlockHash>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, ITMVoxelBlockHash>* secondaryScene) {

// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();
		int noTotalEntries = primaryScene->index.noTotalEntries;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < noTotalEntries; hash++) {
			const ITMHashEntry& currentPrimaryHashEntry = primaryHashTable[hash];
			if (currentPrimaryHashEntry.ptr < 0) continue;
			ITMHashEntry currentSecondaryHashEntry = secondaryHashTable[hash];

			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (currentSecondaryHashEntry.pos != currentPrimaryHashEntry.pos) {
				int secondaryHash;
				if (!FindHashAtPosition(secondaryHash, currentPrimaryHashEntry.pos, secondaryHashTable)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << currentPrimaryHashEntry.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					currentSecondaryHashEntry = secondaryHashTable[secondaryHash];
				}
			}
			// position of the current entry in 3D space in voxel units
			Vector3i hashBlockPosition = currentPrimaryHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
			TVoxelPrimary* localLiveVoxelBlock = &(primaryVoxels[currentPrimaryHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
			TVoxelSecondary* localCanonicalVoxelBlock = &(secondaryVoxels[currentSecondaryHashEntry.ptr *
			                                                              (SDF_BLOCK_SIZE3)]);
			for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
				for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
					for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
						int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
						Vector3i voxelPosition = hashBlockPosition + Vector3i(x, y, z);
						TVoxelPrimary& primaryVoxel = localLiveVoxelBlock[locId];
						TVoxelSecondary& secondaryVoxel = localCanonicalVoxelBlock[locId];
						TStaticFunctor::run(primaryVoxel, secondaryVoxel, voxelPosition);
					}
				}
			}
		}
	}

// endregion
// region ================================ DYNAMIC TWO-SCENE TRAVERSAL =================================================
	template<typename TFunctor>
	inline static void
	DualVoxelTraversal(
			ITMVoxelVolume<TVoxelPrimary, ITMVoxelBlockHash>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, ITMVoxelBlockHash>* secondaryScene,
			TFunctor& functor) {

// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();


		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();
		int noTotalEntries = primaryScene->index.noTotalEntries;

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
					stream << "Could not find corresponding secondary scene block at postion "
					       << currentLiveHashEntry.pos
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
	}

	template<typename TFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue(
			ITMVoxelVolume<TVoxelPrimary, ITMVoxelBlockHash>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, ITMVoxelBlockHash>* secondaryScene,
			TFunctor& functor) {

// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();


		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();
		int noTotalEntries = primaryScene->index.noTotalEntries;

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
					stream << "Could not find corresponding secondary scene block at postion "
					       << currentLiveHashEntry.pos
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
						if(!functor(primaryVoxel, secondaryVoxel)){
							return false;
						}
					}
				}
			}
		}
		return true;
	}

// ========================== WITH VOXEL POSITION ==============
	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal(
			ITMVoxelVolume<TVoxelPrimary, ITMVoxelBlockHash>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, ITMVoxelBlockHash>* secondaryScene,
			TFunctor& functor) {

// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();
		int noTotalEntries = primaryScene->index.noTotalEntries;

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
					stream << "Could not find corresponding secondary scene block at postion "
					       << currentLiveHashEntry.pos
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
	}

	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversalWithinBounds(
			ITMVoxelVolume<TVoxelPrimary, ITMVoxelBlockHash>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, ITMVoxelBlockHash>* secondaryScene,
			TFunctor& functor, Vector6i bounds) {

// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();
		int noTotalEntries = primaryScene->index.noTotalEntries;

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
					stream << "Could not find corresponding secondary scene block at postion "
					       << currentLiveHashEntry.pos
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

			Vector3i hashEntryMinPoint = currentLiveHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
			Vector3i hashEntryMaxPoint = hashEntryMinPoint + Vector3i(SDF_BLOCK_SIZE);
			if (HashBlockDoesNotIntersectBounds(hashEntryMinPoint, hashEntryMaxPoint, bounds)) {
				continue;
			}
			Vector6i localBounds = computeLocalBounds(hashEntryMinPoint, hashEntryMaxPoint, bounds);

			for (int z = localBounds.min_z; z < localBounds.max_z; z++) {
				for (int y = localBounds.min_y; y < localBounds.max_y; y++) {
					for (int x = localBounds.min_x; x < localBounds.max_x; x++) {
						int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
						Vector3i voxelPosition = hashBlockPosition + Vector3i(x, y, z);
						TVoxelPrimary& primaryVoxel = localLiveVoxelBlock[locId];
						TVoxelSecondary& secondaryVoxel = localCanonicalVoxelBlock[locId];
						functor(primaryVoxel, secondaryVoxel, voxelPosition);
					}
				}
			}
		}
	}


	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal_DefaultForMissingSecondary(
			ITMVoxelVolume<TVoxelPrimary, ITMVoxelBlockHash>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, ITMVoxelBlockHash>* secondaryScene,
			TFunctor& functor) {

// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();
		int noTotalEntries = primaryScene->index.noTotalEntries;

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
	}


	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal_SingleThreaded(
			ITMVoxelVolume<TVoxelPrimary, ITMVoxelBlockHash>* primaryScene,
			ITMVoxelVolume<TVoxelSecondary, ITMVoxelBlockHash>* secondaryScene,
			TFunctor& functor) {

// *** traversal vars
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();
		int noTotalEntries = primaryScene->index.noTotalEntries;

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
					stream << "Could not find corresponding secondary scene block at postion "
					       << currentLiveHashEntry.pos
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
	}
// endregion ===========================================================================================================
};

// FOR TWO SCENES WITH SAME VOXEL TYPE AND DIFFERING WARP TYPE
// ====================== FOR TWO SCENES WITH DIFFERING VOXEL TYPES ====================================================

template<typename TVoxel, typename TWarp>
class ITMDualSceneWarpTraversalEngine<TVoxel, TWarp, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU> {
public:
// region ================================ DYNAMIC TWO-SCENE TRAVERSAL =================================================
	template<typename TStaticFunctor>
	inline static void StaticDualVoxelTraversal(
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* primaryScene,
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* secondaryScene,
			ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField) {

// *** traversal vars
		TVoxel* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxel* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();

		TWarp* warpVoxels = warpField->localVBA.GetVoxelBlocks();
		ITMHashEntry* warpHashTable = warpField->index.GetEntries();

		int noTotalEntries = warpField->index.noTotalEntries;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < noTotalEntries; hash++) {
			const ITMHashEntry& currentWarpHashEntry = warpHashTable[hash];
			if (currentWarpHashEntry.ptr < 0) continue;

			ITMHashEntry currentPrimaryHashEntry = primaryHashTable[hash];
			ITMHashEntry currentSecondaryHashEntry = secondaryHashTable[hash];

			// the rare case where we have different positions for warp & primary voxel block with the same index:
			// we have a hash bucket miss, find the primary voxel with the matching coordinates
			if (currentPrimaryHashEntry.pos != currentWarpHashEntry.pos) {
				int primaryHash;
				if (!FindHashAtPosition(primaryHash, currentWarpHashEntry.pos, primaryHashTable)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << currentWarpHashEntry.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					currentPrimaryHashEntry = primaryHashTable[primaryHash];
				}
			}
			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (currentSecondaryHashEntry.pos != currentPrimaryHashEntry.pos) {
				int secondaryHash;
				if (!FindHashAtPosition(secondaryHash, currentPrimaryHashEntry.pos, secondaryHashTable)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << currentPrimaryHashEntry.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					currentSecondaryHashEntry = secondaryHashTable[secondaryHash];
				}
			}
			TVoxel* localPrimaryVoxelBlock = &(primaryVoxels[currentPrimaryHashEntry.ptr * SDF_BLOCK_SIZE3]);
			TVoxel* localSecondaryVoxelBlock = &(secondaryVoxels[currentSecondaryHashEntry.ptr * SDF_BLOCK_SIZE3]);
			TWarp* localWarpVoxelBlock = &(warpVoxels[currentWarpHashEntry.ptr * SDF_BLOCK_SIZE3]);
			for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
				for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
					for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
						int indexWithinBlock = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
						TVoxel& primaryVoxel = localPrimaryVoxelBlock[indexWithinBlock];
						TVoxel& secondaryVoxel = localSecondaryVoxelBlock[indexWithinBlock];
						TWarp& warp = localWarpVoxelBlock[indexWithinBlock];
						TStaticFunctor::run(primaryVoxel, secondaryVoxel, warp);
					}
				}
			}
		}
	}


	template<typename TFunctor>
	inline static void
	DualVoxelTraversal(
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* primaryScene,
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* secondaryScene,
			ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField,
			TFunctor& functor) {

// *** traversal vars
		TVoxel* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxel* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();

		TWarp* warpVoxels = warpField->localVBA.GetVoxelBlocks();
		ITMHashEntry* warpHashTable = warpField->index.GetEntries();

		int noTotalEntries = warpField->index.noTotalEntries;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < noTotalEntries; hash++) {
			const ITMHashEntry& currentPrimaryHashEntry = primaryHashTable[hash];
			if (currentPrimaryHashEntry.ptr < 0) continue;
			ITMHashEntry currentWarpHashEntry = warpHashTable[hash];
			ITMHashEntry currentSecondaryHashEntry = secondaryHashTable[hash];

			// the rare case where we have different positions for warp & primary voxel block with the same index:
			// we have a hash bucket miss, find the primary voxel with the matching coordinates
			if (currentPrimaryHashEntry.pos != currentWarpHashEntry.pos) {
				int warpHash;
				if (!FindHashAtPosition(warpHash, currentPrimaryHashEntry.pos, warpHashTable)) {
					std::stringstream stream;
					stream << "Could not find corresponding warp scene block at position "
					       << currentWarpHashEntry.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					currentWarpHashEntry = warpHashTable[warpHash];
				}
			}
			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (currentSecondaryHashEntry.pos != currentPrimaryHashEntry.pos) {
				int secondaryHash;
				if (!FindHashAtPosition(secondaryHash, currentPrimaryHashEntry.pos, secondaryHashTable)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << currentPrimaryHashEntry.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					currentSecondaryHashEntry = secondaryHashTable[secondaryHash];
				}
			}
			TVoxel* localPrimaryVoxelBlock = &(primaryVoxels[currentPrimaryHashEntry.ptr * SDF_BLOCK_SIZE3]);
			TVoxel* localSecondaryVoxelBlock = &(secondaryVoxels[currentSecondaryHashEntry.ptr * SDF_BLOCK_SIZE3]);
			TWarp* localWarpVoxelBlock = &(warpVoxels[currentWarpHashEntry.ptr * SDF_BLOCK_SIZE3]);

			for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
				for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
					for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
						int indexWithinBlock = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
						TVoxel& primaryVoxel = localPrimaryVoxelBlock[indexWithinBlock];
						TVoxel& secondaryVoxel = localSecondaryVoxelBlock[indexWithinBlock];
						TWarp& warp = localWarpVoxelBlock[indexWithinBlock];
						functor(primaryVoxel, secondaryVoxel, warp);
					}
				}
			}
		}
	}


	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal(
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* primaryScene,
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* secondaryScene,
			ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField,
			TFunctor& functor) {

// *** traversal vars
		TVoxel* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxel* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();

		TWarp* warpVoxels = warpField->localVBA.GetVoxelBlocks();
		ITMHashEntry* warpHashTable = warpField->index.GetEntries();

		int noTotalEntries = warpField->index.noTotalEntries;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < noTotalEntries; hash++) {
			const ITMHashEntry& currentPrimaryHashEntry = primaryHashTable[hash];
			ITMHashEntry currentWarpHashEntry = warpHashTable[hash];
			ITMHashEntry currentSecondaryHashEntry = secondaryHashTable[hash];
			if (currentPrimaryHashEntry.ptr < 0) continue;
			// the rare case where we have different positions for warp & primary voxel block with the same index:
			// we have a hash bucket miss, find the primary voxel with the matching coordinates
			if (currentPrimaryHashEntry.pos != currentWarpHashEntry.pos) {
				int warpHash;
				if (!FindHashAtPosition(warpHash, currentPrimaryHashEntry.pos, warpHashTable)) {
					std::stringstream stream;
					stream << "Could not find corresponding warp scene block at position "
					       << currentWarpHashEntry.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					currentWarpHashEntry = warpHashTable[warpHash];
				}
			}
			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (currentSecondaryHashEntry.pos != currentPrimaryHashEntry.pos) {
				int secondaryHash;
				if (!FindHashAtPosition(secondaryHash, currentPrimaryHashEntry.pos, secondaryHashTable)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << currentPrimaryHashEntry.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					currentSecondaryHashEntry = secondaryHashTable[secondaryHash];
				}
			}
			Vector3i hashBlockPosition = currentWarpHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

			TVoxel* localPrimaryVoxelBlock = &(primaryVoxels[currentPrimaryHashEntry.ptr * SDF_BLOCK_SIZE3]);
			TVoxel* localSecondaryVoxelBlock = &(secondaryVoxels[currentSecondaryHashEntry.ptr * SDF_BLOCK_SIZE3]);
			TWarp* localWarpVoxelBlock = &(warpVoxels[currentWarpHashEntry.ptr * SDF_BLOCK_SIZE3]);

			for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
				for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
					for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
						Vector3i voxelPosition = hashBlockPosition + Vector3i(x, y, z);
						int indexWithinBlock = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
						TVoxel& primaryVoxel = localPrimaryVoxelBlock[indexWithinBlock];
						TVoxel& secondaryVoxel = localSecondaryVoxelBlock[indexWithinBlock];
						TWarp& warp = localWarpVoxelBlock[indexWithinBlock];
						functor(primaryVoxel, secondaryVoxel, warp, voxelPosition);
					}
				}
			}
		}
	}


	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal_DefaultForMissingEntries(
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* primaryScene,
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* secondaryScene,
			ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField,
			TFunctor& functor) {

// *** traversal vars
		TVoxel* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxel* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();

		TWarp* warpVoxels = warpField->localVBA.GetVoxelBlocks();
		ITMHashEntry* warpHashTable = warpField->index.GetEntries();

		int noTotalEntries = warpField->index.noTotalEntries;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int hash = 0; hash < noTotalEntries; hash++) {
			const ITMHashEntry& currentWarpHashEntry = warpHashTable[hash];
			if (currentWarpHashEntry.ptr < 0) continue;

			ITMHashEntry currentPrimaryHashEntry = primaryHashTable[hash];
			ITMHashEntry currentSecondaryHashEntry = secondaryHashTable[hash];
			bool primaryFound = true, secondaryFound = true;
// the rare case where we have different positions for warp & primary voxel block with the same index:
			// we have a hash bucket miss, find the primary voxel with the matching coordinates
			if (currentPrimaryHashEntry.pos != currentWarpHashEntry.pos) {
				int primaryHash;
				if (!FindHashAtPosition(primaryHash, currentWarpHashEntry.pos, primaryHashTable)) {
					primaryFound = false;
				} else {
					currentPrimaryHashEntry = primaryHashTable[primaryHash];
				}
			}
			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (currentSecondaryHashEntry.pos != currentPrimaryHashEntry.pos) {
				int secondaryHash;
				if (!FindHashAtPosition(secondaryHash, currentPrimaryHashEntry.pos, secondaryHashTable)) {
					secondaryFound = false;
				} else {
					currentSecondaryHashEntry = secondaryHashTable[secondaryHash];
				}
			}
			Vector3i hashBlockPosition = currentWarpHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

			TVoxel* localPrimaryVoxelBlock = &(primaryVoxels[currentPrimaryHashEntry.ptr * SDF_BLOCK_SIZE3]);
			TVoxel* localSecondaryVoxelBlock = &(secondaryVoxels[currentSecondaryHashEntry.ptr * SDF_BLOCK_SIZE3]);
			TWarp* localWarpVoxelBlock = &(warpVoxels[currentWarpHashEntry.ptr * SDF_BLOCK_SIZE3]);
			TVoxel defaultVoxel;

			for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
				for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
					for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
						int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
						Vector3i voxelPosition = hashBlockPosition + Vector3i(x, y, z);
						int indexWithinBlock = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
						TVoxel& primaryVoxel = primaryFound ? localPrimaryVoxelBlock[indexWithinBlock] : defaultVoxel;
						TVoxel& secondaryVoxel = secondaryFound ? localSecondaryVoxelBlock[indexWithinBlock]
						                                        : defaultVoxel;
						TWarp& warp = localWarpVoxelBlock[indexWithinBlock];
						functor(primaryVoxel, secondaryVoxel, warp, voxelPosition);
					}
				}
			}
		}
	}


	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal_SingleThreaded(
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* primaryScene,
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* secondaryScene,
			ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField,
			TFunctor& functor) {

// *** traversal vars
		TVoxel* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* secondaryHashTable = secondaryScene->index.GetEntries();

		TVoxel* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		ITMHashEntry* primaryHashTable = primaryScene->index.GetEntries();

		TWarp* warpVoxels = warpField->localVBA.GetVoxelBlocks();
		ITMHashEntry* warpHashTable = warpField->index.GetEntries();

		int noTotalEntries = warpField->index.noTotalEntries;

		for (int hash = 0; hash < noTotalEntries; hash++) {
			const ITMHashEntry& currentWarpHashEntry = warpHashTable[hash];
			if (currentWarpHashEntry.ptr < 0) continue;

			ITMHashEntry currentPrimaryHashEntry = primaryHashTable[hash];
			ITMHashEntry currentSecondaryHashEntry = secondaryHashTable[hash];

			// the rare case where we have different positions for warp & primary voxel block with the same index:
			// we have a hash bucket miss, find the primary voxel with the matching coordinates
			if (currentPrimaryHashEntry.pos != currentWarpHashEntry.pos) {
				int primaryHash;
				if (!FindHashAtPosition(primaryHash, currentWarpHashEntry.pos, primaryHashTable)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << currentWarpHashEntry.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					currentPrimaryHashEntry = primaryHashTable[primaryHash];
				}
			}
			// the rare case where we have different positions for primary & secondary voxel block with the same index:
			// we have a hash bucket miss, find the secondary voxel with the matching coordinates
			if (currentSecondaryHashEntry.pos != currentPrimaryHashEntry.pos) {
				int secondaryHash;
				if (!FindHashAtPosition(secondaryHash, currentPrimaryHashEntry.pos, secondaryHashTable)) {
					std::stringstream stream;
					stream << "Could not find corresponding secondary scene block at postion "
					       << currentPrimaryHashEntry.pos
					       << ". " << __FILE__ << ": " << __LINE__;
					DIEWITHEXCEPTION(stream.str());
				} else {
					currentSecondaryHashEntry = secondaryHashTable[secondaryHash];
				}
			}
			Vector3i hashBlockPosition = currentWarpHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
			TVoxel* localPrimaryVoxelBlock = &(primaryVoxels[currentPrimaryHashEntry.ptr * SDF_BLOCK_SIZE3]);
			TVoxel* localSecondaryVoxelBlock = &(secondaryVoxels[currentSecondaryHashEntry.ptr * SDF_BLOCK_SIZE3]);
			TWarp* localWarpVoxelBlock = &(warpVoxels[currentWarpHashEntry.ptr * SDF_BLOCK_SIZE3]);
			for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
				for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
					for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
						Vector3i voxelPosition = hashBlockPosition + Vector3i(x, y, z);
						int indexWithinBlock = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
						TVoxel& primaryVoxel = localPrimaryVoxelBlock[indexWithinBlock];
						TVoxel& secondaryVoxel = localSecondaryVoxelBlock[indexWithinBlock];
						TWarp& warp = localWarpVoxelBlock[indexWithinBlock];
						functor(primaryVoxel, secondaryVoxel, warp, voxelPosition);
					}
				}
			}
		}
	}
// endregion ===========================================================================================================
};

}//namespace ITMLib