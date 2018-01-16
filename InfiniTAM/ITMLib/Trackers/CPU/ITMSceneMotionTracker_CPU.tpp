//  ================================================================
//  Created by Gregory Kramida on 10/18/17.
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



//stdlib
#include <cmath>
#include <iomanip>
#include <unordered_set>
#include <chrono>

//_DEBUG -- OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

//local
#include "ITMSceneMotionTracker_CPU.h"
#include "ITMSceneMotionTracker_CPU_UpdateWarp.tpp"
#include "../Shared/ITMSceneMotionTracker_Shared.h"


using namespace ITMLib;

//========================================== CONSTRUCTORS AND DESTRUCTORS ================================================
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ITMSceneMotionTracker_CPU(const ITMSceneParams& params)
		: ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>(params),
		  entriesAllocFill(new ORUtils::MemoryBlock<bool>(TIndex::noTotalEntries, MEMORYDEVICE_CPU)),
		  canonicalEntriesAllocType(
				  new ORUtils::MemoryBlock<unsigned char>(TIndex::noTotalEntries, MEMORYDEVICE_CPU)),
		  blockCoords(new ORUtils::MemoryBlock<Vector3s>(TIndex::noTotalEntries, MEMORYDEVICE_CPU)) {
	uchar* entriesAllocType = this->canonicalEntriesAllocType->GetData(MEMORYDEVICE_CPU);
	memset(entriesAllocType, ITMLib::NO_CHANGE, static_cast<size_t>(TIndex::noTotalEntries));

}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::~ITMSceneMotionTracker_CPU() {
	delete entriesAllocFill;
	delete canonicalEntriesAllocType;
	delete blockCoords;
}

//========================================= END CONSTRUCTORS AND DESTRUCTORS============================================



//========================================= HASH BLOCK ALLOCATION AT EACH FRAME ========================================
/**
 * \brief Allocates new hash blocks in the canonical (reference) frame at each frame to accommodate potential new
 * data arising from sensor motion and discovery of previously unseen surfaces
 * \tparam TVoxelCanonical type of voxels in the canonical scene
 * \tparam TVoxelLive type of voxels in the live scene
 * \tparam TIndex index used in scenes
 * \param canonicalScene the canonical (reference) scene
 * \param liveScene the live (target) scene
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::AllocateNewCanonicalHashBlocks(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {

	uchar* entriesAllocType = this->canonicalEntriesAllocType->GetData(MEMORYDEVICE_CPU);
	int entryCount = TIndex::noTotalEntries;
	Vector3s* blockCoords = this->blockCoords->GetData(MEMORYDEVICE_CPU);
	bool* entriesAllocFill = this->entriesAllocFill->GetData(MEMORYDEVICE_CPU);
	memset(entriesAllocFill, false, static_cast<size_t>(entryCount));

	const short neighborhoodSize = 3;//must be an odd positive integer greater than 1.
	const short neighborhoodRangeStart = -neighborhoodSize / 2;
	const short neighborhoodRangeEnd = neighborhoodSize / 2 + 1;

	TVoxelCanonical* canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();
	typename TIndex::IndexCache canonicalCache;

	TVoxelLive* liveVoxels = liveScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* liveHashTable = liveScene->index.GetEntries();
	typename TIndex::IndexCache liveCache;

	//adjust as necessary for alternative neighborhoodSize
	//_DEBUG(older alternative solution)
	Vector3i testVoxelLocations[27] = {Vector3i(0, 0, 0), Vector3i(3, 0, 0), Vector3i(7, 0, 0),
	                                   Vector3i(0, 3, 0), Vector3i(3, 3, 0), Vector3i(7, 3, 0),
	                                   Vector3i(0, 7, 0), Vector3i(3, 7, 0), Vector3i(7, 7, 0),

	                                   Vector3i(0, 0, 3), Vector3i(3, 0, 3), Vector3i(7, 0, 3),
	                                   Vector3i(0, 3, 3), Vector3i(3, 3, 3), Vector3i(7, 3, 3),
	                                   Vector3i(0, 7, 3), Vector3i(4, 7, 3), Vector3i(7, 7, 3),

	                                   Vector3i(0, 0, 7), Vector3i(3, 0, 7), Vector3i(7, 0, 7),
	                                   Vector3i(0, 3, 7), Vector3i(3, 3, 7), Vector3i(7, 3, 7),
	                                   Vector3i(0, 7, 7), Vector3i(4, 7, 7), Vector3i(7, 7, 7)};
	TVoxelCanonical positiveHashEntry[SDF_BLOCK_SIZE3];
	for (int iVoxel = 0; iVoxel < SDF_BLOCK_SIZE3; iVoxel++) {
		positiveHashEntry[iVoxel] = TVoxelCanonical();
		positiveHashEntry[iVoxel].flags = ITMLib::UNKNOWN;
	}

	//_DEBUG
	int countVoxelHashBlocksToAllocate = 0;

#ifdef WITH_OPENMP
#pragma omp parallel for reduction(+:countVoxelHashBlocksToAllocate)//_DEBUG
#endif
	for (int liveHashBlockIndex = 0; liveHashBlockIndex < entryCount; liveHashBlockIndex++) {
		const ITMHashEntry& currentLiveHashBlock = liveHashTable[liveHashBlockIndex];
		//skip unfilled live blocks
		if (currentLiveHashBlock.ptr < 0) {
			continue;
		}
		Vector3s liveHashBlockCoords = currentLiveHashBlock.pos;

		//try to find a corresponding canonical block, and mark it for allocation if not found
		int canonicalBlockIndex = hashIndex(liveHashBlockCoords);
		if (NeedsAllocation(entriesAllocType, blockCoords, canonicalBlockIndex, liveHashBlockCoords,
		                    canonicalHashTable)) {
			//_DEBUG
			countVoxelHashBlocksToAllocate++;
			//end _DEBUG
		}
	}
	//_DEBUG
	std::cout << "Number of live blocks without canonical correspondences: " << countVoxelHashBlocksToAllocate
	          << std::endl;


#ifdef WITH_OPENMP
#pragma omp parallel for reduction(+:countVoxelHashBlocksToAllocate)//_DEBUG
#endif
	for (int canonicalHashBlockIndex = 0; canonicalHashBlockIndex < entryCount; canonicalHashBlockIndex++) {
		const ITMHashEntry& currentCanonicalHashBlock = canonicalHashTable[canonicalHashBlockIndex];
		if (currentCanonicalHashBlock.ptr < 0) continue;
		TVoxelCanonical* localCanonicalVoxelBlock = &(canonicalVoxels[currentCanonicalHashBlock.ptr *
		                                                              (SDF_BLOCK_SIZE3)]);
		Vector3s hashBlockCoords = currentCanonicalHashBlock.pos;
		int iNeighbor = 0;
		for (short x = neighborhoodRangeStart; x < neighborhoodRangeEnd; x++) {
			for (short y = neighborhoodRangeStart; y < neighborhoodRangeEnd; y++) {
				for (short z = neighborhoodRangeStart; z < neighborhoodRangeEnd; z++) {
					//compute neighbor hash block position
					if (x == 0 && y == 0 && z == 0) continue;
					Vector3s currentBlockLocation = hashBlockCoords + Vector3s(x, y, z);
					//compute index in hash table
					int hashIdx = hashIndex(currentBlockLocation);
					unsigned char entryAllocType = entriesAllocType[hashIdx];

					if (entryAllocType == ITMLib::NO_CHANGE &&
					    NeedsAllocation(entriesAllocType, blockCoords, hashIdx, currentBlockLocation,
					                    canonicalHashTable)) {
						//_DEBUG (older alternative solution)
						Vector3i& testVoxelLocation = testVoxelLocations[iNeighbor];
						int locId = testVoxelLocation.x + testVoxelLocation.y * SDF_BLOCK_SIZE +
						            testVoxelLocation.z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
						entriesAllocFill[hashIdx] = localCanonicalVoxelBlock[locId].sdf + 1.0f > FLT_EPSILON;

						//_DEBUG
						countVoxelHashBlocksToAllocate++;
					}
					iNeighbor++;
				}
			}
		}
	}
	std::cout << "Total number of canonical hash blocks to be allocated before optimization: "
	          << countVoxelHashBlocksToAllocate << " out of " << entryCount << std::endl;

	int lastFreeVoxelBlockId = canonicalScene->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = canonicalScene->index.GetLastFreeExcessListId();
	int* voxelAllocationList = canonicalScene->localVBA.GetAllocationList();
	int* excessAllocationList = canonicalScene->index.GetExcessAllocationList();
	for (int iTargetHashBlock = 0; iTargetHashBlock < entryCount; iTargetHashBlock++) {
		unsigned char entryAllocType = entriesAllocType[iTargetHashBlock];
		switch (entryAllocType) {
			case ITMLib::NEEDS_ALLOC_IN_ORDERED_LIST:

				if (lastFreeVoxelBlockId >= 0) //there is room in the voxel block array
				{
					ITMHashEntry hashEntry;
					hashEntry.pos = blockCoords[iTargetHashBlock];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
//_DEBUG older alternative solution
//					if (entriesAllocFill[iTargetHashBlock]) {
//						TVoxelCanonical* localVoxelBlock = &(canonicalVoxels[hashEntry.ptr * (SDF_BLOCK_SIZE3)]);
//						memcpy(localVoxelBlock, &positiveHashEntry, SDF_BLOCK_SIZE3 * sizeof(TVoxelCanonical));
//					}
					TVoxelCanonical* localVoxelBlock = &(canonicalVoxels[hashEntry.ptr * (SDF_BLOCK_SIZE3)]);
					memcpy(localVoxelBlock, &positiveHashEntry, SDF_BLOCK_SIZE3 * sizeof(TVoxelCanonical));

					canonicalHashTable[iTargetHashBlock] = hashEntry;
					entriesAllocType[iTargetHashBlock] = ITMLib::BOUNDARY_STATE;
					lastFreeVoxelBlockId--;
				}

				break;
			case NEEDS_ALLOC_IN_EXCESS_LIST:

				if (lastFreeVoxelBlockId >= 0 &&
				    lastFreeExcessListId >= 0) //there is room in the voxel block array and excess list
				{
					ITMHashEntry hashEntry;
					hashEntry.pos = blockCoords[iTargetHashBlock];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
//_DEBUG older alternative solution
//					if (entriesAllocFill[iTargetHashBlock]) {
//						TVoxelCanonical* localVoxelBlock = &(canonicalVoxels[hashEntry.ptr * (SDF_BLOCK_SIZE3)]);
//						memcpy(localVoxelBlock, &positiveHashEntry, SDF_BLOCK_SIZE3 * sizeof(TVoxelCanonical));
//					}
//END _DEBUG
					TVoxelCanonical* localVoxelBlock = &(canonicalVoxels[hashEntry.ptr * (SDF_BLOCK_SIZE3)]);
					memcpy(localVoxelBlock, &positiveHashEntry, SDF_BLOCK_SIZE3 * sizeof(TVoxelCanonical));

					int exlOffset = excessAllocationList[lastFreeExcessListId];
					canonicalHashTable[iTargetHashBlock].offset = exlOffset + 1; //connect to child
					canonicalHashTable[SDF_BUCKET_NUM + exlOffset] = hashEntry; //add child to the excess list
					entriesAllocType[iTargetHashBlock] = ITMLib::BOUNDARY_STATE;
					lastFreeVoxelBlockId--;
					lastFreeExcessListId--;
				}
				break;
		}
	}
	canonicalScene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	canonicalScene->index.SetLastFreeExcessListId(lastFreeExcessListId);
}

// ========================================== END CANONCICAL HASH BLOCK ALLOCATION =====================================

// ========================================== FUSION ===================================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::FuseFrame(ITMScene<TVoxelCanonical,
		TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {
	TVoxelCanonical* canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();
	typename TIndex::IndexCache canonicalCache;

	const TVoxelLive* liveVoxels = liveScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* liveHashTable = liveScene->index.GetEntries();
	typename TIndex::IndexCache liveCache;

	int maximumWeight = canonicalScene->sceneParams->maxW;

	int noTotalEntries = canonicalScene->index.noTotalEntries;
	uchar* entriesAllocType = this->canonicalEntriesAllocType->GetData(MEMORYDEVICE_CPU);

	//_DEBUG
	int allTruncatedInLiveCount = 0;

#ifdef WITH_OPENMP
	//#pragma omp parallel for reduction(+:allTruncatedInLiveCount)
#endif
	for (int hashBlockIndex = 0; hashBlockIndex < noTotalEntries; hashBlockIndex++) {
		Vector3i canonicalHashEntryPosition;
		const ITMHashEntry& currentCanonicalHashEntry = canonicalHashTable[hashBlockIndex];

		if (currentCanonicalHashEntry.ptr < 0) continue;

		//position of the current hash block entry in 3D space (in voxel units)
		canonicalHashEntryPosition = currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		//pointer to the voxels in the block
		TVoxelCanonical* localVoxelBlock = &(canonicalVoxels[currentCanonicalHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3i originalPosition = canonicalHashEntryPosition + Vector3i(x, y, z);
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxelCanonical& canonicalVoxel = localVoxelBlock[locId];

					int oldWDepth, oldWColor;
					float oldSdf;
					oldSdf = canonicalVoxel.sdf;
					Vector3f oldColor = TO_FLOAT3(canonicalVoxel.clr) / 255.0f;
					oldWDepth = canonicalVoxel.w_depth; //0 for UNKNOWN voxels
					oldWColor = canonicalVoxel.w_color; //0 for UNKNOWN voxels

					//projected position of the sdf point to the most recent frame
					Vector3f projectedPosition = originalPosition.toFloat() + canonicalVoxel.warp_t;

					Vector3f liveColor;
					float liveConfidence;
					int liveWDepth = 1;
					int liveWColor = 1;
					bool struckNarrowBand;
					float liveSdf = interpolateTrilinearly_Corrected(liveVoxels, liveHashTable, projectedPosition,
					                                                 liveCache, liveColor, liveConfidence,
					                                                 struckNarrowBand);
					if (!struckNarrowBand) {
						allTruncatedInLiveCount++;//_DEBUG
						continue;
					}
					float newSdf = oldWDepth * oldSdf + liveWDepth * liveSdf;
					float newWDepth = oldWDepth + liveWDepth;
					newSdf /= newWDepth;
					newWDepth = MIN(newWDepth, maximumWeight);

					Vector3f newColor = oldWColor * oldColor + liveWColor * liveColor;
					float newWColor = oldWColor + liveWColor;
					newColor /= newWColor;
					newWColor = MIN(newWDepth, maximumWeight);

					canonicalVoxel.sdf = TVoxelCanonical::floatToValue(newSdf);
					canonicalVoxel.w_depth = (uchar) newWDepth;
					canonicalVoxel.clr = TO_UCHAR3(newColor * 255.0f);
					canonicalVoxel.w_color = (uchar) newWColor;
					canonicalVoxel.confidence += liveConfidence;
					if (canonicalVoxel.flags == ITMLib::UNKNOWN) {
						canonicalVoxel.flags = ITMLib::KNOWN;
						entriesAllocType[hashBlockIndex] = ITMLib::NO_CHANGE;
					}
				}
			}
		}
	}
	//_DEBUG
	std::cout << "Number of lookups that missed the narrow band in live (target) frame during fusion: "
	          << allTruncatedInLiveCount << std::endl;
}
