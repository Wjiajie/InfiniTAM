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
#include "../../Utils/ITMSceneStatisticsCalculator.h"
#include "../../Objects/Scene/ITMTrilinearDistribution.h"


using namespace ITMLib;

//========================================== CONSTRUCTORS AND DESTRUCTORS ================================================
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ITMSceneMotionTracker_CPU(const ITMSceneParams& params,
                                                                                          std::string scenePath)
		: ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>(params, scenePath),
		  warpedEntryAllocationType(new ORUtils::MemoryBlock<unsigned char>(TIndex::noTotalEntries, MEMORYDEVICE_CPU)),
		  canonicalEntryAllocationTypes(
				  new ORUtils::MemoryBlock<unsigned char>(TIndex::noTotalEntries, MEMORYDEVICE_CPU)),
		  canonicalBlockCoords(new ORUtils::MemoryBlock<Vector3s>(TIndex::noTotalEntries, MEMORYDEVICE_CPU)) {
	uchar* entriesAllocType = this->canonicalEntryAllocationTypes->GetData(MEMORYDEVICE_CPU);
	memset(entriesAllocType, ITMLib::STABLE, static_cast<size_t>(TIndex::noTotalEntries));

}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::~ITMSceneMotionTracker_CPU() {
	delete warpedEntryAllocationType;
	delete canonicalEntryAllocationTypes;
	delete canonicalBlockCoords;
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

	uchar* entriesAllocType = this->canonicalEntryAllocationTypes->GetData(MEMORYDEVICE_CPU);
	Vector3s* allocationBlockCoords = this->canonicalBlockCoords->GetData(MEMORYDEVICE_CPU);
	int entryCount = TIndex::noTotalEntries;

	const short neighborhoodSize = 3;//must be an odd positive integer greater than 1.
	const short neighborhoodRangeStart = -neighborhoodSize / 2;
	const short neighborhoodRangeEnd = neighborhoodSize / 2 + 1;

	TVoxelCanonical* canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();
	typename TIndex::IndexCache canonicalCache;

	ITMHashEntry* liveHashTable = liveScene->index.GetEntries();
	typename TIndex::IndexCache liveCache;

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
		if (MarkAsNeedingAllocationIfNotFound(entriesAllocType, allocationBlockCoords, canonicalBlockIndex,
		                                      liveHashBlockCoords,
		                                      canonicalHashTable)) {
			countVoxelHashBlocksToAllocate++;
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
		if (currentCanonicalHashBlock.ptr < 0) continue; //hash block not allocated, continue
		TVoxelCanonical* localCanonicalVoxelBlock = &(canonicalVoxels[currentCanonicalHashBlock.ptr *
		                                                              (SDF_BLOCK_SIZE3)]);
		Vector3s hashBlockCoords = currentCanonicalHashBlock.pos;
		Vector3s currentBlockLocation = hashBlockCoords;
		int hashIdx = hashIndex(currentBlockLocation);
		int iNeighbor = 0;
		unsigned char entryAllocType = entriesAllocType[hashIdx];

		if (entryAllocType == ITMLib::STABLE) {
			//for all stable voxels, check the neighborhood
			for (short x = neighborhoodRangeStart; x < neighborhoodRangeEnd; x++) {
				for (short y = neighborhoodRangeStart; y < neighborhoodRangeEnd; y++) {
					for (short z = neighborhoodRangeStart; z < neighborhoodRangeEnd; z++) {
						//compute neighbor hash block position
						if (x == 0 && y == 0 && z == 0) continue;
						currentBlockLocation = hashBlockCoords + Vector3s(x, y, z);
						//compute index in hash table
						hashIdx = hashIndex(currentBlockLocation);
						if (MarkAsNeedingAllocationIfNotFound(entriesAllocType, allocationBlockCoords, hashIdx,
						                                      currentBlockLocation,
						                                      canonicalHashTable)) {
							countVoxelHashBlocksToAllocate++;
						}
						iNeighbor++;
					}
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
					hashEntry.pos = allocationBlockCoords[iTargetHashBlock];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
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
					hashEntry.pos = allocationBlockCoords[iTargetHashBlock];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
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

// ========================================== END CANONICAL HASH BLOCK ALLOCATION ======================================

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
	uchar* entriesAllocType = this->canonicalEntryAllocationTypes->GetData(MEMORYDEVICE_CPU);

	//_DEBUG
	int missedKnownVoxels = 0;
	int sdfTruncatedCount = 0;
	int hashBlockStabilizationCount = 0;
	float totalConf = 0.0f;

#ifdef WITH_OPENMP
	//#pragma omp parallel for reduction(+:missedKnownVoxels, sdfTruncatedCount, totalConf)
#endif
	for (int hash = 0; hash < noTotalEntries; hash++) {
		Vector3i canonicalHashEntryPosition;
		const ITMHashEntry& currentCanonicalHashEntry = canonicalHashTable[hash];

		if (currentCanonicalHashEntry.ptr < 0) continue;

		//position of the current hash block entry in 3D space (in voxel units)
		canonicalHashEntryPosition = currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		//pointer to the voxels in the block
		TVoxelCanonical* localVoxelBlock = &(canonicalVoxels[currentCanonicalHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

		//TODO: integrate color info --Greg(GitHub: Algomorph)
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
					oldWDepth = canonicalVoxel.w_depth; //0 for VOXEL_TRUNCATED voxels
					oldWColor = canonicalVoxel.w_color; //0 for VOXEL_TRUNCATED voxels

					//projected position of the sdf point to the most recent frame
					Vector3f projectedPosition = originalPosition.toFloat() + canonicalVoxel.warp_t;

					if (originalPosition == Vector3i(-18, 11, 161)) {
						oldSdf = canonicalVoxel.sdf;
					}

					Vector3f liveColor;
					int liveWDepth = 1;
					int liveWColor = 1;
					bool struckKnownVoxels;

					//TODO: confidence?
					//color & sdf
					float liveSdf = InterpolateTrilinearly_StruckKnownVoxels(
							liveVoxels, liveHashTable,
							projectedPosition,
							liveCache, liveColor,
							struckKnownVoxels);

					bool liveValueTruncated = 1.0 - std::abs(liveSdf) < FLT_EPSILON2;
					/* Conditions to avoid fusion:
					 * 1) No known voxels were struck in the live frame SDF by the query.
					 * 2) There were some voxels struck by the query, but the weights for them were all 0,
					 * resulting in the default value for live SDF.
					 */
					if (!struckKnownVoxels || liveSdf == TVoxelLive::SDF_initialValue()) {
						missedKnownVoxels++;//_DEBUG
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
					if (liveValueTruncated) {
						sdfTruncatedCount++;//_DEBUG
					}
					if (canonicalVoxel.flags == ITMLib::VOXEL_TRUNCATED) {
						if (!liveValueTruncated) {
							//voxel is no longer perceived as truncated
							canonicalVoxel.flags |= ITMLib::VOXEL_NONTRUNCATED;
							if (entriesAllocType[hash] != ITMLib::STABLE) {
								hashBlockStabilizationCount++;
							}
							entriesAllocType[hash] = ITMLib::STABLE;
						}
					}
				}
			}
		}
	}

	//_DEBUG
	std::cout << "Number of lookups that yielded initial (unknown) value in live (target) frame during fusion: "
	          << missedKnownVoxels << std::endl;
	std::cout << "Number of lookups that resulted in truncated value in live (target) frame during fusion: "
	          << sdfTruncatedCount << std::endl;
	std::cout << "Number of hash blocks that were stabilized (converted from boundary status) during fusion: "
	          << hashBlockStabilizationCount << std::endl;
//	std::cout << "Total confidence added from live frame: "
//	          << totalConf << std::endl;

}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
struct WarpSdfDistributionFunctor {
	WarpSdfDistributionFunctor(ITMScene<TVoxelLive, TIndex>* liveScene) {
		this->liveScene = liveScene;
		this->liveVoxels = liveScene->localVBA.GetVoxelBlocks();
		this->liveHashEntries = liveScene->index.GetEntries();

	}

	void operator()(TVoxelCanonical& voxel, Vector3i voxelPosition) {
		if (voxel.flags == ITMLib::VOXEL_TRUNCATED) return; // skip truncated voxels
		Vector3f warpedPosition = voxelPosition.toFloat() + voxel.warp_t;
		float sdfValue = TVoxelCanonical::valueToFloat(voxel.sdf);
		DistributeTrilinearly(liveVoxels, liveHashEntries, liveCache, warpedPosition, sdfValue);
	}

private:
	ITMScene<TVoxelLive, TIndex>* liveScene;
	TVoxelLive* liveVoxels;
	ITMHashEntry* liveHashEntries;
	typename TIndex::IndexCache liveCache;
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
struct WarpHashMarkerFunctor {
	WarpHashMarkerFunctor(ITMScene<TVoxelLive, TIndex>* liveScene, Vector3s* allocationBlockCoords,
	                      uchar* warpedEntryAllocationTypes) :
			liveScene(liveScene),
			liveVoxels(liveScene->localVBA.GetVoxelBlocks()),
			liveHashEntries(liveScene->index.GetEntries()),
			allocationBlockCoords(allocationBlockCoords),
			warpedEntryAllocationTypes(warpedEntryAllocationTypes) {}

	void operator()(TVoxelCanonical& voxel, Vector3i voxelPosition) {
		if (voxel.flags == ITMLib::VOXEL_TRUNCATED) return; // skip truncated voxels
		Vector3f warpedPosition = voxelPosition.toFloat() + voxel.warp_t;
		Vector3i warpedPositionTruncated = warpedPosition.toInt();

		short blockX = static_cast<short>(warpedPositionTruncated.x / SDF_BLOCK_SIZE);
		short blockY = static_cast<short>(warpedPositionTruncated.y / SDF_BLOCK_SIZE);
		short blockZ = static_cast<short>(warpedPositionTruncated.z / SDF_BLOCK_SIZE);

		// assess corner-cases for trilinear distribution to work properly at hash block boundaries
		bool needToAllocateExtraX = warpedPositionTruncated.x % SDF_BLOCK_SIZE == 7;
		bool needToAllocateExtraY = warpedPositionTruncated.y % SDF_BLOCK_SIZE == 7;
		bool needToAllocateExtraZ = warpedPositionTruncated.z % SDF_BLOCK_SIZE == 7;
		bool needToAllocateCornerXY = needToAllocateExtraX && needToAllocateExtraY;
		bool needToAllocateCornerYZ = needToAllocateExtraY && needToAllocateExtraZ;
		bool needToAllocateCornerXZ = needToAllocateExtraX && needToAllocateExtraZ;
		bool needToAllocateCornerXYZ = needToAllocateCornerXY && needToAllocateExtraZ;

		Vector3s originalNeededBlockCoordinate = Vector3s(blockX, blockY, blockZ);
		Vector3s neededBlockCoordinate = originalNeededBlockCoordinate;

#define MARK_ENTRY_FOR_ALLOCATION \
        int liveBlockIndex = hashIndex(neededBlockCoordinate);\
        MarkAsNeedingAllocationIfNotFound(warpedEntryAllocationTypes, allocationBlockCoords, liveBlockIndex,\
                              neededBlockCoordinate, liveHashEntries);
		MARK_ENTRY_FOR_ALLOCATION
		if (needToAllocateExtraX) {
			neededBlockCoordinate = originalNeededBlockCoordinate + Vector3s(1, 0, 0);
			MARK_ENTRY_FOR_ALLOCATION
		}
		if (needToAllocateExtraY) {
			neededBlockCoordinate = originalNeededBlockCoordinate + Vector3s(0, 1, 0);
			MARK_ENTRY_FOR_ALLOCATION
		}
		if (needToAllocateExtraZ) {
			neededBlockCoordinate = originalNeededBlockCoordinate + Vector3s(0, 0, 1);
			MARK_ENTRY_FOR_ALLOCATION
		}
		if (needToAllocateCornerXY) {
			neededBlockCoordinate = originalNeededBlockCoordinate + Vector3s(1, 1, 0);
			MARK_ENTRY_FOR_ALLOCATION
		}
		if (needToAllocateCornerYZ) {
			neededBlockCoordinate = originalNeededBlockCoordinate + Vector3s(0, 1, 1);
			MARK_ENTRY_FOR_ALLOCATION
		}
		if (needToAllocateCornerXZ) {
			neededBlockCoordinate = originalNeededBlockCoordinate + Vector3s(1, 0, 1);
			MARK_ENTRY_FOR_ALLOCATION
		}
		if (needToAllocateCornerXYZ) {
			neededBlockCoordinate = originalNeededBlockCoordinate + Vector3s(1, 1, 1);
			MARK_ENTRY_FOR_ALLOCATION
		}
#undef MARK_ENTRY_FOR_ALLOCATION
	}

private:
	ITMScene<TVoxelLive, TIndex>* liveScene;
	TVoxelLive* liveVoxels;
	ITMHashEntry* liveHashEntries;
	typename TIndex::IndexCache liveCache;
	Vector3s* allocationBlockCoords;
	uchar* warpedEntryAllocationTypes;
};


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ApplyWarp(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {
	int entryCount = TIndex::noTotalEntries;
	Vector3s* allocationBlockCoords = this->canonicalBlockCoords->GetData(MEMORYDEVICE_CPU);
	uchar* warpedEntryAllocationTypes = this->warpedEntryAllocationType->GetData(MEMORYDEVICE_CPU);
	memset(warpedEntryAllocationTypes, (unsigned char) 0, static_cast<size_t>(entryCount));

	WarpHashMarkerFunctor<TVoxelCanonical, TVoxelLive, TIndex> hashMarkerFunctor(liveScene, allocationBlockCoords,
	                                                                             warpedEntryAllocationTypes);
	VoxelPositionTraversal_CPU(*canonicalScene, hashMarkerFunctor);

	// Allocate the marked hash entries

	int lastFreeVoxelBlockId = liveScene->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = liveScene->index.GetLastFreeExcessListId();
	int* voxelAllocationList = liveScene->localVBA.GetAllocationList();
	int* excessAllocationList = liveScene->index.GetExcessAllocationList();
	ITMHashEntry* liveHashEntries = liveScene->index.GetEntries();

	for (int iTargetHashBlock = 0; iTargetHashBlock < entryCount; iTargetHashBlock++) {
		unsigned char entryAllocType = warpedEntryAllocationTypes[iTargetHashBlock];
		switch (entryAllocType) {
			case ITMLib::NEEDS_ALLOC_IN_ORDERED_LIST:

				if (lastFreeVoxelBlockId >= 0) //there is room in the voxel block array
				{
					ITMHashEntry hashEntry;
					hashEntry.pos = allocationBlockCoords[iTargetHashBlock];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
					liveHashEntries[iTargetHashBlock] = hashEntry;
					warpedEntryAllocationTypes[iTargetHashBlock] = ITMLib::BOUNDARY_STATE;
					lastFreeVoxelBlockId--;
				}

				break;
			case NEEDS_ALLOC_IN_EXCESS_LIST:

				if (lastFreeVoxelBlockId >= 0 &&
				    lastFreeExcessListId >= 0) //there is room in the voxel block array and excess list
				{
					ITMHashEntry hashEntry;
					hashEntry.pos = allocationBlockCoords[iTargetHashBlock];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
					int exlOffset = excessAllocationList[lastFreeExcessListId];
					liveHashEntries[iTargetHashBlock].offset = exlOffset + 1; //connect to child
					liveHashEntries[SDF_BUCKET_NUM + exlOffset] = hashEntry; //add child to the excess list
					warpedEntryAllocationTypes[iTargetHashBlock] = ITMLib::BOUNDARY_STATE;
					lastFreeVoxelBlockId--;
					lastFreeExcessListId--;
				}
				break;
		}
	}
	canonicalScene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	canonicalScene->index.SetLastFreeExcessListId(lastFreeExcessListId);

	WarpSdfDistributionFunctor<TVoxelCanonical, TVoxelLive, TIndex> distributionFunctor(liveScene);
	VoxelPositionTraversal_CPU(*canonicalScene, distributionFunctor);
}
