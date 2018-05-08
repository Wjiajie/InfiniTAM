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
#include <unordered_map>

//local
#include "ITMSceneMotionTracker_CPU.h"
#include "ITMSceneMotionTracker_CPU_UpdateWarp.tpp"
#include "../Shared/ITMSceneMotionTracker_Shared_Old.h"
#include "../../Utils/ITMSceneStatisticsCalculator.h"
#include "../../Objects/Scene/ITMTrilinearDistribution.h"
#include "../../Objects/Scene/ITMSceneManipulation.h"
#include "../../Utils/ITMLibSettings.h"


using namespace ITMLib;

// region ================================ CONSTRUCTORS AND DESTRUCTORS ================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ITMSceneMotionTracker_CPU(const ITMSceneParams& params, std::string scenePath, bool enableDataTerm,
                                                                                          bool enableLevelSetTerm, bool enableSmoothingTerm, bool enableKillingTerm,
                                                                                          bool enableGradientSmoothing)
		: ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>(params, scenePath),
		  hashEntryAllocationTypes(new ORUtils::MemoryBlock<unsigned char>(TIndex::noTotalEntries, MEMORYDEVICE_CPU)),
		  canonicalEntryAllocationTypes(
				  new ORUtils::MemoryBlock<unsigned char>(TIndex::noTotalEntries, MEMORYDEVICE_CPU)),
		  allocationBlockCoordinates(new ORUtils::MemoryBlock<Vector3s>(TIndex::noTotalEntries, MEMORYDEVICE_CPU)),
		  enableDataTerm(enableDataTerm),
		  enableLevelSetTerm(enableLevelSetTerm),
		  enableSmoothingTerm(enableSmoothingTerm),
		  enableKillingTerm(enableKillingTerm),
          enableGradientSmoothing(enableGradientSmoothing){
	InitializeHelper(params);

}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ITMSceneMotionTracker_CPU(const ITMSceneParams& params, std::string scenePath, Vector3i focusCoordinates,
                                                                                          bool enableDataTerm, bool enableLevelSetTerm, bool enableSmoothingTerm,
                                                                                          bool enableKillingTerm, bool enableGradientSmoothing)
		: ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>(params, scenePath, focusCoordinates),
		  hashEntryAllocationTypes(new ORUtils::MemoryBlock<unsigned char>(TIndex::noTotalEntries, MEMORYDEVICE_CPU)),
		  canonicalEntryAllocationTypes(
				  new ORUtils::MemoryBlock<unsigned char>(TIndex::noTotalEntries, MEMORYDEVICE_CPU)),
		  allocationBlockCoordinates(new ORUtils::MemoryBlock<Vector3s>(TIndex::noTotalEntries, MEMORYDEVICE_CPU)),
		  enableDataTerm(enableDataTerm),
		  enableLevelSetTerm(enableLevelSetTerm),
		  enableSmoothingTerm(enableSmoothingTerm),
		  enableKillingTerm(enableKillingTerm),
		  enableGradientSmoothing(enableGradientSmoothing){
	InitializeHelper(params);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::~ITMSceneMotionTracker_CPU() {
	delete hashEntryAllocationTypes;
	delete canonicalEntryAllocationTypes;
	delete allocationBlockCoordinates;

}

// endregion ============================== END CONSTRUCTORS AND DESTRUCTORS============================================


// region ================================== HASH BLOCK ALLOCATION AT EACH FRAME =======================================
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
	Vector3s* allocationBlockCoords = this->allocationBlockCoordinates->GetData(MEMORYDEVICE_CPU);
	int entryCount = TIndex::noTotalEntries;

	TVoxelCanonical* canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();
	typename TIndex::IndexCache canonicalCache;

	ITMHashEntry* liveHashTable = liveScene->index.GetEntries();
	typename TIndex::IndexCache liveCache;

	//_DEBUG
	//int countVoxelHashBlocksToAllocate = 0;


	// TODO: make a separate function
	const int& currentFrameIx = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::currentFrameIx;
	if (currentFrameIx == 0) {
		// region === INITIALIZATION =======================================================================================
		// at frame zero, allocate all the same blocks as in live frame
#ifdef WITH_OPENMP
#pragma omp parallel for
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
			                                      liveHashBlockCoords, canonicalHashTable)) {
				//countVoxelHashBlocksToAllocate++;
			}
		}
		//_DEBUG
//		std::cout << "Frame 0 allocation, number of live blocks without canonical correspondences: "
//		          << countVoxelHashBlocksToAllocate
//		          << std::endl;

//		std::cout << "Total number of canonical hash blocks to be allocated before optimization: "
//		          << countVoxelHashBlocksToAllocate << " out of " << entryCount << std::endl;
		AllocateHashEntriesUsingLists_CPU(canonicalScene, entriesAllocType, allocationBlockCoords, ITMLib::STABLE);
		// endregion
	} else {
		// region === EXPANSION ============================================================================================
		// traverse canonical allocated blocks, if a block is stable, mark it's neighbors for allocation as boundaries
		const short neighborhoodSize = 3;//must be an odd positive integer greater than 1.
		const short neighborhoodRangeStart = -neighborhoodSize / 2;
		const short neighborhoodRangeEnd = neighborhoodSize / 2 + 1;

#ifdef WITH_OPENMP
#pragma omp parallel for
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
							                                      currentBlockLocation, canonicalHashTable)) {
								//countVoxelHashBlocksToAllocate++;
							}
							iNeighbor++;
						}
					}
				}
			}
		}
//		std::cout << "Total number of canonical hash blocks to be allocated before optimization: "
//		          << countVoxelHashBlocksToAllocate << " out of " << entryCount << std::endl;
		AllocateHashEntriesUsingLists_CPU(canonicalScene, entriesAllocType, allocationBlockCoords, ITMLib::BOUNDARY);
		// endregion
	}
}

// endregion ==================================== END CANONICAL HASH BLOCK ALLOCATION ==================================

// region =================================== FUSION ===================================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::FuseFrame(ITMScene<TVoxelCanonical,
		TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {
	TVoxelCanonical* canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();
	typename TIndex::IndexCache canonicalCache;

	const TVoxelLive* liveVoxels = liveScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* liveHashTable = liveScene->index.GetEntries();
	typename TIndex::IndexCache liveCache;
	int maximumWeight = canonicalScene->sceneParams->maxW;
	int noTotalEntries = canonicalScene->index.noTotalEntries;
	uchar* entriesAllocType = this->canonicalEntryAllocationTypes->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
	//#pragma omp parallel for reduction(+:missedKnownVoxels, sdfTruncatedCount, totalConf)
#endif
	for (int hash = 0; hash < noTotalEntries; hash++) {
		const ITMHashEntry& currentLiveHashEntry = liveHashTable[hash];
		if (currentLiveHashEntry.ptr < 0) continue;
		ITMHashEntry& currentCanonicalHashEntry = canonicalHashTable[hash];

		// the rare case where we have different positions for live & canonical voxel block with the same index:
		// we have a hash bucket miss, find the canonical voxel with the matching coordinates
		if (currentCanonicalHashEntry.pos != currentLiveHashEntry.pos) {
			int canonicalHash = hash;
			if (!FindHashAtPosition(canonicalHash, currentLiveHashEntry.pos, canonicalHashTable)) {
				std::stringstream stream;
				const int currentFrameIx = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::currentFrameIx;
				stream << "Could not find corresponding canonical block at postion " << currentLiveHashEntry.pos
				       << " at frame " << currentFrameIx << ". " << __FILE__ << ": " << __LINE__;
				DIEWITHEXCEPTION(stream.str());
			}
			currentCanonicalHashEntry = canonicalHashTable[canonicalHash];
		}

		const TVoxelLive* localLiveVoxelBlock = &(liveVoxels[currentLiveHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		TVoxelCanonical* localCanonicalVoxelBlock = &(canonicalVoxels[currentCanonicalHashEntry.ptr *
		                                                              (SDF_BLOCK_SIZE3)]);

		//TODO: integrate color info --Greg(GitHub: Algomorph)
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

					const TVoxelLive& liveVoxel = localLiveVoxelBlock[locId];
					if (liveVoxel.flags != ITMLib::VOXEL_NONTRUNCATED) continue;
					TVoxelCanonical& canonicalVoxel = localCanonicalVoxelBlock[locId];
					int oldWDepth = canonicalVoxel.w_depth;
					float oldSdf = TVoxelCanonical::valueToFloat(canonicalVoxel.sdf);

					//float liveWeight = 1.0f;
					float liveSdf = TVoxelLive::valueToFloat(liveVoxel.sdf);


					float newSdf = oldWDepth * oldSdf + liveSdf;
					float newWDepth = oldWDepth + 1.0f;
					newSdf /= newWDepth;
					newWDepth = MIN(newWDepth, maximumWeight);

					canonicalVoxel.sdf = TVoxelCanonical::floatToValue(newSdf);
					canonicalVoxel.w_depth = (uchar) newWDepth;

					switch (canonicalVoxel.flags) {
						case ITMLib::VOXEL_UNKNOWN:
							if (liveVoxel.flags == ITMLib::VOXEL_NONTRUNCATED) {
								//voxel is no longer perceived as truncated
								canonicalVoxel.flags = ITMLib::VOXEL_NONTRUNCATED;
								entriesAllocType[hash] = ITMLib::STABLE;
							} else {
								//TODO: remove this if the system works -Greg (GitHub:Algomorph)
								canonicalVoxel.flags = ITMLib::VOXEL_TRUNCATED;
							}
							break;
						case ITMLib::VOXEL_TRUNCATED:
							if (liveVoxel.flags == ITMLib::VOXEL_NONTRUNCATED) {
								//voxel is no longer perceived as truncated
								canonicalVoxel.flags = ITMLib::VOXEL_NONTRUNCATED;
								entriesAllocType[hash] = ITMLib::STABLE;
							} else if (std::signbit(oldSdf) != std::signbit(liveSdf)) {
								//both voxels are truncated but differ in sign
								//TODO: remove this if the system works -Greg (GitHub:Algomorph)
								//TODO: if it is faster, optimize this to short-circuit the computation before instead -Greg (GitHub: Algomorph)
								canonicalVoxel.sdf = 1.0;
								canonicalVoxel.w_depth = 1.0;
//								if (liveWeight > 0.25f && liveSdf > 0.0f) {
//									//set truncated to the sign of the live truncated, i.e. now belongs to different surface
//									canonicalVoxel.sdf = 1.0;
//									canonicalVoxel.w_depth = liveWeight;
//								} else {
//									canonicalVoxel.sdf = oldSdf;
//									canonicalVoxel.w_depth -= liveWeight;
//								}
							}
							break;
						default:
							break;
					}
				}
			}
		}
	}
	ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::currentFrameIx++;
}
// endregion ===========================================================================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
struct WarpSdfDistributionFunctor {
	WarpSdfDistributionFunctor(ITMScene<TVoxelLive, TIndex>* liveScene) {
		this->liveScene = liveScene;
		this->liveVoxels = liveScene->localVBA.GetVoxelBlocks();
		this->liveHashEntries = liveScene->index.GetEntries();

	}

	void operator()(TVoxelCanonical& voxel, Vector3i voxelPosition) {
		if (voxel.flags == ITMLib::VOXEL_TRUNCATED) return; // skip truncated voxels
		Vector3f warpedPosition = voxelPosition.toFloat() + voxel.warp;
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
struct WarpHashAllocationMarkerFunctor {
	WarpHashAllocationMarkerFunctor(ITMScene<TVoxelLive, TIndex>* targetScene, Vector3s* allocationBlockCoords,
	                                uchar* warpedEntryAllocationTypes) :
			targetScene(targetScene),
			targetVoxels(targetScene->localVBA.GetVoxelBlocks()),
			targetHashEntries(targetScene->index.GetEntries()),
			allocationBlockCoords(allocationBlockCoords),
			warpedEntryAllocationTypes(warpedEntryAllocationTypes) {}

	void MarkEntryForAllocationIfTrue(const bool& condition, Vector3s neededBlockCoordinate) {
		if (condition) {
			int liveBlockIndex = hashIndex(neededBlockCoordinate);
			MarkAsNeedingAllocationIfNotFound(warpedEntryAllocationTypes, allocationBlockCoords, liveBlockIndex,
			                                  neededBlockCoordinate, targetHashEntries);
		}
	}

	void operator()(TVoxelCanonical& voxel, Vector3i voxelPosition) {
		if (voxel.flags != ITMLib::VOXEL_NONTRUNCATED) return; // skip truncated voxels
		Vector3f warpedPosition = voxelPosition.toFloat() + voxel.warp;
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
		MarkEntryForAllocationIfTrue(true, originalNeededBlockCoordinate);
		MarkEntryForAllocationIfTrue(needToAllocateExtraX, originalNeededBlockCoordinate + Vector3s(1, 0, 0));
		MarkEntryForAllocationIfTrue(needToAllocateExtraY, originalNeededBlockCoordinate + Vector3s(0, 1, 0));
		MarkEntryForAllocationIfTrue(needToAllocateExtraZ, originalNeededBlockCoordinate + Vector3s(0, 0, 1));
		MarkEntryForAllocationIfTrue(needToAllocateCornerXY, originalNeededBlockCoordinate + Vector3s(1, 1, 0));
		MarkEntryForAllocationIfTrue(needToAllocateCornerYZ, originalNeededBlockCoordinate + Vector3s(0, 1, 1));
		MarkEntryForAllocationIfTrue(needToAllocateCornerXZ, originalNeededBlockCoordinate + Vector3s(1, 0, 1));
		MarkEntryForAllocationIfTrue(needToAllocateCornerXYZ, originalNeededBlockCoordinate + Vector3s(1, 1, 1));
	}

private:
	ITMScene<TVoxelLive, TIndex>* targetScene;
	TVoxelLive* targetVoxels;
	ITMHashEntry* targetHashEntries;
	typename TIndex::IndexCache liveCache;
	Vector3s* allocationBlockCoords;
	uchar* warpedEntryAllocationTypes;
};


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::WarpCanonicalToLive(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {
	int entryCount = TIndex::noTotalEntries;
	Vector3s* allocationBlockCoords = this->allocationBlockCoordinates->GetData(MEMORYDEVICE_CPU);
	uchar* warpedEntryAllocationTypes = this->hashEntryAllocationTypes->GetData(MEMORYDEVICE_CPU);
	memset(warpedEntryAllocationTypes, (unsigned char) 0, static_cast<size_t>(entryCount));

	WarpHashAllocationMarkerFunctor<TVoxelCanonical, TVoxelLive, TIndex> hashMarkerFunctor(liveScene,
	                                                                                       allocationBlockCoords,
	                                                                                       warpedEntryAllocationTypes);
	VoxelPositionTraversal_CPU(*canonicalScene, hashMarkerFunctor);
	AllocateHashEntriesUsingLists_CPU(liveScene, warpedEntryAllocationTypes, allocationBlockCoords, ITMLib::STABLE);

	WarpSdfDistributionFunctor<TVoxelCanonical, TVoxelLive, TIndex> distributionFunctor(liveScene);
	VoxelPositionTraversal_CPU(*canonicalScene, distributionFunctor);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::InitializeHelper(
		const ITMLib::ITMSceneParams& sceneParams) {

	PrintSettings();
	uchar* entriesAllocType = this->canonicalEntryAllocationTypes->GetData(MEMORYDEVICE_CPU);
	memset(entriesAllocType, ITMLib::STABLE, static_cast<size_t>(TIndex::noTotalEntries));
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::PrintSettings() {
	std::cout << bright_cyan << "*** ITMSceneMotionTracker_CPU Settings: ***" << reset << std::endl;
#define print_bool(something) (something ? green : red) << (something ? "true" : "false") << reset
	std::cout << "Data term enabled: " << print_bool(enableDataTerm) << std::endl;
	std::cout << "Smoothing term enabled: " << print_bool(enableSmoothingTerm) << std::endl;
	std::cout << "Level Set term enabled: " << print_bool(enableLevelSetTerm) << std::endl;
	std::cout << "Killing term enabled: " << print_bool(enableKillingTerm) << std::endl;
	std::cout << "Gradient smoothing enabled: " << print_bool(enableGradientSmoothing) << std::endl;
#undef print_bool
	std::cout << bright_cyan << "*** *********************************** ***" << reset << std::endl;
}


template<typename TVoxelWarpSource, typename TVoxelSdfSource, typename TIndex>
struct WarpBasedAllocationMarkerFunctor {
	WarpBasedAllocationMarkerFunctor(
			ITMScene<TVoxelSdfSource, TIndex>* sceneToAllocate,
			Vector3s* allocationBlockCoords,
			uchar* warpedEntryAllocationTypes) :

			sdfScene(sceneToAllocate),
			sdfVoxels(sceneToAllocate->localVBA.GetVoxelBlocks()),
			sdfHashEntries(sceneToAllocate->index.GetEntries()),
			sdfCache(),

			allocationBlockCoords(allocationBlockCoords),
			warpedEntryAllocationTypes(warpedEntryAllocationTypes){}

	void operator()(TVoxelWarpSource& voxel, Vector3i voxelPosition, Vector3s hashBlockPosition) {
		Vector3f warpedPosition = voxelPosition.toFloat() + voxel.warp;
		Vector3i warpedPositionTruncated = warpedPosition.toInt();
		// perform lookup
		int vmIndex;
		const TVoxelSdfSource& sdfVoxelAtWarp = readVoxel(sdfVoxels, sdfHashEntries, warpedPositionTruncated,
		                                                vmIndex, sdfCache);
		//_DEBUG
		if (sdfVoxelAtWarp.flags != ITMLib::VOXEL_NONTRUNCATED) return; // skip truncated voxels in raw live

		int liveBlockHash = hashIndex(hashBlockPosition);
		MarkAsNeedingAllocationIfNotFound(warpedEntryAllocationTypes, allocationBlockCoords, liveBlockHash,
		                                  hashBlockPosition, sdfHashEntries);

	}

private:
	ITMScene<TVoxelSdfSource, TIndex>* sdfScene;
	TVoxelSdfSource* sdfVoxels;
	ITMHashEntry* sdfHashEntries;
	typename TIndex::IndexCache sdfCache;


	Vector3s* allocationBlockCoords;
	uchar* warpedEntryAllocationTypes;

	//_DEBUG

};

template<typename TVoxel>
struct CompleteWarpWarpedPositionFunctor {
	inline static
	Vector3f ComputeWarpedPosition(TVoxel voxel, Vector3i voxelPosition) {
		return voxelPosition.toFloat() + voxel.warp;
	}
};

template<typename TVoxel>
struct WarpGradient0WarpedPositionFunctor {
	inline static
	Vector3f ComputeWarpedPosition(TVoxel voxel, Vector3i voxelPosition) {
		return voxelPosition.toFloat() + voxel.gradient0;
	}
};

template<typename TVoxelWarpSource, typename TVoxelSdf, typename TIndex, typename WarpedPositionFunctor>
struct TrilinearInterpolationFunctor {
	/**
	 * \brief Initialize to transfer data from source sdf scene to a target sdf scene using the warps in the warp source scene
	 * \details traverses
	 * \param sdfSourceScene
	 * \param warpSourceScene
	 */
	TrilinearInterpolationFunctor(
			ITMScene<TVoxelSdf, TIndex>* sdfSourceScene,
			ITMScene<TVoxelWarpSource, TIndex>* warpSourceScene,
			int sourceSdfIndex, int targetSdfIndex) :

			sdfSourceScene(sdfSourceScene),
			sdfSourceVoxels(sdfSourceScene->localVBA.GetVoxelBlocks()),
			sdfSourceHashEntries(sdfSourceScene->index.GetEntries()),
			sdfSourceCache(),

			warpSourceScene(warpSourceScene),
			warpSourceVoxels(warpSourceScene->localVBA.GetVoxelBlocks()),
			warpSourceHashEntries(warpSourceScene->index.GetEntries()),
			warpSourceCache(),
			sourceSdfIndex(sourceSdfIndex),
			targetSdfIndex(targetSdfIndex){}


	void operator()(TVoxelSdf& destinationVoxel, Vector3i warpAndDestionVoxelPosition) {
		int vmIndex;
		// perform lookup at current position in canonical
		const TVoxelWarpSource& warpSourceVoxel = readVoxel(warpSourceVoxels, warpSourceHashEntries,
		                                                    warpAndDestionVoxelPosition, vmIndex, warpSourceCache);

		Vector3f warpedPosition = WarpedPositionFunctor::ComputeWarpedPosition(warpSourceVoxel,
		                                                                       warpAndDestionVoxelPosition);
		bool struckKnown;
		float sdf = InterpolateMultiSdfTrilinearly_StruckKnown(
				sdfSourceVoxels, sdfSourceHashEntries, warpedPosition, sourceSdfIndex, sdfSourceCache, struckKnown);

		destinationVoxel.sdf_values[targetSdfIndex] = TVoxelSdf::floatToValue(sdf);

		if (struckKnown) {
			if (1.0f - std::abs(sdf) < FLT_EPSILON) {
				destinationVoxel.flags = ITMLib::VOXEL_TRUNCATED;
			} else {
				destinationVoxel.flags = ITMLib::VOXEL_NONTRUNCATED;
			}
		}
	}

private:
	ITMScene<TVoxelSdf, TIndex>* sdfSourceScene;
	TVoxelSdf* sdfSourceVoxels;
	ITMHashEntry* sdfSourceHashEntries;
	typename TIndex::IndexCache sdfSourceCache;

	ITMScene<TVoxelWarpSource, TIndex>* warpSourceScene;
	TVoxelWarpSource* warpSourceVoxels;
	ITMHashEntry* warpSourceHashEntries;
	typename TIndex::IndexCache warpSourceCache;

	const int sourceSdfIndex;
	const int targetSdfIndex;
};

/**
 * \brief Uses trilinear interpolation of the raw live frame at canonical voxel positions augmented with their warps to
 *  generate a new SDF grid in the target scene. Intended to be used for initialization of new live frame before
 *  optimization.
 * \details Assumes target frame is empty / has been reset.
 * Does 3 things:
 * <ol>
 *  <li> Traverses allocated canonical hash blocks and voxels, checks raw frame at their warped positions (truncated),
 *       if there is a non-truncated voxel in the raw frame, marks the block in target sdf at the current canonical hash
 *       block position for allocation
 *  <li> Traverses target hash blocks, if one is marked for allocation, allocates it
 *  <li> Traverses allocated target hash blocks and voxels, retrieves canonical voxel at same location, if it is marked
 *       as "truncated", skips it, otherwise uses trilinear interpolation at [current voxel position + warp vector] to
 *       retrieve SDF value from raw live frame, then stores it into the current target voxel, and marks latter voxel as
 *       truncated, non-truncated, or unknown based on the lookup flags & resultant SDF value.
 * </ol>
 * \param canonicalScene canonical scene with warp vectors at each voxel.
 * \param liveScene source raw live scene
 * \param initializedLiveScene target live scene
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ApplyWarpFieldToLive(ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {

	AllocateHashBlocksAtWarpedLocations(canonicalScene, liveScene);

	//Do trilinear interpolation to compute voxel values
	//The "iteration" argument is 1 to use 0 as the source index and 1 as the target
	TrilinearInterpolationFunctor<TVoxelCanonical, TVoxelLive, TIndex, CompleteWarpWarpedPositionFunctor<TVoxelCanonical>>
			trilinearInterpolationFunctor(liveScene, canonicalScene, 0, 1);

	VoxelPositionTraversal_CPU(*liveScene, trilinearInterpolationFunctor);
}

/**
 * \brief Uses trilinear interpolation of the live frame at [canonical voxel positions + scaled engergy gradient]
 *  (not complete warps) to generate a new live SDF grid in the target scene. Intended to be used at every iteration.
 * \details Assumes target frame is empty / has been reset.
 * Does 3 things:
 * <ol>
 *  <li> Traverses allocated canonical hash blocks and voxels, checks raw frame at [],
 *       if there is a non-truncated voxel in the live frame, marks the block in target sdf at the current canonical
 *       hash block position for allocation
 *  <li> Traverses target hash blocks, if one is marked for allocation, allocates it
 *  <li> Traverses allocated target hash blocks and voxels, retrieves canonical voxel at same location, if it is marked
 *       as "truncated", skips it, otherwise uses trilinear interpolation at [current voxel position + warp vector] to
 *       retrieve SDF value from live frame, then stores it into the current target voxel, and marks latter voxel as
 *       truncated, non-truncated, or unknown based on the lookup flags & resultant SDF value.
 * </ol>
 * \param canonicalScene canonical scene with warp vectors at each voxel.
 * \param liveScene source (current iteration) live scene
 * \param targetLiveScene target (next iteration) live scene
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ApplyWarpUpdateToLive(ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {

	AllocateHashBlocksAtWarpedLocations(canonicalScene, liveScene);
	const int iteration = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::iteration;
	const int sourceSdfIndex = (iteration+1) % 2;
	const int targetSdfIndex = iteration % 2;
	//Do trilinear interpolation to compute voxel values
	TrilinearInterpolationFunctor<TVoxelCanonical, TVoxelLive, TIndex,
			WarpGradient0WarpedPositionFunctor<TVoxelCanonical>>
			trilinearInterpolationFunctor(liveScene, canonicalScene, sourceSdfIndex, targetSdfIndex);
	VoxelPositionTraversal_CPU(*liveScene, trilinearInterpolationFunctor);
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::AllocateHashBlocksAtWarpedLocations(
		ITMScene<TVoxelCanonical, TIndex>* warpSourceScene, ITMScene<TVoxelLive, TIndex>* sdfScene) {

	int entryCount = TIndex::noTotalEntries;
	Vector3s* allocationBlockCoords = this->allocationBlockCoordinates->GetData(MEMORYDEVICE_CPU);
	uchar* warpedEntryAllocationTypes = this->hashEntryAllocationTypes->GetData(MEMORYDEVICE_CPU);

	//reset allocation types
	memset(warpedEntryAllocationTypes, (unsigned char) 0, static_cast<size_t>(entryCount));

	//Mark up hash entries in the target scene that will need allocation
	WarpBasedAllocationMarkerFunctor<TVoxelCanonical, TVoxelLive, TIndex>
			hashMarkerFunctor(sdfScene, allocationBlockCoords, warpedEntryAllocationTypes);
	VoxelAndHashBlockPositionTraversal_CPU(*warpSourceScene, hashMarkerFunctor);

	//Allocate the hash entries that will potentially have any data
	AllocateHashEntriesUsingLists_CPU(sdfScene, warpedEntryAllocationTypes, allocationBlockCoords,
	                                  ITMLib::STABLE);
}








