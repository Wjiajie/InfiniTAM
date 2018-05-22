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
#include <opencv/cv.hpp>

//local
#include "ITMSceneMotionTracker_CPU.h"
#include "ITMCalculateWarpGradientFunctor.tpp"
#include "../Shared/ITMSceneMotionTracker_Shared_Old.h"
#include "../../Utils/ITMSceneStatisticsCalculator.h"
#include "../../Objects/Scene/ITMTrilinearDistribution.h"
#include "../../Objects/Scene/ITMSceneManipulation.h"
#include "../../Utils/ITMLibSettings.h"
#include "../../Objects/Scene/ITMSceneTraversal.h"


using namespace ITMLib;

// region ================================ CONSTRUCTORS AND DESTRUCTORS ================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ITMSceneMotionTracker_CPU(
		const ITMLibSettings* settings)

		: ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>(settings),

		  hashEntryAllocationTypes(new ORUtils::MemoryBlock<unsigned char>(TIndex::noTotalEntries, MEMORYDEVICE_CPU)),
		  canonicalEntryAllocationTypes(
				  new ORUtils::MemoryBlock<unsigned char>(TIndex::noTotalEntries, MEMORYDEVICE_CPU)),
		  allocationBlockCoordinates(new ORUtils::MemoryBlock<Vector3s>(TIndex::noTotalEntries, MEMORYDEVICE_CPU)),

		  switches{
				  settings->enableDataTerm,
				  settings->enableLevelSetTerm,
				  settings->enableSmoothingTerm,
				  settings->enableKillingTerm,
				  settings->enableGradientSmoothing
		  }{
	InitializeHelper(settings->sceneParams);

}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::~ITMSceneMotionTracker_CPU() {
	delete hashEntryAllocationTypes;
	delete canonicalEntryAllocationTypes;
	delete allocationBlockCoordinates;

}

// endregion ============================== END CONSTRUCTORS AND DESTRUCTORS============================================

// region =========================== CANONICAL HASH BLOCK ALLOCATION AT EACH FRAME ====================================
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
	const int& currentFrameIx = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::trackedFrameCount;
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
	const int iteration = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::iteration;
	int targetSdfIndex = iteration % 2;


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
				const int currentFrameIx = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::trackedFrameCount;
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
					if (liveVoxel.flag_values[targetSdfIndex] != ITMLib::VOXEL_NONTRUNCATED) {
						continue;
					}
					TVoxelCanonical& canonicalVoxel = localCanonicalVoxelBlock[locId];
					int oldWDepth = canonicalVoxel.w_depth;
					float oldSdf = TVoxelCanonical::valueToFloat(canonicalVoxel.sdf);

					float liveSdf = TVoxelLive::valueToFloat(liveVoxel.sdf_values[targetSdfIndex]);

					float newSdf = oldWDepth * oldSdf + liveSdf;
					float newWDepth = oldWDepth + 1.0f;
					newSdf /= newWDepth;
					newWDepth = MIN(newWDepth, maximumWeight);

					canonicalVoxel.sdf = TVoxelCanonical::floatToValue(newSdf);
					canonicalVoxel.w_depth = (uchar) newWDepth;

					canonicalVoxel.flags = ITMLib::VOXEL_NONTRUNCATED;
					entriesAllocType[hash] = ITMLib::STABLE;
				}
			}
		}
	}
}

// endregion ===========================================================================================================

// region ======================= OLD FUNCTIONS FOR WARPING CANONICAL TO LIVE VIA DISTRIBUTION =========================

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
	VoxelPositionTraversal_CPU(canonicalScene, hashMarkerFunctor);
	AllocateHashEntriesUsingLists_CPU(liveScene, warpedEntryAllocationTypes, allocationBlockCoords, ITMLib::STABLE);

	WarpSdfDistributionFunctor<TVoxelCanonical, TVoxelLive, TIndex> distributionFunctor(liveScene);
	VoxelPositionTraversal_CPU(canonicalScene, distributionFunctor);
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
	std::cout << "Data term enabled: " << print_bool(this->switches.enableDataTerm) << std::endl;
	std::cout << "Smoothing term enabled: " << print_bool(this->switches.enableSmoothingTerm) << std::endl;
	std::cout << "Level Set term enabled: " << print_bool(this->switches.enableLevelSetTerm) << std::endl;
	std::cout << "Killing term enabled: " << print_bool(this->switches.enableKillingTerm) << std::endl;
	std::cout << "Gradient smoothing enabled: " << print_bool(this->switches.enableGradientSmoothing) << std::endl
	          << std::endl;

	std::cout << "Max iteration count: " << this->parameters.maxIterationCount << std::endl;
	std::cout << "Warp vector update threshold: " << this->parameters.maxVectorUpdateThresholdMeters << " m"
																									 << std::endl;
	std::cout << "Gradient descent learning rate: " << this->parameters.gradientDescentLearningRate << std::endl;
	std::cout << "Rigidity enforcement factor: " << this->parameters.rigidityEnforcementFactor << std::endl;
	std::cout << "Weight of the data term: " << this->parameters.weightDataTerm << std::endl;
	std::cout << "Weight of the smoothness term: " << this->parameters.weightSmoothnessTerm << std::endl;
	std::cout << "Weight of the level set term: " << this->parameters.weightLevelSetTerm << std::endl;
	std::cout << "Epsilon for the level set term: " << this->parameters.epsilon << std::endl;
	std::cout << "Unity scaling factor: " << this->parameters.unity << std::endl;
#undef print_bool
	std::cout << bright_cyan << "*** *********************************** ***" << reset << std::endl;
}

// endregion ===========================================================================================================

// region ===================================== APPLY WARP/UPDATE TO LIVE ==============================================


template<typename TVoxelMulti>
struct IndexedFieldClearFunctor {
	IndexedFieldClearFunctor(int flagFieldIndex) : flagFieldIndex(flagFieldIndex) {}

	void operator()(TVoxelMulti& voxel) {
		voxel.flag_values[flagFieldIndex] = ITMLib::VOXEL_UNKNOWN;
		voxel.sdf_values[flagFieldIndex] = TVoxelMulti::SDF_initialValue();
	}

private:
	const int flagFieldIndex;
};

template<typename TVoxel>
struct WarpClearFunctor {
	static void run(TVoxel& voxel) {
		voxel.warp = Vector3f(0.0f);
	}
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ClearOutWarps(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene) {
	StaticVoxelTraversal_CPU<WarpClearFunctor<TVoxelCanonical>>(canonicalScene);
};


template<typename TVoxelWarpSource, typename TVoxelSdfSource, typename TIndex>
struct WarpBasedAllocationMarkerFunctor {
	WarpBasedAllocationMarkerFunctor(
			ITMScene<TVoxelSdfSource, TIndex>* sceneToAllocate,
			Vector3s* allocationBlockCoords,
			uchar* warpedEntryAllocationTypes,
			int flagIndex) :

			sdfScene(sceneToAllocate),
			sdfVoxels(sceneToAllocate->localVBA.GetVoxelBlocks()),
			sdfHashEntries(sceneToAllocate->index.GetEntries()),
			sdfCache(),

			allocationBlockCoords(allocationBlockCoords),
			warpedEntryAllocationTypes(warpedEntryAllocationTypes),

			flagIndex(flagIndex) {}

	void operator()(TVoxelWarpSource& voxel, Vector3i voxelPosition, Vector3s hashBlockPosition) {
		Vector3f warpedPosition = voxelPosition.toFloat() + voxel.gradient0;
		Vector3i warpedPositionTruncated = warpedPosition.toInt();
		// perform lookup
		int vmIndex;
		const TVoxelSdfSource& sdfVoxelAtWarp = readVoxel(sdfVoxels, sdfHashEntries, warpedPositionTruncated,
		                                                  vmIndex, sdfCache);
		// skip truncated voxels in raw/old live
		if (sdfVoxelAtWarp.flag_values[flagIndex] != ITMLib::VOXEL_NONTRUNCATED) return;

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
	const int flagIndex;
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
			targetSdfIndex(targetSdfIndex) {}


	void operator()(TVoxelSdf& destinationVoxel, Vector3i warpAndDestionVoxelPosition) {
		int vmIndex;
		// perform lookup at current position in canonical
		const TVoxelWarpSource& warpSourceVoxel = readVoxel(warpSourceVoxels, warpSourceHashEntries,
		                                                    warpAndDestionVoxelPosition, vmIndex, warpSourceCache);

		Vector3f warpedPosition = WarpedPositionFunctor::ComputeWarpedPosition(warpSourceVoxel,
		                                                                       warpAndDestionVoxelPosition);
		bool struckKnown, struckNonTruncated;
		float cumulativeWeight;
		float sdf = InterpolateMultiSdfTrilinearly_StruckKnown(
				sdfSourceVoxels, sdfSourceHashEntries, warpedPosition, sourceSdfIndex, sdfSourceCache, struckKnown);

		destinationVoxel.sdf_values[targetSdfIndex] = TVoxelSdf::floatToValue(sdf);

		// Update flags
		if (struckKnown) {
			if (1.0f - std::abs(sdf) < FLT_EPSILON) {
				destinationVoxel.flag_values[targetSdfIndex] = ITMLib::VOXEL_TRUNCATED;
			} else {
				destinationVoxel.flag_values[targetSdfIndex] = ITMLib::VOXEL_NONTRUNCATED;
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
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ApplyWarpFieldToLive(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {

	const int sourceSdfIndex = ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::GetSourceLiveSdfIndex(
			this->iteration);
	const int targetSdfIndex = ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::GetTargetLiveSdfIndex(
			this->iteration);

	// Clear out the flags at target index
	IndexedFieldClearFunctor<TVoxelLive> flagClearFunctor(targetSdfIndex);
	VoxelTraversal_CPU(*liveScene, flagClearFunctor);

	// Allocate new hash blocks due to warps where necessary, use flags from raw frame
	AllocateHashBlocksAtWarpedLocations(canonicalScene, liveScene, sourceSdfIndex);

	// Always use 0 as the source index and 1 as the target, since raw frame SDF φ^{i}_{proj} is initialized in
	// the 0-set of fields
	TrilinearInterpolationFunctor<TVoxelCanonical, TVoxelLive, TIndex, CompleteWarpWarpedPositionFunctor<TVoxelCanonical>>
			trilinearInterpolationFunctor(liveScene, canonicalScene, sourceSdfIndex, targetSdfIndex);
	// Do trilinear interpolation to compute voxel values
	VoxelPositionTraversal_CPU(liveScene, trilinearInterpolationFunctor);
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
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ApplyWarpUpdateToLive(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {

	const int sourceSdfIndex = ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::GetSourceLiveSdfIndex(
			this->iteration);
	const int targetSdfIndex = ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::GetTargetLiveSdfIndex(
			this->iteration);

	// Clear out the flags at target index
	IndexedFieldClearFunctor<TVoxelLive> flagClearFunctor(targetSdfIndex);
	VoxelTraversal_CPU(*liveScene, flagClearFunctor);

	// Allocate new blocks where necessary, filter based on flags from source
	AllocateHashBlocksAtWarpedLocations(canonicalScene, liveScene, sourceSdfIndex);

	TrilinearInterpolationFunctor<TVoxelCanonical, TVoxelLive, TIndex,
			WarpGradient0WarpedPositionFunctor<TVoxelCanonical>>
			trilinearInterpolationFunctor(liveScene, canonicalScene, sourceSdfIndex, targetSdfIndex);

	// Interpolate to obtain the new live frame values (at target index)
	VoxelPositionTraversal_CPU(liveScene, trilinearInterpolationFunctor);
}

/**
 * \brief Helper method which looks at voxel grid with warps and an SDF voxel grid and allocates all hash blocks in the
 * SDF grid where warp vectors are pointing to (if not already allocated).
 * \details scans each (allocated) voxel in the SDF voxel grid, checks the warp vector at the corresponding location,
 * finds the voxel where the warp vector is pointing to, and, if the hash block for that voxel is not yet allocated,
 * allocates it.
 * \param warpSourceScene voxel grid where each voxel has a .warp Vector3f field defined
 * \param sdfScene sdf grid whose hash blocks to allocate if needed
 * \param fieldIndex index of the sdf / flag field to use in the sdfScene
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::AllocateHashBlocksAtWarpedLocations(
		ITMScene<TVoxelCanonical, TIndex>* warpSourceScene, ITMScene<TVoxelLive, TIndex>* sdfScene, int fieldIndex) {

	int entryCount = TIndex::noTotalEntries;
	Vector3s* allocationBlockCoords = this->allocationBlockCoordinates->GetData(MEMORYDEVICE_CPU);
	uchar* warpedEntryAllocationTypes = this->hashEntryAllocationTypes->GetData(MEMORYDEVICE_CPU);

	//reset allocation types
	memset(warpedEntryAllocationTypes, (unsigned char) 0, static_cast<size_t>(entryCount));

	//Mark up hash entries in the target scene that will need allocation
	WarpBasedAllocationMarkerFunctor<TVoxelCanonical, TVoxelLive, TIndex>
			hashMarkerFunctor(sdfScene, allocationBlockCoords, warpedEntryAllocationTypes, fieldIndex);
	VoxelAndHashBlockPositionTraversal_CPU(*warpSourceScene, hashMarkerFunctor);

	//Allocate the hash entries that will potentially have any data
	AllocateHashEntriesUsingLists_CPU(sdfScene, warpedEntryAllocationTypes, allocationBlockCoords,
	                                  ITMLib::STABLE);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
int ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::GetSourceLiveSdfIndex(int iteration) {
	return iteration % 2;
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
int ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::GetTargetLiveSdfIndex(int iteration) {
	return (iteration + 1) % 2;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const int ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::rawLiveFrameSdfIndex = 0;

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const int ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::firstInitializedLiveFrameSdfIndex = 1;


// endregion ===========================================================================================================

// region ===================================== CALCULATE GRADIENT SMOOTHING ===========================================
		
template<typename TVoxelCanonical>
struct ClearOutGradientStaticFunctor {
	static void run(TVoxelCanonical& voxel) {
		voxel.gradient0 = Vector3f(0.0f);
		voxel.gradient1 = Vector3f(0.0f);
	}
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::CalculateWarpGradient(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
		ITMScene<TVoxelLive, TIndex>* liveScene) {

	StaticVoxelTraversal_CPU<ClearOutGradientStaticFunctor<TVoxelCanonical>>(canonicalScene);
	//TODO: initialize this struct in the constructor of ITMSceneMotionTracker_CPU  and adjust to recompute sourceSdfIndex based on the iteration on the fly (via separate member function) -Greg (GitHub: Algomorph)
	CalculateWarpGradientFunctor<TVoxelCanonical, TVoxelLive, TIndex> calculateGradientFunctor(
			liveScene, canonicalScene, this->parameters, this->switches, this->iteration, this->trackedFrameCount,
			this->hasFocusCoordinates, this->focusCoordinates, this->sceneLogger);
	calculateGradientFunctor.restrictZtrackingForDebugging = this->restrictZtrackingForDebugging;

	DualVoxelPositionTraversal_AllocateSecondaryOnMiss_CPU(
			liveScene, canonicalScene, this->canonicalEntryAllocationTypes, calculateGradientFunctor, true);

	calculateGradientFunctor.FinalizePrintAndRecordStatistics(this->energy_stat_file);
}

// endregion ===========================================================================================================
// region ========================================== SOBOLEV GRADIENT SMOOTHING ========================================

enum TraversalDirection : int {
	X = 0, Y = 1, Z = 2
};


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex, TraversalDirection TDirection>
struct GradientSmoothingPassFunctor {
	GradientSmoothingPassFunctor(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                             ITMScene<TVoxelLive, TIndex>* liveScene,
	                             int sourceIndex) :
			canonicalScene(canonicalScene),
			canonicalVoxels(canonicalScene->localVBA.GetVoxelBlocks()),
			canoincalHashEntries(canonicalScene->index.GetEntries()),
			canonicalCache(),

			liveScene(liveScene),
			liveVoxels(liveScene->localVBA.GetVoxelBlocks()),
			liveHashEntries(liveScene->index.GetEntries()),
			liveCache(),
			sourceIndex(sourceIndex) {}

	void operator()(TVoxelCanonical& voxel, Vector3i position) {
		int vmIndex;
		const TVoxelLive& liveVoxel = readVoxel(liveVoxels, liveHashEntries, position, vmIndex, liveCache);

		const int directionIndex = (int) TDirection;

		Vector3i receptiveVoxelPosition = position;
		receptiveVoxelPosition[directionIndex] -= (sobolevFilterSize / 2);
		Vector3f smoothedGradient(0.0f);

		for (int iVoxel = 0; iVoxel < sobolevFilterSize; iVoxel++, receptiveVoxelPosition[directionIndex]++) {
			const TVoxelCanonical& receptiveVoxel = readVoxel(canonicalVoxels, canoincalHashEntries,
			                                                  receptiveVoxelPosition, vmIndex, canonicalCache);
			smoothedGradient += sobolevFilter1D[iVoxel] * GetGradient(receptiveVoxel);
		}
		SetGradient(voxel, smoothedGradient);
	}

private:
	Vector3f GetGradient(const TVoxelCanonical& voxel) const {
		switch (TDirection) {
			case X:
				return voxel.gradient0;
			case Y:
				return voxel.gradient1;
			case Z:
				return voxel.gradient0;
		}
	}

	void SetGradient(TVoxelCanonical& voxel, const Vector3f gradient) const {
		switch (TDirection) {
			case X:
				voxel.gradient1 = gradient;
				return;
			case Y:
				voxel.gradient0 = gradient;
				return;
			case Z:
				voxel.gradient1 = gradient;
				return;
		}
	}

	ITMScene<TVoxelCanonical, TIndex>* canonicalScene;
	TVoxelCanonical* canonicalVoxels;
	ITMHashEntry* canoincalHashEntries;
	typename TIndex::IndexCache canonicalCache;

	ITMScene<TVoxelLive, TIndex>* liveScene;
	TVoxelLive* liveVoxels;
	ITMHashEntry* liveHashEntries;
	typename TIndex::IndexCache liveCache;

	const int sourceIndex;

	static const int sobolevFilterSize;
	static const float sobolevFilter1D[];
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex, TraversalDirection TDirection>
const int GradientSmoothingPassFunctor<TVoxelCanonical, TVoxelLive, TIndex, TDirection>::sobolevFilterSize = 7;
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex, TraversalDirection TDirection>
const float GradientSmoothingPassFunctor<TVoxelCanonical, TVoxelLive, TIndex, TDirection>::sobolevFilter1D[] = {
		2.995861099047703036e-04f,
		4.410932423926419363e-03f,
		6.571314272194948847e-02f,
		9.956527876693953560e-01f,
		6.571314272194946071e-02f,
		4.410932423926422832e-03f,
		2.995861099045313996e-04f};

//_DEBUG -- normalized version
//template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex, TraversalDirection TDirection>
//const float GradientSmoothingPassFunctor<TVoxelCanonical, TVoxelLive, TIndex, TDirection>::sobolevFilter1D[] = {
//		2.636041325812907461e-04f,
//		3.881154276361719040e-03f,
//		5.782062280706985746e-02f,
//		8.760692375679742794e-01f,
//		5.782062280706985746e-02f,
//		3.881154276361719040e-03f,
//		2.636041325812907461e-04f};



template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ApplySmoothingToGradient(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {

	int sourceIndex = GetSourceLiveSdfIndex(this->iteration);
	if (this->switches.enableGradientSmoothing) {
		GradientSmoothingPassFunctor<TVoxelCanonical, TVoxelLive, TIndex, X> passFunctorX(canonicalScene, liveScene, sourceIndex);
		GradientSmoothingPassFunctor<TVoxelCanonical, TVoxelLive, TIndex, Y> passFunctorY(canonicalScene, liveScene, sourceIndex);
		GradientSmoothingPassFunctor<TVoxelCanonical, TVoxelLive, TIndex, Z> passFunctorZ(canonicalScene, liveScene, sourceIndex);

		VoxelPositionTraversal_CPU(canonicalScene, passFunctorX);
		VoxelPositionTraversal_CPU(canonicalScene, passFunctorY);
		VoxelPositionTraversal_CPU(canonicalScene, passFunctorZ);
	}
}

// endregion ===========================================================================================================

// region ======================================== APPLY WARP UPDATE TO THE WARP ITSELF ================================
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
float ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ApplyWarpUpdateToWarp_SingleThreadedVerbose(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {

	const float learningRate = this->parameters.gradientDescentLearningRate;
	const int currentFrameIx = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::trackedFrameCount;
	const int iteration = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::iteration;
	const int sourceSdfIndex = ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::GetSourceLiveSdfIndex(iteration);

	// *** traversal vars
	// ** canonical frame
	TVoxelCanonical* canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();
	typename TIndex::IndexCache canonicalCache;
	// ** live frame
	TVoxelLive* liveVoxels = liveScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* liveHashTable = liveScene->index.GetEntries();
	int noTotalEntries = liveScene->index.noTotalEntries;
	typename TIndex::IndexCache liveCache;

	// *** stats
	float maxWarpLength = 0.0f;
	float maxWarpUpdateLength = 0.0f;
	Vector3i maxWarpPosition(0);
	Vector3i maxWarpUpdatePosition(0);

	//Apply the update
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
				stream << "Could not find corresponding canonical block at postion " << currentLiveHashEntry.pos
				       << " at frame " << currentFrameIx << ". " << __FILE__ << ": " << __LINE__;
				DIEWITHEXCEPTION(stream.str());
			}
			currentCanonicalHashEntry = canonicalHashTable[canonicalHash];
		}

		TVoxelLive* localLiveVoxelBlock = &(liveVoxels[currentLiveHashEntry.ptr * SDF_BLOCK_SIZE3]);
		TVoxelCanonical* localCanonicalVoxelBlock =
				&(canonicalVoxels[currentCanonicalHashEntry.ptr * SDF_BLOCK_SIZE3]);

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxelLive& liveVoxel = localLiveVoxelBlock[locId];
					TVoxelCanonical& canonicalVoxel = localCanonicalVoxelBlock[locId];
					Vector3f warpUpdate = -learningRate * (this->switches.enableGradientSmoothing ?
					                                       canonicalVoxel.gradient1 : canonicalVoxel.gradient0);

					canonicalVoxel.gradient0 = warpUpdate;
					canonicalVoxel.warp += warpUpdate;
					float warpLength = ORUtils::length(canonicalVoxel.warp);
					float warpUpdateLength = ORUtils::length(warpUpdate);
					if (warpLength > maxWarpLength) {
						maxWarpLength = warpLength;
						maxWarpPosition = currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE + Vector3i(x, y, z);
					}
					if (warpUpdateLength > maxWarpUpdateLength) {
						maxWarpUpdateLength = warpUpdateLength;
						maxWarpUpdatePosition =
								currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE + Vector3i(x, y, z);
					}
				}
			}
		}
	}

	//Warp Update Length Histogram
	// <20%, 40%, 60%, 80%, 100%
	const int histBinCount = 10;
	int warpBins[histBinCount] = {0};
	int updateBins[histBinCount] = {0};

	for (int hash = 0; hash < noTotalEntries; hash++) {
		const ITMHashEntry& currentLiveHashEntry = liveHashTable[hash];
		if (currentLiveHashEntry.ptr < 0) continue;
		if (currentLiveHashEntry.ptr < 0) continue;
		ITMHashEntry& currentCanonicalHashEntry = canonicalHashTable[hash];

		// the rare case where we have different positions for live & canonical voxel block with the same index:
		// we have a hash bucket miss, find the canonical voxel with the matching coordinates
		if (currentCanonicalHashEntry.pos != currentLiveHashEntry.pos) {
			int canonicalHash = hash;
			if (!FindHashAtPosition(canonicalHash, currentLiveHashEntry.pos, canonicalHashTable)) {
				std::stringstream stream;
				stream << "Could not find corresponding canonical block at postion " << currentLiveHashEntry.pos
				       << " at frame " << currentFrameIx << ". " << __FILE__ << ": " << __LINE__;
				DIEWITHEXCEPTION(stream.str());
			}
			currentCanonicalHashEntry = canonicalHashTable[canonicalHash];
		}

		TVoxelLive* localLiveVoxelBlock = &(liveVoxels[currentLiveHashEntry.ptr * SDF_BLOCK_SIZE3]);
		TVoxelCanonical* localCanonicalVoxelBlock =
				&(canonicalVoxels[currentCanonicalHashEntry.ptr * SDF_BLOCK_SIZE3]);

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxelLive& liveVoxel = localLiveVoxelBlock[locId];
					if (liveVoxel.flags != ITMLib::VOXEL_NONTRUNCATED) {
						continue;
					}
					TVoxelCanonical& canonicalVoxel = localCanonicalVoxelBlock[locId];

					float warpLength = ORUtils::length(canonicalVoxel.warp);
					float warpUpdateLength = ORUtils::length(canonicalVoxel.gradient0);
					int binIdx = 0;
					if (maxWarpLength > 0) {
						binIdx = std::min(histBinCount - 1, (int) (warpLength * histBinCount / maxWarpLength));
					}
					warpBins[binIdx]++;
					if (maxWarpUpdateLength > 0) {
						binIdx = std::min(histBinCount - 1,
						                  (int) (warpUpdateLength * histBinCount / maxWarpUpdateLength));
					}
					updateBins[binIdx]++;
				}
			}
		}
	}

	std::cout << "  Warp length histogram: ";
	for (int iBin = 0; iBin < histBinCount; iBin++) {
		std::cout << std::setfill(' ') << std::setw(7) << warpBins[iBin] << "  ";
	}
	std::cout << std::endl;
	std::cout << "Update length histogram: ";
	for (int iBin = 0; iBin < histBinCount; iBin++) {
		std::cout << std::setfill(' ') << std::setw(7) << updateBins[iBin] << "  ";
	}
	std::cout << std::endl;

	std::cout << green << "Max warp: [" << maxWarpLength << " at " << maxWarpPosition << "] Max update: ["
	          << maxWarpUpdateLength << " at " << maxWarpUpdatePosition << "]." << reset << std::endl;

	return maxWarpUpdateLength;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
float ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ApplyWarpUpdateToWarp(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {

	return ApplyWarpUpdateToWarp_SingleThreadedVerbose(canonicalScene, liveScene);

};

//endregion ============================================================================================================