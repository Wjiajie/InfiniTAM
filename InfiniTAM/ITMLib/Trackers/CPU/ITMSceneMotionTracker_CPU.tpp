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
	const ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();
	typename TIndex::IndexCache canonicalCache;

	const TVoxelLive* liveVoxels = liveScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* liveHashTable = liveScene->index.GetEntries();
	typename TIndex::IndexCache liveCache;

	int maximumWeight = canonicalScene->sceneParams->maxW;
	//_DEBUG
	const int currentFrameIx = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::currentFrameIx;

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
//TODO: confidence, color?
					int oldWDepth;//, oldWColor;
					float oldSdf;
					oldSdf = canonicalVoxel.sdf;
					//Vector3f oldColor = TO_FLOAT3(canonicalVoxel.clr) / 255.0f;
					oldWDepth = canonicalVoxel.w_depth; //0 for VOXEL_TRUNCATED voxels
					//oldWColor = canonicalVoxel.w_color; //0 for VOXEL_TRUNCATED voxels

					if (currentFrameIx == 4 && originalPosition == Vector3i(-12, 19, 192)) {
						int i = 42;
					}

					//projected position of the sdf point to the most recent frame
					Vector3f projectedPosition = originalPosition.toFloat() + canonicalVoxel.warp;

					Vector3f liveColor;
					float liveWeight;
					bool struckKnownVoxels;
					bool struckNonTruncatedVoxels;


					//color & sdf
//					float weightedLiveSdf = InterpolateTrilinearly_SdfColor_StruckNonTruncatedAndKnown_SmartWeights(
//							liveVoxels, liveHashTable,
//							projectedPosition,
//							liveCache, liveColor,
//							struckKnownVoxels, struckNonTruncatedVoxels, liveWeight);
					float weightedLiveSdf = InterpolateTrilinearly_Sdf_StruckNonTruncatedAndKnown_SmartWeights(
							liveVoxels, liveHashTable,
							projectedPosition,
							liveCache,
							struckKnownVoxels, struckNonTruncatedVoxels, liveWeight);

					/* *
					 * Conditions to avoid fusion:
					 * No known voxels with non-zero interpolation weights were struck in the live frame SDF by the query.
					 * */
					if (!struckKnownVoxels) {
						missedKnownVoxels++;//_DEBUG
						continue;
					}

					float newSdf = oldWDepth * oldSdf + weightedLiveSdf;
					float newWDepth = oldWDepth + liveWeight;
					newSdf /= newWDepth;
					newWDepth = MIN(newWDepth, maximumWeight);

					//TODO: this color-weighting probably is incorrect, since not all live voxels will have viable color -Greg (GitHub:Algomorph)
//					Vector3f newColor = oldWColor * oldColor + liveColor;
//					float newWColor = oldWColor + liveWeight;
//					newColor /= newWColor;
//					newWColor = MIN(newWDepth, maximumWeight);

					canonicalVoxel.sdf = TVoxelCanonical::floatToValue(newSdf);
					canonicalVoxel.w_depth = (uchar) newWDepth;
//					canonicalVoxel.clr = TO_UCHAR3(newColor);
//					canonicalVoxel.w_color = (uchar) newWColor;
					if (!struckNonTruncatedVoxels) {
						sdfTruncatedCount++;//_DEBUG
					}
					switch (canonicalVoxel.flags) {
						case ITMLib::VOXEL_UNKNOWN:
							if (struckNonTruncatedVoxels) {
								//voxel is no longer perceived as truncated
								canonicalVoxel.flags = ITMLib::VOXEL_NONTRUNCATED;
								if (entriesAllocType[hash] != ITMLib::STABLE) {
									hashBlockStabilizationCount++;
								}
								entriesAllocType[hash] = ITMLib::STABLE;
							} else {
								canonicalVoxel.flags = ITMLib::VOXEL_TRUNCATED;
							}
							break;
						case ITMLib::VOXEL_TRUNCATED:
							if (struckNonTruncatedVoxels) {
								//voxel is no longer perceived as truncated
								canonicalVoxel.flags = ITMLib::VOXEL_NONTRUNCATED;
								if (entriesAllocType[hash] != ITMLib::STABLE) {
									hashBlockStabilizationCount++;
								}
								entriesAllocType[hash] = ITMLib::STABLE;
							} else if (std::signbit(oldSdf) != std::signbit(weightedLiveSdf)) {
								//both voxels are truncated but differ in sign
								//TODO: if it is faster, optimize this to short-circuit the computation before instead -Greg (GitHub: Algomorph)
								if (liveWeight > 0.25f && weightedLiveSdf > 0.0f) {
									//set truncated to the sign of the live truncated, i.e. now belongs to different surface
									canonicalVoxel.sdf = 1.0;
									canonicalVoxel.w_depth = liveWeight;
								} else {
									canonicalVoxel.sdf = oldSdf;
									canonicalVoxel.w_depth -= liveWeight;
								}
							}
							break;
						default:
							break;
					}
					//_DEBUG
//					if (canonicalVoxel.sdf > -1.0 && canonicalVoxel.sdf < 1.0 && canonicalVoxel.flags == ITMLib::VOXEL_TRUNCATED) {
//						std::cerr << red << "Error: Voxel at " << originalPosition << ", label: "
//						          << VoxelFlagsAsCString((ITMLib::VoxelFlags) canonicalVoxel.flags) << ". Sdf value: "
//						          << canonicalVoxel.sdf << "." << reset << std::endl;
//					}
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
	this->targetLiveScene = new ITMScene<ITMVoxelLive, TIndex>(
			&sceneParams, false, MEMORYDEVICE_CPU);

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
			ITMScene<TVoxelSdfSource, TIndex>* sourceScene,
			ITMScene<TVoxelSdfSource, TIndex>* targetScene,
			Vector3s* allocationBlockCoords,
			uchar* warpedEntryAllocationTypes) :

			sourceScene(sourceScene),
			sourceVoxels(sourceScene->localVBA.GetVoxelBlocks()),
			sourceHashEntries(sourceScene->index.GetEntries()),
			sourceCache(),

			targetScene(targetScene),
			targetHashEntries(targetScene->index.GetEntries()),
			allocationBlockCoords(allocationBlockCoords),
			warpedEntryAllocationTypes(warpedEntryAllocationTypes) {}

	void operator()(TVoxelWarpSource& voxel, Vector3i voxelPosition, Vector3s hashBlockPosition) {
		Vector3f warpedPosition = voxelPosition.toFloat() + voxel.warp;
		Vector3i warpedPositionTruncated = warpedPosition.toInt();
		// perform lookup
		int vmIndex;
		const TVoxelSdfSource& sourceVoxelAtWarp = readVoxel(sourceVoxels, sourceHashEntries, warpedPositionTruncated,
		                                                vmIndex, sourceCache);
		//_DEBUG
		if (sourceVoxelAtWarp.flags != ITMLib::VOXEL_NONTRUNCATED) return; // skip truncated voxels in raw live

		int liveBlockHash = hashIndex(hashBlockPosition);
		MarkAsNeedingAllocationIfNotFound(warpedEntryAllocationTypes, allocationBlockCoords, liveBlockHash,
		                                  hashBlockPosition, targetHashEntries);

	}

private:
	ITMScene<TVoxelSdfSource, TIndex>* sourceScene;
	TVoxelSdfSource* sourceVoxels;
	ITMHashEntry* sourceHashEntries;
	typename TIndex::IndexCache sourceCache;

	ITMScene<TVoxelSdfSource, TIndex>* targetScene;
	ITMHashEntry* targetHashEntries;

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
			ITMScene<TVoxelWarpSource, TIndex>* warpSourceScene) :

			sdfSourceScene(sdfSourceScene),
			sdfSourceVoxels(sdfSourceScene->localVBA.GetVoxelBlocks()),
			sdfSourceHashEntries(sdfSourceScene->index.GetEntries()),
			sdfSourceCache(),

			warpSourceScene(warpSourceScene),
			warpSourceVoxels(warpSourceScene->localVBA.GetVoxelBlocks()),
			warpSourceHashEntries(warpSourceScene->index.GetEntries()),
			warpSourceCache() {}

	void operator()(TVoxelSdf& destinationVoxel, Vector3i warpAndDestionVoxelPosition) {
		int vmIndex;
		// perform lookup at current position in canonical
		const TVoxelWarpSource& warpSourceVoxel = readVoxel(warpSourceVoxels, warpSourceHashEntries,
		                                                    warpAndDestionVoxelPosition, vmIndex, warpSourceCache);

		Vector3f warpedPosition = WarpedPositionFunctor::ComputeWarpedPosition(warpSourceVoxel,
		                                                                       warpAndDestionVoxelPosition);
		bool struckKnown;
		float sdf = InterpolateTrilinearly_StruckKnown(sdfSourceVoxels, sdfSourceHashEntries, warpedPosition,
		                                               sdfSourceCache, struckKnown);
		destinationVoxel.sdf = TVoxelSdf::floatToValue(sdf);

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
 * \param rawLiveScene source raw live scene
 * \param initializedLiveScene target live scene
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ApplyWarpFieldToLive(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
		ITMScene<TVoxelLive, TIndex>* rawLiveScene,
		ITMScene<TVoxelLive, TIndex>* initializedLiveScene) {

	AllocateHashBlocksAtWarpedLocations(canonicalScene, rawLiveScene, initializedLiveScene);
	//Do trilinear interpolation to compute voxel values
	TrilinearInterpolationFunctor<TVoxelCanonical, TVoxelLive, TIndex, CompleteWarpWarpedPositionFunctor<TVoxelCanonical>>
			trilinearInterpolationFunctor(rawLiveScene, canonicalScene);

	VoxelPositionTraversal_CPU(*initializedLiveScene, trilinearInterpolationFunctor);
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
 * \param sourceLiveScene source (current iteration) live scene
 * \param targetLiveScene target (next iteration) live scene
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ApplyWarpUpdateToLive(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
		ITMScene<TVoxelLive, TIndex>* sourceLiveScene,
		ITMScene<TVoxelLive, TIndex>* targetLiveScene) {

	AllocateHashBlocksAtWarpedLocations(canonicalScene, sourceLiveScene, targetLiveScene);

	//Do trilinear interpolation to compute voxel values
	TrilinearInterpolationFunctor<TVoxelCanonical, TVoxelLive, TIndex, WarpGradient0WarpedPositionFunctor<TVoxelCanonical>>
			trilinearInterpolationFunctor(sourceLiveScene, canonicalScene);

	VoxelPositionTraversal_CPU(*targetLiveScene, trilinearInterpolationFunctor);
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::AllocateHashBlocksAtWarpedLocations(
		ITMScene<TVoxelCanonical, TIndex>* warpSourceScene,
		ITMScene<TVoxelLive, TIndex>* sdfSourceScene,
		ITMScene<TVoxelLive, TIndex>* sdfTargetScene) {

	int entryCount = TIndex::noTotalEntries;
	Vector3s* allocationBlockCoords = this->allocationBlockCoordinates->GetData(MEMORYDEVICE_CPU);
	uchar* warpedEntryAllocationTypes = this->hashEntryAllocationTypes->GetData(MEMORYDEVICE_CPU);

	//reset allocation types
	memset(warpedEntryAllocationTypes, (unsigned char) 0, static_cast<size_t>(entryCount));

	//Mark up hash entries in the target scene that will need allocation
	WarpBasedAllocationMarkerFunctor<TVoxelCanonical, TVoxelLive, TIndex>
			hashMarkerFunctor(sdfSourceScene, sdfTargetScene, allocationBlockCoords, warpedEntryAllocationTypes);
	VoxelAndHashBlockPositionTraversal_CPU(*warpSourceScene, hashMarkerFunctor);

	//Allocate the hash entries that will potentially have any data
	AllocateHashEntriesUsingLists_CPU(sdfTargetScene, warpedEntryAllocationTypes, allocationBlockCoords,
	                                  ITMLib::STABLE);
}








