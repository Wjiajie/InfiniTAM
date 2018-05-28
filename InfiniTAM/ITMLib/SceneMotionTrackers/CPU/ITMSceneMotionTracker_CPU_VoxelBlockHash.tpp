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

#include "../Shared/ITMSceneMotionTracker_Shared_Old.h"
#include "../../Utils/Analytics/ITMSceneStatisticsCalculator.h"
#include "../../Objects/Scene/ITMTrilinearDistribution.h"
#include "../../Objects/Scene/ITMSceneManipulation.h"
#include "../../Utils/ITMLibSettings.h"
#include "../../Objects/Scene/ITMSceneTraversal.h"


using namespace ITMLib;

// region ================================ CONSTRUCTORS AND DESTRUCTORS ================================================


template<typename TVoxelCanonical, typename TVoxelLive>
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::ITMSceneMotionTracker_CPU(
		const ITMLibSettings* settings, ITMDynamicFusionLogger<TVoxelLive, TVoxelCanonical, ITMVoxelBlockHash>& logger)
		:ITMSceneMotionTracker(settings, logger),
		 calculateGradientFunctor(this->parameters, this->switches, logger){

};
// endregion ============================== END CONSTRUCTORS AND DESTRUCTORS============================================


// endregion ===========================================================================================================


// region ===================================== CALCULATE GRADIENT SMOOTHING ===========================================

template<typename TVoxelCanonical>
struct ClearOutGradientStaticFunctor {
	static void run(TVoxelCanonical& voxel) {
		voxel.gradient0 = Vector3f(0.0f);
		voxel.gradient1 = Vector3f(0.0f);
	}
};

template<typename TVoxelCanonical, typename TVoxelLive> void
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::CalculateWarpGradient(
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene, ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene,
		bool hasFocusCoordinates, const Vector3i& focusCoordinates,
		ITMSceneLogger<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>* sceneLogger, int sourceFieldIndex,
		bool restrictZTrackingForDebugging, std::ofstream& energy_stat_file) {

	StaticVoxelTraversal_CPU<ClearOutGradientStaticFunctor<TVoxelCanonical>>(canonicalScene);
	hashManager.AllocateCanonicalFromLive(canonicalScene,liveScene);

	calculateGradientFunctor.PrepareForOptimization( liveScene, canonicalScene, sourceFieldIndex,
	                                                 hasFocusCoordinates, focusCoordinates,
	                                                 restrictZTrackingForDebugging, sceneLogger);

	DualVoxelPositionTraversal_CPU(liveScene, canonicalScene, calculateGradientFunctor);

	calculateGradientFunctor.FinalizePrintAndRecordStatistics();
}

// endregion ===========================================================================================================
// region ========================================== SOBOLEV GRADIENT SMOOTHING ========================================

enum TraversalDirection : int {
	X = 0, Y = 1, Z = 2
};


template<typename TVoxelCanonical, typename TVoxelLive, TraversalDirection TDirection>
struct GradientSmoothingPassFunctor {
	GradientSmoothingPassFunctor(ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene,
	                             ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene,
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

	ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene;
	TVoxelCanonical* canonicalVoxels;
	ITMHashEntry* canoincalHashEntries;
	typename ITMVoxelBlockHash::IndexCache canonicalCache;

	ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene;
	TVoxelLive* liveVoxels;
	ITMHashEntry* liveHashEntries;
	typename ITMVoxelBlockHash::IndexCache liveCache;

	const int sourceIndex;

	static const int sobolevFilterSize;
	static const float sobolevFilter1D[];
};

template<typename TVoxelCanonical, typename TVoxelLive, TraversalDirection TDirection>
const int GradientSmoothingPassFunctor<TVoxelCanonical, TVoxelLive, TDirection>::sobolevFilterSize = 7;
template<typename TVoxelCanonical, typename TVoxelLive, TraversalDirection TDirection>
const float GradientSmoothingPassFunctor<TVoxelCanonical, TVoxelLive, TDirection>::sobolevFilter1D[] = {
		2.995861099047703036e-04f,
		4.410932423926419363e-03f,
		6.571314272194948847e-02f,
		9.956527876693953560e-01f,
		6.571314272194946071e-02f,
		4.410932423926422832e-03f,
		2.995861099045313996e-04f};


template<typename TVoxelCanonical, typename TVoxelLive>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::SmoothWarpGradient(
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene, ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene,
		int sourceSdfIndex) {

	if (this->switches.enableGradientSmoothing) {
		GradientSmoothingPassFunctor<TVoxelCanonical, TVoxelLive, X> passFunctorX(canonicalScene, liveScene,
		                                                                                  sourceSdfIndex);
		GradientSmoothingPassFunctor<TVoxelCanonical, TVoxelLive, Y> passFunctorY(canonicalScene, liveScene,
		                                                                                  sourceSdfIndex);
		GradientSmoothingPassFunctor<TVoxelCanonical, TVoxelLive, Z> passFunctorZ(canonicalScene, liveScene,
		                                                                                  sourceSdfIndex);

		VoxelPositionTraversal_CPU(canonicalScene, passFunctorX);
		VoxelPositionTraversal_CPU(canonicalScene, passFunctorY);
		VoxelPositionTraversal_CPU(canonicalScene, passFunctorZ);
	}
}

// endregion ===========================================================================================================

// region ======================================== APPLY WARP UPDATE TO THE WARP ITSELF ================================
template<typename TVoxelCanonical, typename TVoxelLive>
float ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::ApplyWarpUpdateToWarp_SingleThreadedVerbose(
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene, ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene,
		int sourceSdfIndex) {

	const float learningRate = this->parameters.gradientDescentLearningRate;

	// *** traversal vars
	// ** canonical frame
	TVoxelCanonical* canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();
	typename ITMVoxelBlockHash::IndexCache canonicalCache;
	// ** live frame
	TVoxelLive* liveVoxels = liveScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* liveHashTable = liveScene->index.GetEntries();
	int noTotalEntries = liveScene->index.noTotalEntries;
	typename ITMVoxelBlockHash::IndexCache liveCache;

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
				       << ". " << __FILE__ << ": " << __LINE__;
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
				       << ". " << __FILE__ << ": " << __LINE__;
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

template<typename TVoxelCanonical, typename TVoxelLive>
float ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::ApplyWarpUpdateToWarp(
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene, ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene) {

	return ApplyWarpUpdateToWarp_SingleThreadedVerbose(canonicalScene, liveScene, 0);

}


//endregion ============================================================================================================