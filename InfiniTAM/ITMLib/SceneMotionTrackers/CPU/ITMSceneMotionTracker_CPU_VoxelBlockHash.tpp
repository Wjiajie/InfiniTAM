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

#include "../Shared/ITMSceneMotionTracker_Functors.h"

#include "../../Utils/Analytics/ITMSceneStatisticsCalculator.h"
#include "../../Objects/Scene/ITMTrilinearDistribution.h"
#include "../../Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../../Utils/ITMLibSettings.h"
#include "../../Engines/Manipulation/CPU/ITMSceneTraversal_CPU_VoxelBlockHash.h"


using namespace ITMLib;

// region ================================ CONSTRUCTORS AND DESTRUCTORS ================================================


template<typename TVoxelCanonical, typename TVoxelLive>
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::ITMSceneMotionTracker_CPU()
		:ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>(),
		 calculateGradientFunctor(this->parameters, this->switches) {};
// endregion ============================== END CONSTRUCTORS AND DESTRUCTORS============================================

// region ===================================== HOUSEKEEPING ===========================================================


template<typename TVoxelCanonical, typename TVoxelLive>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::ResetWarps(
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene) {
	ITMSceneTraversalEngine<TVoxelCanonical, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::template
	StaticVoxelTraversal<WarpClearFunctor<TVoxelCanonical, TVoxelCanonical::hasGlobalWarp>>(canonicalScene);
};


template<typename TVoxelCanonical, typename TVoxelLive>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::ClearOutFramewiseWarp(
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene) {
	ITMSceneTraversalEngine<TVoxelCanonical, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::template
	StaticVoxelTraversal<ClearOutFramewiseWarpStaticFunctor<TVoxelCanonical>>(canonicalScene);
}


// endregion ===========================================================================================================

//_DEBUG
template<typename TVoxel, typename TIndex>
inline static void PrintSceneStatistics(
		ITMScene<TVoxel, TIndex>* scene,
		std::string description) {
	ITMSceneStatisticsCalculator<TVoxel, TIndex>& calculator = ITMSceneStatisticsCalculator<TVoxel, TIndex>::Instance();
	std::cout << green << "=== Stats for scene '" << description << "' ===" << reset << std::endl;
	std::cout << "    Total voxel count: " << calculator.ComputeAllocatedVoxelCount(scene) << std::endl;
	std::cout << "    NonTruncated voxel count: " << calculator.ComputeNonTruncatedVoxelCount(scene) << std::endl;
	std::cout << "    +1.0 voxel count: " << calculator.ComputeVoxelWithValueCount(scene, 1.0f) << std::endl;
	std::vector<int> allocatedHashes = calculator.GetFilledHashBlockIds(scene);
	std::cout << "    Allocated hash count: " << allocatedHashes.size() << std::endl;
	std::cout << "    NonTruncated SDF sum: " << calculator.ComputeNonTruncatedVoxelAbsSdfSum(scene) << std::endl;
	std::cout << "    Truncated SDF sum: " << calculator.ComputeTruncatedVoxelAbsSdfSum(scene) << std::endl;
};

// region ===================================== CALCULATE GRADIENT SMOOTHING ===========================================

template<typename TVoxelCanonical, typename TVoxelLive>
void
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::CalculateWarpGradient(
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene,
		ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene, int sourceFieldIndex,
		bool restrictZTrackingForDebugging) {
	ITMSceneTraversalEngine<TVoxelCanonical, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::template
	StaticVoxelTraversal<ClearOutGradientStaticFunctor<TVoxelCanonical>>(canonicalScene);
	hashManager.AllocateCanonicalFromLive(canonicalScene, liveScene);
	calculateGradientFunctor.PrepareForOptimization(liveScene, canonicalScene, sourceFieldIndex,
	                                                restrictZTrackingForDebugging);

	ITMDualSceneTraversalEngine<TVoxelLive, TVoxelCanonical, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::
	DualVoxelPositionTraversal(liveScene, canonicalScene, calculateGradientFunctor);

	calculateGradientFunctor.FinalizePrintAndRecordStatistics();
}

// endregion ===========================================================================================================
// region ========================================== SOBOLEV GRADIENT SMOOTHING ========================================


template<typename TVoxelCanonical, typename TVoxelLive>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::SmoothWarpGradient(
		ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene,
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene, int sourceFieldIndex) {

	if (this->switches.enableGradientSmoothing) {
		SmoothWarpGradient_common<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>
				(liveScene, canonicalScene, sourceFieldIndex);
	}
}

// endregion ===========================================================================================================

// region ============================= UPDATE FRAMEWISE & GLOBAL (CUMULATIVE) WARPS ===================================
template<typename TVoxelCanonical, typename TVoxelLive>
float ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::UpdateWarps(
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene,
		ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene, int sourceSdfIndex) {
	return UpdateWarps_common<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>(
			canonicalScene, liveScene, sourceSdfIndex, this->parameters.gradientDescentLearningRate,
			this->switches.enableGradientSmoothing);
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::AddFramewiseWarpToWarp(
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene, bool clearFramewiseWarp) {

	AddFramewiseWarpToWarp_common<TVoxelCanonical, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>
		(canonicalScene, clearFramewiseWarp);
}

//endregion ============================================================================================================