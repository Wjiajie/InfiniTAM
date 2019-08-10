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


template<typename TVoxel, typename TWarp>
ITMSceneMotionTracker_CPU<TVoxel, TWarp, ITMVoxelBlockHash>::ITMSceneMotionTracker_CPU()
		:ITMSceneMotionTracker<TVoxel, TWarp, ITMVoxelBlockHash>(),
		 calculateGradientFunctor(this->parameters, this->switches) {};
// endregion ============================== END CONSTRUCTORS AND DESTRUCTORS============================================

// region ===================================== HOUSEKEEPING ===========================================================


template<typename TVoxel, typename TWarp>
void ITMSceneMotionTracker_CPU<TVoxel, TWarp, ITMVoxelBlockHash>::ResetWarps(
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* canonicalScene) {
	ITMSceneTraversalEngine<TVoxel, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::template
	StaticVoxelTraversal<WarpClearFunctor<TWarp, TWarp::hasCumulativeWarp>>(canonicalScene);
};


template<typename TVoxel, typename TWarp>
void ITMSceneMotionTracker_CPU<TVoxel, TWarp, ITMVoxelBlockHash>::ClearOutFlowWarp(
		ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField) {
	ITMSceneTraversalEngine<TWarp, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::template
	StaticVoxelTraversal<ClearOutFlowWarpStaticFunctor<TWarp>>(warpField);
}


// endregion ===========================================================================================================

//_DEBUG
template<typename TVoxel, typename TIndex>
inline static void PrintSceneStatistics(
		ITMVoxelVolume<TVoxel, TIndex>* scene,
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

template<typename TVoxel, typename TWarp>
void
ITMSceneMotionTracker_CPU<TVoxel, TWarp, ITMVoxelBlockHash>::CalculateWarpGradient(
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* canonicalScene,
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* liveScene,
		ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField,
		bool restrictZTrackingForDebugging) {
	ITMSceneTraversalEngine<TWarp, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::template
	StaticVoxelTraversal<ClearOutGradientStaticFunctor<TWarp>>(warpField);
	hashManager.AllocateCanonicalFromLive(canonicalScene, liveScene);
	calculateGradientFunctor.PrepareForOptimization(liveScene, canonicalScene, warpField,
	                                                restrictZTrackingForDebugging);

	ITMDualSceneWarpTraversalEngine<TVoxel, TWarp, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::
	DualVoxelPositionTraversal(liveScene, canonicalScene, warpField, calculateGradientFunctor);

	calculateGradientFunctor.FinalizePrintAndRecordStatistics();
}

// endregion ===========================================================================================================
// region ========================================== SOBOLEV GRADIENT SMOOTHING ========================================


template<typename TVoxel, typename TWarp>
void ITMSceneMotionTracker_CPU<TVoxel, TWarp, ITMVoxelBlockHash>::SmoothWarpGradient(
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* liveScene,
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* canonicalScene,
		ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField) {

	if (this->switches.enableGradientSmoothing) {
		SmoothWarpGradient_common<TVoxel, TWarp, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>
				(liveScene, canonicalScene, warpField);
	}
}

// endregion ===========================================================================================================

// region ============================= UPDATE FRAMEWISE & GLOBAL (CUMULATIVE) WARPS ===================================
template<typename TVoxel, typename TWarp>
float ITMSceneMotionTracker_CPU<TVoxel, TWarp, ITMVoxelBlockHash>::UpdateWarps(
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* canonicalScene,
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* liveScene,
		ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField) {
	return UpdateWarps_common<TVoxel, TWarp, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>(
			canonicalScene, liveScene, warpField, this->parameters.gradientDescentLearningRate,
			this->switches.enableGradientSmoothing);
}

template<typename TVoxel, typename TWarp>
void ITMSceneMotionTracker_CPU<TVoxel, TWarp, ITMVoxelBlockHash>::AddFlowWarpToWarp(
		ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField, bool clearFlowWarp) {
	AddFlowWarpToWarp_common<TWarp, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>(warpField, clearFlowWarp);
}

//endregion ============================================================================================================