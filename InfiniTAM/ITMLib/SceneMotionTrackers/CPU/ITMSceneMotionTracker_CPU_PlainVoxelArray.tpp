//  ================================================================
//  Created by Gregory Kramida on 5/28/18.
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

#include <iomanip>
#include "ITMSceneMotionTracker_CPU.h"
#include "../../Utils/ITMVoxelFlags.h"
#include "../../Utils/ITMPrintHelpers.h"
#include "../../Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../../Engines/Manipulation/CPU/ITMSceneTraversal_CPU_PlainVoxelArray.h"
#include "../Shared/ITMSceneMotionTracker_Functors.h"
#include "../Shared/ITMSceneMotionTracker_Debug.h"

using namespace ITMLib;


// region ================================ CONSTRUCTORS AND DESTRUCTORS ================================================

template<typename TVoxelCanonical, typename TVoxelLive>
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::ITMSceneMotionTracker_CPU()
		: ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>(),
		  calculateGradientFunctor(this->parameters, this->switches) {
}
// endregion ============================== END CONSTRUCTORS AND DESTRUCTORS============================================
// region ===================================== CALCULATE GRADIENT SMOOTHING ===========================================

template<typename TVoxelCanonical, typename TVoxelLive>
void
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::CalculateWarpGradient(
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene,
		ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene, int sourceFieldIndex,
		bool restrictZTrackingForDebugging) {
	ITMSceneTraversalEngine<TVoxelCanonical, ITMPlainVoxelArray, ITMLibSettings::DEVICE_CPU>::template
	StaticVoxelTraversal<ClearOutGradientStaticFunctor<TVoxelCanonical>>(canonicalScene);
	calculateGradientFunctor.PrepareForOptimization(liveScene, canonicalScene, sourceFieldIndex,
	                                                restrictZTrackingForDebugging);
	ITMDualSceneTraversalEngine<TVoxelLive, TVoxelCanonical, ITMPlainVoxelArray, ITMLibSettings::DEVICE_CPU>::
	DualVoxelPositionTraversal(liveScene, canonicalScene, calculateGradientFunctor);

	calculateGradientFunctor.FinalizePrintAndRecordStatistics();
}

// endregion ===========================================================================================================
// region ========================================== SOBOLEV GRADIENT SMOOTHING ========================================


template<typename TVoxelCanonical, typename TVoxelLive>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::SmoothWarpGradient(
		ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene,
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene,
		int sourceFieldIndex) {

	if (this->switches.enableGradientSmoothing) {
		SmoothWarpGradient_common<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray, ITMLibSettings::DEVICE_CPU>
		        (liveScene, canonicalScene, sourceFieldIndex);
	}
}

// endregion ===========================================================================================================

// region ======================================== APPLY WARP UPDATE TO THE WARP ITSELF ================================


template<typename TVoxelCanonical, typename TVoxelLive>
float ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::UpdateWarps(
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene,
		ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene, int sourceSdfIndex) {
	return UpdateWarps_common<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray, ITMLibSettings::DEVICE_CPU>
			(canonicalScene, liveScene, sourceSdfIndex, this->parameters.gradientDescentLearningRate,
			 this->switches.enableGradientSmoothing);
}


//endregion ============================================================================================================


template<typename TVoxelCanonical, typename TVoxelLive>
void ITMLib::ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::ResetWarps(
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene) {

	ITMSceneTraversalEngine<TVoxelCanonical, ITMPlainVoxelArray, ITMLibSettings::DEVICE_CPU>::template
	StaticVoxelTraversal<WarpClearFunctor<TVoxelCanonical, TVoxelCanonical::hasCumulativeWarp>>(canonicalScene);
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::ClearOutFlowWarp(
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene) {
	ITMSceneTraversalEngine<TVoxelCanonical, ITMPlainVoxelArray, ITMLibSettings::DEVICE_CPU>::template
	StaticVoxelTraversal<ClearOutFlowWarpStaticFunctor<TVoxelCanonical>>(canonicalScene);
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::AddFlowWarpToWarp(
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene, bool clearFlowWarp) {

	AddFlowWarpToWarp_common<TVoxelCanonical, ITMPlainVoxelArray, ITMLibSettings::DEVICE_CPU>
		(canonicalScene, clearFlowWarp);
};