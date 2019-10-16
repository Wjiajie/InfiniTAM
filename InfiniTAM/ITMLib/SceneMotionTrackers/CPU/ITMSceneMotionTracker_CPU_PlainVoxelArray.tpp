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
#include "../../Engines/Traversal/CPU/ITMSceneTraversal_CPU_PlainVoxelArray.h"
#include "../Shared/ITMSceneMotionTracker_Functors.h"

using namespace ITMLib;


// region ================================ CONSTRUCTORS AND DESTRUCTORS ================================================

template<typename TVoxel, typename TWarp, typename TGradientFunctor>
ITMSceneMotionTracker_CPU<TVoxel, TWarp, TGradientFunctor, ITMPlainVoxelArray>::ITMSceneMotionTracker_CPU()
		: ITMSceneMotionTracker<TVoxel, TWarp, ITMPlainVoxelArray>()
		  {
}
// endregion ============================== END CONSTRUCTORS AND DESTRUCTORS============================================
// region ===================================== CALCULATE GRADIENT SMOOTHING ===========================================


template<typename TVoxel, typename TWarp, typename TGradientFunctor>
void ITMSceneMotionTracker_CPU<TVoxel, TWarp, TGradientFunctor, ITMPlainVoxelArray>::CalculateWarpGradient(
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* canonicalScene,
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* liveScene,
		ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField,
		bool restrictZTrackingForDebugging) {

	ITMSceneTraversalEngine<TWarp, ITMPlainVoxelArray, ITMLibSettings::DEVICE_CPU>::template
	StaticVoxelTraversal<ClearOutGradientStaticFunctor<TWarp>>(warpField);


	TGradientFunctor
			calculateGradientFunctor(this->parameters, this->switches,
			                         liveScene->localVBA.GetVoxelBlocks(), liveScene->index.getIndexData(),
			                         canonicalScene->localVBA.GetVoxelBlocks(), canonicalScene->index.getIndexData(),
			                         warpField->localVBA.GetVoxelBlocks(), warpField->index.getIndexData());

	ITMDualSceneWarpTraversalEngine<TVoxel, TWarp, ITMPlainVoxelArray, ITMLibSettings::DEVICE_CPU>::
	DualVoxelPositionTraversal(liveScene, canonicalScene, warpField, calculateGradientFunctor);

	calculateGradientFunctor.PrintStatistics();
}


// endregion ===========================================================================================================
// region ========================================== SOBOLEV GRADIENT SMOOTHING ========================================


template<typename TVoxel, typename TWarp, typename TGradientFunctor>
void ITMSceneMotionTracker_CPU<TVoxel, TWarp, TGradientFunctor, ITMPlainVoxelArray>::SmoothWarpGradient(
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* canonicalScene,
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* liveScene,
		ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField) {

	if (this->switches.enableGradientSmoothing) {
		SmoothWarpGradient_common<TVoxel, TWarp, ITMPlainVoxelArray, ITMLibSettings::DEVICE_CPU>
		        (liveScene, canonicalScene, warpField);
	}
}

// endregion ===========================================================================================================

// region ======================================== APPLY WARP UPDATE TO THE WARP ITSELF ================================


template<typename TVoxel, typename TWarp, typename TGradientFunctor>
float ITMSceneMotionTracker_CPU<TVoxel, TWarp, TGradientFunctor, ITMPlainVoxelArray>::UpdateWarps(
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* canonicalScene,
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* liveScene,
		ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField) {
	return UpdateWarps_common<TVoxel, TWarp, ITMPlainVoxelArray, ITMLibSettings::DEVICE_CPU>
			(canonicalScene, liveScene, warpField, this->parameters.gradientDescentLearningRate,
			 this->switches.enableGradientSmoothing);
}


//endregion ============================================================================================================


template<typename TVoxel, typename TWarp, typename TGradientFunctor>
void ITMLib::ITMSceneMotionTracker_CPU<TVoxel, TWarp, TGradientFunctor, ITMPlainVoxelArray>::ResetWarps(
		ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField) {

	ITMSceneTraversalEngine<TWarp, ITMPlainVoxelArray, ITMLibSettings::DEVICE_CPU>::template
	StaticVoxelTraversal<WarpClearFunctor<TWarp, TWarp::hasCumulativeWarp>>(warpField);
}

template<typename TVoxel, typename TWarp, typename TGradientFunctor>
void ITMSceneMotionTracker_CPU<TVoxel, TWarp, TGradientFunctor, ITMPlainVoxelArray>::ClearOutFlowWarp(
		ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField) {
	ITMSceneTraversalEngine<TWarp, ITMPlainVoxelArray, ITMLibSettings::DEVICE_CPU>::template
	StaticVoxelTraversal<ClearOutFlowWarpStaticFunctor<TWarp>>(warpField);
}

template<typename TVoxel, typename TWarp, typename TGradientFunctor>
void ITMSceneMotionTracker_CPU<TVoxel, TWarp, TGradientFunctor, ITMPlainVoxelArray>::AddFlowWarpToWarp(
		ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField, bool clearFlowWarp) {

	AddFlowWarpToWarp_common<TWarp, ITMPlainVoxelArray, ITMLibSettings::DEVICE_CPU>
		(warpField, clearFlowWarp);
}

