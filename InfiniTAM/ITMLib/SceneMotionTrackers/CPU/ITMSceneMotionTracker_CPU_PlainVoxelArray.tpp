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
#include "../../Objects/Scene/ITMSceneManipulation.h"
#include "../../Objects/Scene/ITMSceneTraversal.h"
#include "../Shared/ITMSceneMotionTracker_Debug.h"

using namespace ITMLib;


// region ================================ CONSTRUCTORS AND DESTRUCTORS ================================================

template<typename TVoxelCanonical, typename TVoxelLive>
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::ITMSceneMotionTracker_CPU(
		const ITMLibSettings* settings, ITMDynamicFusionLogger<TVoxelLive,TVoxelCanonical,ITMPlainVoxelArray>& logger)
		: ITMSceneMotionTracker(settings, logger){
}
// endregion ============================== END CONSTRUCTORS AND DESTRUCTORS============================================
// region ===================================== CALCULATE GRADIENT SMOOTHING ===========================================

template<typename TVoxelCanonical>
struct ClearOutGradientStaticFunctor {
	static void run(TVoxelCanonical& voxel) {
		voxel.gradient0 = Vector3f(0.0f);
		voxel.gradient1 = Vector3f(0.0f);
	}
};

template<typename TVoxelCanonical, typename TVoxelLive> void
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::CalculateWarpGradient(
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene, ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene,
		bool hasFocusCoordinates, const Vector3i& focusCoordinates,
		ITMSceneLogger<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>* sceneLogger, int sourceFieldIndex,
		bool restrictZTrackingForDebugging, std::ofstream& energy_stat_file) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

// endregion ===========================================================================================================
// region ========================================== SOBOLEV GRADIENT SMOOTHING ========================================

enum TraversalDirection : int {
	X = 0, Y = 1, Z = 2
};


template<typename TVoxelCanonical, typename TVoxelLive>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::SmoothWarpGradient(
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene, ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene,
		int sourceSdfIndex) {

	if (this->switches.enableGradientSmoothing) {
		DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
	}
}

// endregion ===========================================================================================================

// region ======================================== APPLY WARP UPDATE TO THE WARP ITSELF ================================


template<typename TVoxelCanonical, typename TVoxelLive>
float ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::ApplyWarpUpdateToWarp(
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene, ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene) {

	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");

};

//endregion ============================================================================================================

