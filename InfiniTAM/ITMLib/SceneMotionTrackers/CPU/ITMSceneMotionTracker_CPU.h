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
#pragma once

#include <opencv2/core/mat.hpp>
#include "../Interface/ITMSceneMotionTracker.h"
#include "../../Utils/ITMHashBlockProperties.h"
#include "ITMCalculateWarpGradientFunctor.h"


namespace ITMLib {


//region ======================================== VOXEL BLOCK HASH =====================================================

template<typename TVoxelCanonical, typename TVoxelLive>
class ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash> :
		public ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash> {
public:
	explicit ITMSceneMotionTracker_CPU(const ITMLibSettings* settings, ITMDynamicFusionLogger<TVoxelLive,TVoxelCanonical,ITMVoxelBlockHash>& logger);
	virtual ~ITMSceneMotionTracker_CPU() = default;

	void CalculateWarpGradient(
			ITMScene <TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene, ITMScene <TVoxelLive, ITMVoxelBlockHash>* liveScene,
			bool hasFocusCoordinates, const Vector3i& focusCoordinates,
			ITMSceneLogger <TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>* sceneLogger, int sourceFieldIndex,
			bool restrictZTrackingForDebugging, std::ofstream& energy_stat_file) override;
	void SmoothWarpGradient(
			ITMScene <TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene, ITMScene <TVoxelLive, ITMVoxelBlockHash>* liveScene,
			int sourceSdfIndex) override;
	float ApplyWarpUpdateToWarp(
			ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene, ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene) override;


private:
	float ApplyWarpUpdateToWarp_SingleThreadedVerbose(
			ITMScene <TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene, ITMScene <TVoxelLive, ITMVoxelBlockHash>* liveScene,
			int sourceSdfIndex);

	ITMDynamicHashManagementEngine_CPU<TVoxelCanonical,TVoxelLive> hashManager;
	ITMCalculateWarpGradientFunctor<TVoxelCanonical, TVoxelLive> calculateGradientFunctor;

};

// endregion ===========================================================================================================
//region ======================================== PLAIN VOXEL ARRAY ====================================================

template<typename TVoxelCanonical, typename TVoxelLive>
class ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>:
		public ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray> {
public:
	explicit ITMSceneMotionTracker_CPU(
			const ITMLibSettings* settings, ITMDynamicFusionLogger<TVoxelLive,TVoxelCanonical,ITMPlainVoxelArray>& logger);
	virtual ~ITMSceneMotionTracker_CPU() = default;

	void CalculateWarpGradient(
			ITMScene <TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene, ITMScene <TVoxelLive, ITMPlainVoxelArray>* liveScene,
			bool hasFocusCoordinates, const Vector3i& focusCoordinates,
			ITMSceneLogger <TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>* sceneLogger, int sourceFieldIndex,
			bool restrictZTrackingForDebugging, std::ofstream& energy_stat_file) override;
	void SmoothWarpGradient(
			ITMScene <TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene, ITMScene <TVoxelLive, ITMPlainVoxelArray>* liveScene,
			int sourceSdfIndex) override;
	float ApplyWarpUpdateToWarp(
			ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene, ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene) override;

};

// endregion ===========================================================================================================

}//namespace ITMLib


