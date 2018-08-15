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
#include "../Shared/ITMCalculateWarpGradientBasedOnWarpedLiveFunctor.h"
#include "../../Engines/Reconstruction/CPU/ITMDynamicHashManagementEngine_CPU.tpp"


namespace ITMLib {

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMSceneMotionTracker_CPU:
		public ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>{
	ITMSceneMotionTracker_CPU() {}
};


//region ======================================== VOXEL BLOCK HASH =====================================================

template<typename TVoxelCanonical, typename TVoxelLive>
class ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash> :
		public ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash> {
public:
	explicit ITMSceneMotionTracker_CPU();
	virtual ~ITMSceneMotionTracker_CPU() = default;

	void ClearOutFlowWarp(ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene) override;
	void AddFlowWarpToWarp(
			ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene, bool clearFlowWarp) override;
	void CalculateWarpGradient(ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene,
		                           ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene, int sourceFieldIndex,
		                           bool restrictZTrackingForDebugging) override;
	void SmoothWarpGradient(
			ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene,
			ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene, int sourceFieldIndex) override;
	float UpdateWarps(
			ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene,
			ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene, int sourceSdfIndex) override;
	void ResetWarps(ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene) override;


private:

	ITMDynamicHashManagementEngine_CPU<TVoxelCanonical, TVoxelLive> hashManager;
	ITMCalculateWarpGradientBasedOnWarpedLiveFunctor<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash> calculateGradientFunctor;


};

// endregion ===========================================================================================================
//region ======================================== PLAIN VOXEL ARRAY ====================================================

template<typename TVoxelCanonical, typename TVoxelLive>
class ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray> :
		public ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray> {
public:
	explicit ITMSceneMotionTracker_CPU();
	virtual ~ITMSceneMotionTracker_CPU() = default;

	void ClearOutFlowWarp(ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene) override;
	void AddFlowWarpToWarp(
			ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene, bool clearFlowWarp) override;
	void CalculateWarpGradient(ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene,
		                           ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene, int sourceFieldIndex,
		                           bool restrictZTrackingForDebugging) override;
	void SmoothWarpGradient(ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene,
	                        ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene,
	                        int sourceFieldIndex) override;
	float UpdateWarps(
			ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene,
			ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene, int sourceSdfIndex) override;

	void ResetWarps(ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene) override;

private:
	ITMCalculateWarpGradientBasedOnWarpedLiveFunctor<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray> calculateGradientFunctor;

};

// endregion ===========================================================================================================

}//namespace ITMLib


