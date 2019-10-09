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
#include "../Shared/ITMSceneMotionTracker_WarpCalculationFunctors.h"
#include "../../Engines/Reconstruction/CPU/ITMHashAllocationEngine_CPU.tpp"


namespace ITMLib {

template<typename TVoxel, typename TWarp, typename TIndex>
class ITMSceneMotionTracker_CPU:
		public ITMSceneMotionTracker<TVoxel, TWarp, TIndex>{
	ITMSceneMotionTracker_CPU() {}
};


//region ======================================== VOXEL BLOCK HASH =====================================================

template<typename TVoxel, typename TWarp>
class ITMSceneMotionTracker_CPU<TVoxel, TWarp, ITMVoxelBlockHash> :
		public ITMSceneMotionTracker<TVoxel, TWarp, ITMVoxelBlockHash> {
public:
	explicit ITMSceneMotionTracker_CPU();
	virtual ~ITMSceneMotionTracker_CPU() = default;

	void ClearOutFlowWarp(ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField) override;
	void AddFlowWarpToWarp(
			ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField, bool clearFlowWarp) override;
	void CalculateWarpGradient(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* canonicalScene,
	                           ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* liveScene,
	                           ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField,
	                           bool restrictZTrackingForDebugging) override;
	void SmoothWarpGradient(
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* canonicalScene,
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* liveScene,
			ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField) override;

	float UpdateWarps(
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* canonicalScene,
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* liveScene,
			ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField) override;
	void ResetWarps(ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField) override;


private:

	ITMHashAllocationEngine_CPU<TVoxel, TWarp> hashManager;
	ITMCalculateWarpGradientBasedOnWarpedLiveFunctor<TVoxel, TWarp, ITMVoxelBlockHash> calculateGradientFunctor;


};

// endregion ===========================================================================================================
//region ======================================== PLAIN VOXEL ARRAY ====================================================

//TODO: reorder argument lists in both classes for consistency with reconstruction engine: warp field should come first,
//  canonical (as the "target") should come last

template<typename TVoxel, typename TWarp>
class ITMSceneMotionTracker_CPU<TVoxel, TWarp, ITMPlainVoxelArray> :
		public ITMSceneMotionTracker<TVoxel, TWarp, ITMPlainVoxelArray> {
public:
	explicit ITMSceneMotionTracker_CPU();
	virtual ~ITMSceneMotionTracker_CPU() = default;

	void ClearOutFlowWarp(ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField) override;
	void AddFlowWarpToWarp(
			ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField, bool clearFlowWarp) override;
	void CalculateWarpGradient(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* canonicalScene,
	                           ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* liveScene,
	                           ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField,
	                           bool restrictZTrackingForDebugging) override;
	void SmoothWarpGradient(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* canonicalScene,
	                        ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* liveScene,
	                        ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField) override;
	float UpdateWarps(
			ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* canonicalScene,
			ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* liveScene,
			ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField) override;

	void ResetWarps(ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField) override;

private:
	ITMCalculateWarpGradientBasedOnWarpedLiveFunctor<TVoxel, TWarp, ITMPlainVoxelArray> calculateGradientFunctor;

};

// endregion ===========================================================================================================

}//namespace ITMLib


