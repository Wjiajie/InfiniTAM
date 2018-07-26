//  ================================================================
//  Created by Gregory Kramida on 7/26/18.
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
#pragma once

#include "ITMSceneMotionTracker_CUDA.h"
#include "../../Objects/Scene/ITMSceneTraversal_CUDA_PlainVoxelArray.h"
#include "../Shared/ITMSceneMotionTracker_Functors.h"


using namespace ITMLib;


template<typename TVoxelCanonical, typename TVoxelLive>
ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::ITMSceneMotionTracker_CUDA()
		: ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>(),
			calculateGradientFunctor(this->parameters, this->switches){

}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::ClearOutFramewiseWarp(
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene) {
	StaticVoxelTraversal<ClearOutFramewiseWarpStaticFunctor<TVoxelCanonical>>(canonicalScene);

}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::AddFramewiseWarpToWarp(
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene, bool clearFramewiseWarp) {
	AddFramewiseWarpToWarp_common(canonicalScene,clearFramewiseWarp);
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::CalculateWarpGradient(
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene,
		ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene, int sourceFieldIndex, bool restrictZTrackingForDebugging) {
	StaticVoxelTraversal<ClearOutGradientStaticFunctor<TVoxelCanonical>>(canonicalScene);
	calculateGradientFunctor.PrepareForOptimization(liveScene, canonicalScene, sourceFieldIndex,
	                                                restrictZTrackingForDebugging);

	DualVoxelPositionTraversal(liveScene, canonicalScene, calculateGradientFunctor);
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::SmoothWarpGradient(
		ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene,
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene, int sourceFieldIndex) {
	if (this->switches.enableGradientSmoothing) {
		SmoothWarpGradient_common(liveScene,canonicalScene,sourceFieldIndex);
	}
}

template<typename TVoxelCanonical, typename TVoxelLive>
float ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::UpdateWarps(
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene,
		ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene, int sourceSdfIndex) {
	return UpdateWarps_common(canonicalScene,liveScene,sourceSdfIndex,
	                          this->parameters.gradientDescentLearningRate,this->switches.enableGradientSmoothing);
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::ResetWarps(
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene) {
	StaticVoxelTraversal<WarpClearFunctor<TVoxelCanonical>>(canonicalScene);
}
