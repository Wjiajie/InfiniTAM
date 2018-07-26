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


using namespace ITMLib;

template<typename TVoxelCanonical, typename TVoxelLive>
ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::ITMSceneMotionTracker_CUDA() {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::ClearOutFramewiseWarp(
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::AddFramewiseWarpToWarp(
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene, bool clearFramewiseWarp) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::CalculateWarpGradient(
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene,
		ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene, int sourceFieldIndex, bool restrictZTrackingForDebugging) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::SmoothWarpGradient(
		ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene,
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene, int sourceFieldIndex) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<typename TVoxelCanonical, typename TVoxelLive>
float ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::UpdateWarps(
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene,
		ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene, int sourceSdfIndex) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::ResetWarps(
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}
