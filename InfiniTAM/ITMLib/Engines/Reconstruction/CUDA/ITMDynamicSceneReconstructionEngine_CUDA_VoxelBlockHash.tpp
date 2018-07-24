//  ================================================================
//  Created by Gregory Kramida on 7/24/18.
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

#include "ITMDynamicSceneReconstructionEngine_CUDA.h"

using namespace ITMLib;

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMDynamicSceneReconstructionEngine_CUDA<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::UpdateVisibleList(
		ITMScene<TVoxelLive, ITMVoxelBlockHash>* scene, const ITMView* view, const ITMTrackingState* trackingState,
		const ITMRenderState* renderState, bool resetVisibleList) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<typename TVoxelCanonical, typename TVoxelLive>
void
ITMDynamicSceneReconstructionEngine_CUDA<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::GenerateRawLiveSceneFromView(
		ITMScene<TVoxelLive, ITMVoxelBlockHash>* scene, const ITMView* view, const ITMTrackingState* trackingState,
		const ITMRenderState* renderState) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMDynamicSceneReconstructionEngine_CUDA<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::FuseLiveIntoCanonicalSdf(
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene,
		ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene, int liveSourceFieldIndex) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMDynamicSceneReconstructionEngine_CUDA<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::WarpScene(
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene,
		ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene, int sourceSdfIndex, int targetSdfIndex) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMDynamicSceneReconstructionEngine_CUDA<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::UpdateWarpedScene(
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene,
		ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene, int sourceSdfIndex, int targetSdfIndex) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMDynamicSceneReconstructionEngine_CUDA<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::CopyIndexedScene(
		ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene, int sourceSdfIndex, int targetSdfIndex) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMDynamicSceneReconstructionEngine_CUDA<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::IntegrateIntoScene(
		ITMScene<TVoxelLive, ITMVoxelBlockHash>* scene, const ITMView* view, const ITMTrackingState* trackingState,
		const ITMRenderState* renderState) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}
