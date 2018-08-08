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

#include "../Interface/ITMDynamicSceneReconstructionEngine.h"
#include "../../../Objects/Scene/ITMPlainVoxelArray.h"
#include "../../Manipulation/CUDA/ITMSceneManipulationEngine_CUDA.h"


namespace ITMLib {
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMDynamicSceneReconstructionEngine_CUDA
		: public ITMDynamicSceneReconstructionEngine<TVoxelCanonical, TVoxelLive, TIndex> {
};

//region =================================== VOXEL BLOCK HASH ==========================================================

template<typename TVoxelCanonical, typename TVoxelLive>
class ITMDynamicSceneReconstructionEngine_CUDA<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>
		: public ITMDynamicSceneReconstructionEngine<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash> {
public:
	ITMDynamicSceneReconstructionEngine_CUDA() = default;
	~ITMDynamicSceneReconstructionEngine_CUDA() = default;
	void UpdateVisibleList(ITMScene<TVoxelLive, ITMVoxelBlockHash>* scene, const ITMView* view,
	                       const ITMTrackingState* trackingState, const ITMRenderState* renderState,
	                       bool resetVisibleList) override;
	void GenerateRawLiveSceneFromView(ITMScene<TVoxelLive, ITMVoxelBlockHash>* scene, const ITMView* view,
	                                  const ITMTrackingState* trackingState,
	                                  const ITMRenderState* renderState) override;
	void FuseLiveIntoCanonicalSdf(ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene,
	                              ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene,
	                              int liveSourceFieldIndex) override;
	void WarpScene_CumulativeWarps(ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene,
	                               ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene, int sourceSdfIndex,
	                               int targetSdfIndex) override;
	void WarpScene_FlowWarps(ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene,
	                         ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene, int sourceSdfIndex,
	                         int targetSdfIndex) override;
	void WarpScene_WarpUpdates(ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene,
	                           ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene, int sourceSdfIndex,
	                           int targetSdfIndex) override;
	void CopyIndexedScene(ITMScene <TVoxelLive, ITMVoxelBlockHash>* liveScene, int sourceSdfIndex, int targetSdfIndex) override;
protected:
	void IntegrateIntoScene(ITMScene<TVoxelLive, ITMVoxelBlockHash>* scene, const ITMView* view,
	                        const ITMTrackingState* trackingState, const ITMRenderState* renderState);
	template<Warp TWarpSource>
	void WarpScene(ITMScene <TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene,
	               ITMScene <TVoxelLive, ITMVoxelBlockHash>* liveScene, int sourceSdfIndex, int targetSdfIndex);

private:
	//ITMDynamicHashManagementEngine_CUDA<TVoxelCanonical, TVoxelLive> hashManager;
	ITMSceneManipulationEngine_CUDA<TVoxelLive, ITMVoxelBlockHash> liveSceneManager;
};

// endregion ===========================================================================================================
// region ==================================== PLAIN VOXEL ARRAY =======================================================

template<typename TVoxelCanonical, typename TVoxelLive>
class ITMDynamicSceneReconstructionEngine_CUDA<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>
		: public ITMDynamicSceneReconstructionEngine<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray> {
public:
	void UpdateVisibleList(ITMScene<TVoxelLive, ITMPlainVoxelArray>* scene, const ITMView* view,
	                       const ITMTrackingState* trackingState, const ITMRenderState* renderState,
	                       bool resetVisibleList) override;
	void GenerateRawLiveSceneFromView(ITMScene<TVoxelLive, ITMPlainVoxelArray>* scene, const ITMView* view,
	                                  const ITMTrackingState* trackingState,
	                                  const ITMRenderState* renderState) override;
	void FuseLiveIntoCanonicalSdf(ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene,
	                              ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene,
	                              int liveSourceFieldIndex) override;
	void WarpScene_CumulativeWarps(ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene,
	                               ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene, int sourceSdfIndex,
	                               int targetSdfIndex) override;
	void WarpScene_FlowWarps(ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene,
	                         ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene, int sourceSdfIndex,
	                         int targetSdfIndex) override;
	void WarpScene_WarpUpdates(ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene,
	                           ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene, int sourceSdfIndex,
	                           int targetSdfIndex) override;

	void CopyIndexedScene(ITMScene <TVoxelLive, ITMPlainVoxelArray>* liveScene, int sourceSdfIndex, int targetSdfIndex) override;

	ITMDynamicSceneReconstructionEngine_CUDA() = default;
	~ITMDynamicSceneReconstructionEngine_CUDA() = default;
protected:
	void IntegrateIntoScene(ITMScene<TVoxelLive, ITMPlainVoxelArray>* scene, const ITMView* view,
	                        const ITMTrackingState* trackingState, const ITMRenderState* renderState);
private:
	ITMSceneManipulationEngine_CUDA<TVoxelLive, ITMPlainVoxelArray> liveSceneManager;

	template<Warp TWarpSource>
	void WarpScene(ITMScene <TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene,
	               ITMScene <TVoxelLive, ITMPlainVoxelArray>* liveScene, int sourceSdfIndex, int targetSdfIndex);
};

// endregion ===========================================================================================================

}//namespace ITMLib