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
template<typename TVoxel, typename TWarp, typename TIndex>
class ITMDynamicSceneReconstructionEngine_CUDA
		: public ITMDynamicSceneReconstructionEngine<TVoxel, TVoxel, TIndex> {
};

//region =================================== VOXEL BLOCK HASH ==========================================================

template<typename TVoxel, typename TWarp>
class ITMDynamicSceneReconstructionEngine_CUDA<TVoxel, TWarp, ITMVoxelBlockHash>
		: public ITMDynamicSceneReconstructionEngine<TVoxel, TWarp, ITMVoxelBlockHash> {
public:
	ITMDynamicSceneReconstructionEngine_CUDA() = default;
	~ITMDynamicSceneReconstructionEngine_CUDA() = default;
	void UpdateVisibleList(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, const ITMView* view,
	                       const ITMTrackingState* trackingState, const ITMRenderState* renderState,
	                       bool resetVisibleList) override;
	void GenerateRawLiveSceneFromView(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, const ITMView* view,
	                                  const ITMTrackingState* trackingState,
	                                  const ITMRenderState* renderState) override;
	void FuseLiveIntoCanonicalSdf(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* canonicalScene,
	                              ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* liveScene) override;
	void WarpScene_CumulativeWarps(ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField,
	                               ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* sourceTSDF,
	                               ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* targetTSDF) override;
	void WarpScene_FlowWarps(ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField,
	                         ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* sourceTSDF,
	                         ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* targetTSDF) override;
	void WarpScene_WarpUpdates(ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField,
	                           ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* sourceTSDF,
	                           ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* targetTSDF) override;
	void CopyScene(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* sourceTSDF,
	               ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* targetTSDF) override;
protected:
	void IntegrateIntoScene(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, const ITMView* view,
	                        const ITMTrackingState* trackingState, const ITMRenderState* renderState);
	template<WarpType TWarpSource>
	void WarpScene(ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField,
	               ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* sourceTSDF,
	               ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* targetTSDF);

private:
	//ITMDynamicHashManagementEngine_CUDA<TVoxel, TWarp> hashManager;
	ITMSceneManipulationEngine_CUDA<TVoxel, ITMVoxelBlockHash> liveSceneManager;
};

// endregion ===========================================================================================================
// region ==================================== PLAIN VOXEL ARRAY =======================================================

template<typename TVoxel, typename TWarp>
class ITMDynamicSceneReconstructionEngine_CUDA<TVoxel, TWarp, ITMPlainVoxelArray>
		: public ITMDynamicSceneReconstructionEngine<TVoxel, TWarp, ITMPlainVoxelArray> {
public:
	void UpdateVisibleList(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, const ITMView* view,
	                       const ITMTrackingState* trackingState, const ITMRenderState* renderState,
	                       bool resetVisibleList) override;
	void GenerateRawLiveSceneFromView(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, const ITMView* view,
	                                  const ITMTrackingState* trackingState,
	                                  const ITMRenderState* renderState) override;
	void FuseLiveIntoCanonicalSdf(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* canonicalScene,
	                              ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* liveScene) override;
	void WarpScene_CumulativeWarps(
			ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField,
			ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* sourceTSDF,
			ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* targetTSDF) override;
	void WarpScene_FlowWarps(
			ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField,
			ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* sourceTSDF,
			ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* targetTSDF) override;
	void WarpScene_WarpUpdates(
			ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField,
			ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* sourceTSDF,
			ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* targetTSDF) override;

	void CopyScene(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* sourceTSDF,
	               ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* targetTSDF) override;

	ITMDynamicSceneReconstructionEngine_CUDA() = default;
	~ITMDynamicSceneReconstructionEngine_CUDA() = default;
protected:
	void IntegrateIntoScene(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, const ITMView* view,
	                        const ITMTrackingState* trackingState, const ITMRenderState* renderState);
private:
	ITMSceneManipulationEngine_CUDA<TVoxel, ITMPlainVoxelArray> liveSceneManager;

	template<WarpType TWarpSource>
	void WarpScene(ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField,
	               ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* sourceTSDF,
	               ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* targetTSDF);
};

// endregion ===========================================================================================================

}//namespace ITMLib