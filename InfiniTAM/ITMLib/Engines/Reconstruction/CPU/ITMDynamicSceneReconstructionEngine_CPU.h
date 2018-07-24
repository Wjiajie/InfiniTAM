// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../SceneMotionTrackers/Interface/ITMSceneMotionTracker.h"
#include "../Interface/ITMDynamicSceneReconstructionEngine.h"
#include "../../../Objects/Scene/ITMPlainVoxelArray.h"
#include "../../Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "ITMDynamicHashManagementEngine_CPU.h"


namespace ITMLib {
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMDynamicSceneReconstructionEngine_CPU
		: public ITMDynamicSceneReconstructionEngine<TVoxelCanonical, TVoxelLive, TIndex> {
};

template<typename TVoxelCanonical, typename TVoxelLive>
class ITMDynamicSceneReconstructionEngine_CPU<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>
		: public ITMDynamicSceneReconstructionEngine<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash> {
public:
	ITMDynamicSceneReconstructionEngine_CPU() = default;
	~ITMDynamicSceneReconstructionEngine_CPU() = default;
	void UpdateVisibleList(ITMScene<TVoxelLive, ITMVoxelBlockHash>* scene, const ITMView* view,
	                       const ITMTrackingState* trackingState, const ITMRenderState* renderState,
	                       bool resetVisibleList) override;
	void GenerateRawLiveSceneFromView(ITMScene<TVoxelLive, ITMVoxelBlockHash>* scene, const ITMView* view,
	                                  const ITMTrackingState* trackingState,
	                                  const ITMRenderState* renderState) override;
	void FuseLiveIntoCanonicalSdf(ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene,
	                              ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene,
	                              int liveSourceFieldIndex) override;
	void WarpScene(ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene,
	               ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene, int sourceSdfIndex, int targetSdfIndex) override;
	void UpdateWarpedScene(ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene,
	                       ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene, int sourceSdfIndex,
	                       int targetSdfIndex) override;
	void CopyIndexedScene(ITMScene <TVoxelLive, ITMVoxelBlockHash>* liveScene, int sourceSdfIndex, int targetSdfIndex) override;
protected:
	void IntegrateIntoScene(ITMScene<TVoxelLive, ITMVoxelBlockHash>* scene, const ITMView* view,
	                        const ITMTrackingState* trackingState, const ITMRenderState* renderState);


private:
	ITMDynamicHashManagementEngine_CPU<TVoxelCanonical, TVoxelLive> hashManager;
	ITMSceneManipulationEngine_CPU<TVoxelLive, ITMVoxelBlockHash> liveSceneManager;

};

template<typename TVoxelCanonical, typename TVoxelLive>
class ITMDynamicSceneReconstructionEngine_CPU<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>
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
	void WarpScene(ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene,
	               ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene, int sourceSdfIndex,
	               int targetSdfIndex) override;
	void CopyIndexedScene(ITMScene <TVoxelLive, ITMPlainVoxelArray>* liveScene, int sourceSdfIndex, int targetSdfIndex) override;
	void UpdateWarpedScene(ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene,
	                       ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene, int sourceSdfIndex,
	                       int targetSdfIndex) override;

	ITMDynamicSceneReconstructionEngine_CPU() = default;
	~ITMDynamicSceneReconstructionEngine_CPU() = default;
protected:
	void IntegrateIntoScene(ITMScene<TVoxelLive, ITMPlainVoxelArray>* scene, const ITMView* view,
	                        const ITMTrackingState* trackingState, const ITMRenderState* renderState);
private:
	ITMSceneManipulationEngine_CPU<TVoxelLive, ITMPlainVoxelArray> liveSceneManager;
};
}
