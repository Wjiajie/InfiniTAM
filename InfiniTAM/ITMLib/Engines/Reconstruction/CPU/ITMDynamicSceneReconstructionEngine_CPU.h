// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../SurfaceTrackers/Interface/SurfaceTrackerInterface.h"
#include "../Interface/ITMDynamicSceneReconstructionEngine.h"
#include "../../../Objects/Scene/ITMPlainVoxelArray.h"
#include "../../Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../../Indexing/VBH/CPU/ITMIndexingEngine_CPU_VoxelBlockHash.h"
#include "../../Common/ITMWarpEnums.h"


namespace ITMLib {
template<typename TVoxel, typename TWarp, typename TIndex>
class ITMDynamicSceneReconstructionEngine_CPU
		: public ITMDynamicSceneReconstructionEngine<TVoxel, TWarp, TIndex> {
};

template<typename TVoxel, typename TWarp>
class ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, ITMVoxelBlockHash>
		: public ITMDynamicSceneReconstructionEngine<TVoxel, TWarp, ITMVoxelBlockHash> {
public:
	ITMDynamicSceneReconstructionEngine_CPU() = default;
	~ITMDynamicSceneReconstructionEngine_CPU() = default;

	void UpdateVisibleList(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, const ITMView* view,
	                       const ITMTrackingState* trackingState, const ITMRenderState* renderState,
	                       bool resetVisibleList) override;
	void GenerateTsdfVolumeFromView(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, const ITMView* view,
	                                const ITMTrackingState* trackingState) override;
	void GenerateTsdfVolumeFromView(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, const ITMView* view,
	                                const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;
	void GenerateTsdfVolumeFromViewExpanded(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* volume,
	                                        ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* temporaryAllocationVolume,
	                                        const ITMView* view,
	                                        const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;
	void FuseOneTsdfVolumeIntoAnother(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* canonicalScene,
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

	void IntegrateDepthImageIntoTsdfVolume(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* volume, const ITMView* view);
	void IntegrateDepthImageIntoTsdfVolume(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* volume, const ITMView* view,
	                                      const ITMTrackingState* trackingState);
private:
	void IntegrateDepthImageIntoTsdfVolume_Helper(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* volume, const ITMView* view,
	                                             Matrix4f depth_camera_matrix = Matrix4f::Identity());
	template<WarpType TWarpType>
	void WarpScene(ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField,
	               ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* sourceTSDF,
	               ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* targetTSDF);
	ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>& sceneManager = ITMSceneManipulationEngine_CPU<TVoxel, ITMVoxelBlockHash>::Inst();

};

template<typename TVoxel, typename TWarp>
class ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, ITMPlainVoxelArray>
		: public ITMDynamicSceneReconstructionEngine<TVoxel, TWarp, ITMPlainVoxelArray> {
public:
	void UpdateVisibleList(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, const ITMView* view,
	                       const ITMTrackingState* trackingState, const ITMRenderState* renderState,
	                       bool resetVisibleList) override;
	void GenerateTsdfVolumeFromView(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, const ITMView* view,
	                                const ITMTrackingState* trackingState) override;
	void GenerateTsdfVolumeFromView(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, const ITMView* view,
	                                const Matrix4f& depth_camera_matrix=Matrix4f::Identity()) override;
	void GenerateTsdfVolumeFromViewExpanded(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* volume,
	                                        ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* temporaryAllocationVolume,
	                                        const ITMView* view,
	                                        const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;
	void FuseOneTsdfVolumeIntoAnother(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* canonicalScene,
	                              ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* liveScene) override;

	void WarpScene_CumulativeWarps(ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField,
	                               ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* sourceTSDF,
	                               ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* targetTSDF) override;
	void WarpScene_FlowWarps(ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField,
	                         ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* sourceTSDF,
	                         ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* targetTSDF) override;
	void WarpScene_WarpUpdates(ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField,
	                           ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* sourceTSDF,
	                           ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* targetTSDF) override;
	void IntegrateDepthImageIntoTsdfVolume(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* volume, const ITMView* view);
	void IntegrateDepthImageIntoTsdfVolume(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* volume, const ITMView* view,
	                                      const ITMTrackingState* trackingState);

	ITMDynamicSceneReconstructionEngine_CPU() = default;
	~ITMDynamicSceneReconstructionEngine_CPU() = default;
private:
	void IntegrateDepthImageIntoTsdfVolume_Helper(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* volume, const ITMView* view,
	                                             Matrix4f camera_depth_matrix = Matrix4f::Identity());
	template<WarpType TWarpType>
	void WarpScene(ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField,
	               ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* sourceTSDF,
	               ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* targetTSDF);
	ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>& sceneManager =
			ITMSceneManipulationEngine_CPU<TVoxel, ITMPlainVoxelArray>::Inst();
};
} // namespace ITMLib
