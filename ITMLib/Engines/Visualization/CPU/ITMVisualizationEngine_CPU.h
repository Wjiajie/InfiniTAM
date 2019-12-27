// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMVisualisationEngine.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMVisualizationEngine_CPU : public ITMVisualisationEngine < TVoxel, TIndex >
	{
	public:
		explicit ITMVisualizationEngine_CPU(void) { }
		~ITMVisualizationEngine_CPU(void) { }

		void FindVisibleBlocks(ITMVoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
		int CountVisibleBlocks(const ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMRenderState *renderState, int minBlockId, int maxBlockId) const;
		void CreateExpectedDepths(const ITMVoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
		void RenderImage(ITMVoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState,
		                 ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE,
		                 IITMVisualisationEngine::RenderRaycastSelection raycastType = IITMVisualisationEngine::RENDER_FROM_NEW_RAYCAST) const;
		void FindSurface(ITMVoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const;
		void CreatePointCloud(ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) const;
		void CreateICPMaps(ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
		void ForwardRender(const ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
	};

	template<class TVoxel>
	class ITMVisualizationEngine_CPU<TVoxel, ITMVoxelBlockHash> : public ITMVisualisationEngine < TVoxel, ITMVoxelBlockHash >
	{
	public:
		explicit ITMVisualizationEngine_CPU(void) { }
		~ITMVisualizationEngine_CPU(void) { }

		void FindVisibleBlocks(ITMVoxelVolume<TVoxel,ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
		int CountVisibleBlocks(const ITMVoxelVolume<TVoxel,ITMVoxelBlockHash> *scene, const ITMRenderState *renderState, int minBlockId, int maxBlockId) const;
		void CreateExpectedDepths(const ITMVoxelVolume<TVoxel,ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
		void RenderImage(ITMVoxelVolume<TVoxel,ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState,
		                 ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE,
		                 IITMVisualisationEngine::RenderRaycastSelection raycastType = IITMVisualisationEngine::RENDER_FROM_NEW_RAYCAST) const;
		void FindSurface(ITMVoxelVolume<TVoxel,ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const;
		void CreatePointCloud(ITMVoxelVolume<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) const;
		void CreateICPMaps(ITMVoxelVolume<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
		void ForwardRender(const ITMVoxelVolume<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
	};
}
