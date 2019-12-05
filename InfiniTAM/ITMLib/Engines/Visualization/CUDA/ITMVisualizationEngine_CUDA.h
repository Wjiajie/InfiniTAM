// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMVisualisationEngine.h"

struct RenderingBlock;

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMVisualizationEngine_CUDA : public ITMVisualisationEngine < TVoxel, TIndex >
	{
	private:
		uint *noTotalPoints_device;

	public:
		explicit ITMVisualizationEngine_CUDA(void);
		~ITMVisualizationEngine_CUDA(void);

		ITMRenderState* CreateRenderState(const ITMVoxelVolume<TVoxel, TIndex> *scene, const Vector2i & imgSize) const;
		void FindVisibleBlocks(const ITMVoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
		int CountVisibleBlocks(const ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMRenderState *renderState, int minBlockId, int maxBlockId) const;
		void CreateExpectedDepths(const ITMVoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
		void RenderImage(const ITMVoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState,
		                 ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE,
		                 IITMVisualisationEngine::RenderRaycastSelection raycastType = IITMVisualisationEngine::RENDER_FROM_NEW_RAYCAST) const;
		void FindSurface(const ITMVoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const;
		void CreatePointCloud(const ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) const;
		void CreateICPMaps(const ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
		void ForwardRender(const ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
	};

	template<class TVoxel>
	class ITMVisualizationEngine_CUDA<TVoxel, ITMVoxelBlockHash> : public ITMVisualisationEngine < TVoxel, ITMVoxelBlockHash >
	{
	private:
		uint *noTotalPoints_device;
		RenderingBlock *renderingBlockList_device;
		uint *noTotalBlocks_device;
		int *noVisibleEntries_device;
	public:
		explicit ITMVisualizationEngine_CUDA(void);
		~ITMVisualizationEngine_CUDA(void);

		ITMRenderState_VH* CreateRenderState(const ITMVoxelVolume<TVoxel, ITMVoxelBlockHash> *scene, const Vector2i & imgSize) const;
		void FindVisibleBlocks(const ITMVoxelVolume<TVoxel,ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
		int CountVisibleBlocks(const ITMVoxelVolume<TVoxel,ITMVoxelBlockHash> *scene, const ITMRenderState *renderState, int minBlockId, int maxBlockId) const;
		void CreateExpectedDepths(const ITMVoxelVolume<TVoxel,ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
		void RenderImage(const ITMVoxelVolume<TVoxel,ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState,
		                 ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE,
		                 IITMVisualisationEngine::RenderRaycastSelection raycastType = IITMVisualisationEngine::RENDER_FROM_NEW_RAYCAST) const;
		void FindSurface(const ITMVoxelVolume<TVoxel,ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const;
		void CreatePointCloud(const ITMVoxelVolume<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) const;
		void CreateICPMaps(const ITMVoxelVolume<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
		void ForwardRender(const ITMVoxelVolume<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
	};
}
