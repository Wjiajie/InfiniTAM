// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/MultiVisualizationEngine.h"

struct RenderingBlock;

namespace ITMLib {

	template<class TVoxel, class TIndex>
	class MultiVisualizationEngine_CUDA : public MultiVisualizationEngine<TVoxel, TIndex>
	{
	private:

	public:
		MultiVisualizationEngine_CUDA(void);
		~MultiVisualizationEngine_CUDA(void);

		void PrepareRenderState(const ITMVoxelMapGraphManager<TVoxel, TIndex> & sceneManager, ITMRenderState *state);

		void CreateExpectedDepths(const ITMVoxelMapGraphManager<TVoxel, TIndex> & sceneManager, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;

		void RenderImage(const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState, ITMUChar4Image *outputImage, IVisualizationEngine::RenderImageType type) const;
	};

	template<class TVoxel>
	class MultiVisualizationEngine_CUDA<TVoxel, VoxelBlockHash> : public MultiVisualizationEngine<TVoxel, VoxelBlockHash>
	{
	private:
		RenderingBlock *renderingBlockList_device;
		uint *noTotalBlocks_device;

	public:
		MultiVisualizationEngine_CUDA(void);
		~MultiVisualizationEngine_CUDA(void);

		void PrepareRenderState(const ITMVoxelMapGraphManager<TVoxel, VoxelBlockHash> & sceneManager, ITMRenderState *state);

		void CreateExpectedDepths(const ITMVoxelMapGraphManager<TVoxel, VoxelBlockHash> & sceneManager, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;

		void RenderImage(const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState, ITMUChar4Image *outputImage, IVisualizationEngine::RenderImageType type) const;
	};
}