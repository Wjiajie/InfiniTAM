// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMMultiVisualisationEngine.h"

namespace ITMLib 
{
	template<class TVoxel, class TIndex>
	class ITMMultiVisualizationEngine_CPU : public ITMMultiVisualisationEngine<TVoxel, TIndex>
	{
	public:
		ITMMultiVisualizationEngine_CPU(void) {}
		~ITMMultiVisualizationEngine_CPU(void) {}

		void PrepareRenderState(const ITMVoxelMapGraphManager<TVoxel, TIndex> & sceneManager, ITMRenderState *state);

		void CreateExpectedDepths(const ITMVoxelMapGraphManager<TVoxel, TIndex> & sceneManager, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;

		void RenderImage(const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState, ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type) const;
	};

template<class TVoxel>
class ITMMultiVisualizationEngine_CPU<TVoxel,ITMVoxelBlockHash> : public ITMMultiVisualisationEngine<TVoxel, ITMVoxelBlockHash>
{
public:
	ITMMultiVisualizationEngine_CPU(void) {}
	~ITMMultiVisualizationEngine_CPU(void) {}

	void PrepareRenderState(const ITMVoxelMapGraphManager<TVoxel, ITMVoxelBlockHash> & sceneManager, ITMRenderState *state);

	void CreateExpectedDepths(const ITMVoxelMapGraphManager<TVoxel, ITMVoxelBlockHash> & sceneManager, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;

	void RenderImage(const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState, ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type) const;
};
}

