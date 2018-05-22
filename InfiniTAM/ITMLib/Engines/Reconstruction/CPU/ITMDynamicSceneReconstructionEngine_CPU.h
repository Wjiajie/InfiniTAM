// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMDynamicSceneReconstructionEngine.h"
#include "../../../Objects/Scene/ITMPlainVoxelArray.h"

namespace ITMLib
{
	template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
	class ITMDynamicSceneReconstructionEngine_CPU : public ITMDynamicSceneReconstructionEngine < TVoxelCanonical, TVoxelLive, TIndex >
	{};

	template<typename TVoxelCanonical, typename TVoxelLive>
	class ITMDynamicSceneReconstructionEngine_CPU<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash> : public ITMDynamicSceneReconstructionEngine < TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash >
	{
	protected:
		ORUtils::MemoryBlock<unsigned char> *entriesAllocType;
		ORUtils::MemoryBlock<Vector4s> *blockCoords;
		template<typename TVoxel>
		void ResetScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene);

	public:
		void ResetCanonicalScene(ITMScene<TVoxelCanonical, ITMVoxelBlockHash> *scene) override;
		void ResetLiveScene(ITMScene<TVoxelLive, ITMVoxelBlockHash> *scene) override;

		void AllocateSceneFromDepth(ITMScene<TVoxelLive, ITMVoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
			const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false) override;

		void IntegrateIntoScene(ITMScene<TVoxelLive, ITMVoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
			const ITMRenderState *renderState);

		ITMDynamicSceneReconstructionEngine_CPU(void);
		~ITMDynamicSceneReconstructionEngine_CPU(void);
	};

	template<typename TVoxelCanonical, typename TVoxelLive>
	class ITMDynamicSceneReconstructionEngine_CPU<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray> : public ITMDynamicSceneReconstructionEngine < TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray >
	{
	protected:
		template <typename TVoxel>
		void ResetScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene);
	public:
		void ResetCanonicalScene(ITMScene<TVoxelCanonical, ITMPlainVoxelArray> *scene) override;
		void ResetLiveScene(ITMScene<TVoxelLive, ITMPlainVoxelArray> *scene) override;

		void AllocateSceneFromDepth(ITMScene<TVoxelLive, ITMPlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
			const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false);

		void IntegrateIntoScene(ITMScene<TVoxelLive, ITMPlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
			const ITMRenderState *renderState);

		ITMDynamicSceneReconstructionEngine_CPU(void);
		~ITMDynamicSceneReconstructionEngine_CPU(void);
	};
}
