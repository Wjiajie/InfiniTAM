// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMMultiMeshingEngine.h"
#include "../../../Objects/Scene/ITMMultiSceneAccess.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMMultiMeshingEngine_CPU : public ITMMultiMeshingEngine<TVoxel, TIndex>
	{
	public:
		explicit ITMMultiMeshingEngine_CPU(const TIndex& index){};
		void MeshScene(ITMMesh *mesh, const ITMVoxelMapGraphManager<TVoxel, TIndex> & sceneManager) {}
	};

	template<class TVoxel>
	class ITMMultiMeshingEngine_CPU<TVoxel, ITMVoxelBlockHash> : public ITMMultiMeshingEngine < TVoxel, ITMVoxelBlockHash >
	{
	public:
		explicit ITMMultiMeshingEngine_CPU(const ITMVoxelBlockHash& index){};
		typedef typename ITMMultiIndex<ITMVoxelBlockHash>::IndexData MultiIndexData;
		typedef ITMMultiVoxel<TVoxel> MultiVoxelData;
		typedef ITMVoxelMapGraphManager<TVoxel, ITMVoxelBlockHash> MultiSceneManager;
		void MeshScene(ITMMesh *mesh, const MultiSceneManager & sceneManager);
	};
}