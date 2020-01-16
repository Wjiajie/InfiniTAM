// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMMeshingEngine.h"
#include "../../../Objects/Scene/PlainVoxelArray.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMMeshingEngine_CUDA : public ITMMeshingEngine < TVoxel, TIndex >{};

	template<class TVoxel>
	class ITMMeshingEngine_CUDA<TVoxel, VoxelBlockHash> : public ITMMeshingEngine < TVoxel, VoxelBlockHash >
	{
	private:
		unsigned int  *noTriangles_device;
		Vector4s *visibleBlockGlobalPos_device;

	public:
		void MeshScene(ITMMesh *mesh, const ITMVoxelVolume<TVoxel, VoxelBlockHash> *scene) override;

		explicit ITMMeshingEngine_CUDA(const VoxelBlockHash& index);
		~ITMMeshingEngine_CUDA();
	};

	template<class TVoxel>
	class ITMMeshingEngine_CUDA<TVoxel, PlainVoxelArray> : public ITMMeshingEngine < TVoxel, PlainVoxelArray >
	{
	public:
		void MeshScene(ITMMesh *mesh, const ITMVoxelVolume<TVoxel, PlainVoxelArray> *scene) override;

		explicit ITMMeshingEngine_CUDA(const PlainVoxelArray& index);
		~ITMMeshingEngine_CUDA();
	};
}
