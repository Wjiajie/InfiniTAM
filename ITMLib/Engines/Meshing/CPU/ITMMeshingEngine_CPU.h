// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMMeshingEngine.h"
#include "../../../Objects/Scene/PlainVoxelArray.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMMeshingEngine_CPU : public ITMMeshingEngine < TVoxel, TIndex >
	{
	public:
		explicit ITMMeshingEngine_CPU(const TIndex& index) : ITMMeshingEngine<TVoxel,TIndex>(index){};
		//TODO: implement meshing for PVA (for completeness / consistency)
		void MeshScene(ITMMesh *mesh, const ITMVoxelVolume<TVoxel, TIndex> *scene) {
			DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
		}
	};

	template<class TVoxel>
	class ITMMeshingEngine_CPU<TVoxel, VoxelBlockHash> : public ITMMeshingEngine < TVoxel, VoxelBlockHash >
	{
	public:
		void MeshScene(ITMMesh *mesh, const ITMVoxelVolume<TVoxel, VoxelBlockHash> *scene);

		explicit ITMMeshingEngine_CPU(const VoxelBlockHash& index) :
			ITMMeshingEngine<TVoxel,VoxelBlockHash>(index) { }
		~ITMMeshingEngine_CPU() = default;
	};
}
