// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <math.h>

#include "../../../Objects/Meshing/ITMMesh.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"

namespace ITMLib
{
/**
 * \brief Responsible for generating a triangular mesh from the voxel grid / scene containing implicit
 * signed distance function values
 * \tparam TVoxel type of voxels used in the voxel grid / scene
 * \tparam TIndex type of indexing structure used to access the voxels in the voxel grid / scene
 */
	template<class TVoxel, class TIndex>
	class ITMMeshingEngine
	{
	public:
		/**
		 * \brief Runs MarchingCubes on the voxel grid to generate the triangle mesh
		 * \param mesh[out] mesh that is generated
		 * \param scene[in] voxel grid with SDF values
		 */
		virtual void MeshScene(ITMMesh *mesh, const ITMVoxelVolume<TVoxel,TIndex> *scene) = 0;

		explicit ITMMeshingEngine(const TIndex& index){}
		virtual ~ITMMeshingEngine() = default;
	};
}
