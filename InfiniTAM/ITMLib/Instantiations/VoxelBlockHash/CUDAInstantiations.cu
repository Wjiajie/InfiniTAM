// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM


#include "../../ITMLibDefines.h"

#include "../../Engines/Meshing/CUDA/ITMMeshingEngine_CUDA.tcu"
#include "../../Engines/Meshing/CUDA/ITMMultiMeshingEngine_CUDA.tcu"
#include "../../Engines/Reconstruction/CUDA/ITMSceneReconstructionEngine_CUDA.tcu"
#include "../../Engines/Swapping/CUDA/ITMSwappingEngine_CUDA.tcu"

#include "../../Engines/Visualization/CUDA/ITMVisualizationEngine_CUDA.tcu"
#include "../../Engines/Visualization/CUDA/ITMMultiVisualizationEngine_CUDA.tcu"

namespace ITMLib
{
	template class ITMMeshingEngine_CUDA<ITMVoxel, ITMVoxelBlockHash>;
	template class ITMMultiMeshingEngine_CUDA<ITMVoxel, ITMVoxelBlockHash>;
	template class ITMSceneReconstructionEngine_CUDA<ITMVoxel, ITMVoxelBlockHash>;
	template class ITMSwappingEngine_CUDA<ITMVoxel, ITMVoxelBlockHash>;
	template class ITMVisualizationEngine_CUDA<ITMVoxel, ITMVoxelBlockHash>;
	template class ITMMultiVisualizationEngine_CUDA<ITMVoxel, ITMVoxelBlockHash>;
}
