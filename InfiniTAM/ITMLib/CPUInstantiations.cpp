// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMLibDefines.h"
//Note: ".tpp" files have to be included for all explicit instanciations in order to link properly
#include "Objects/Scene/ITMSceneManipulation.tpp"
#include "Core/ITMBasicEngine.tpp"
#include "Core/ITMBasicSurfelEngine.tpp"
#include "Core/ITMMultiEngine.tpp"
#include "Core/ITMKillingEngine.tpp"
#include "Core/ITMDenseMapper.tpp"
#include "Core/ITMDenseSurfelMapper.tpp"
#include "Core/ITMDenseDynamicMapper.tpp"
#include "Engines/Meshing/CPU/ITMMeshingEngine_CPU.tpp"
#include "Engines/Meshing/CPU/ITMMultiMeshingEngine_CPU.tpp"
#include "Engines/MultiScene/ITMMapGraphManager.tpp"
#include "Engines/Visualisation/CPU/ITMMultiVisualisationEngine_CPU.tpp"
#include "Engines/Reconstruction/ITMSurfelSceneReconstructionEngineFactory.tpp"
#include "Engines/Reconstruction/CPU/ITMSceneReconstructionEngine_CPU.tpp"
#include "Engines/Reconstruction/CPU/ITMSurfelSceneReconstructionEngine_CPU.tpp"
#include "Engines/Reconstruction/Interface/ITMSurfelSceneReconstructionEngine.tpp"
#include "Engines/Swapping/CPU/ITMSwappingEngine_CPU.tpp"
#include "Engines/Visualisation/ITMSurfelVisualisationEngineFactory.tpp"
#include "Engines/Visualisation/CPU/ITMSurfelVisualisationEngine_CPU.tpp"
#include "Engines/Visualisation/CPU/ITMVisualisationEngine_CPU.tpp"
#include "Engines/Visualisation/Interface/ITMSurfelVisualisationEngine.tpp"
#include "Engines/Visualisation/Interface/ITMVisualisationEngine.h"
#include "Trackers/ITMTrackerFactory.h"
#include "Trackers/Interface/ITMSceneMotionTracker.tpp"
#include "Trackers/CPU/ITMSceneMotionTracker_CPU.tpp"
#include "Utils/ITMSceneSliceRasterizer.tpp"
#include "Utils/ITMSceneStatisticsCalculator.tpp"
#include "Utils/ITMSceneLogger.tpp"

namespace ITMLib
{
	//voxel fusion
	template class ITMBasicEngine<ITMVoxel, ITMVoxelIndex>;
	template class ITMBasicSurfelEngine<ITMSurfel_grey>;
	template class ITMBasicSurfelEngine<ITMSurfel_rgb>;
	template class ITMMultiEngine<ITMVoxel, ITMVoxelIndex>;
	template class ITMDenseMapper<ITMVoxel, ITMVoxelIndex>;
	template class ITMVoxelMapGraphManager<ITMVoxel, ITMVoxelIndex>;
	template class ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>;
	template class ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
	template class ITMMultiMeshingEngine_CPU<ITMVoxel, ITMVoxelIndex>;
	template class ITMSwappingEngine_CPU<ITMVoxel, ITMVoxelIndex>;

	//dynamic-fusion-specific
	template class ITMSwappingEngine_CPU<ITMVoxelCanonical, ITMVoxelIndex>;
	template class ITMKillingEngine<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>;
	template class ITMDenseDynamicMapper<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>;
	template class ITMSceneReconstructionEngine_CPU<ITMVoxelCanonical, ITMVoxelIndex>;
	template class ITMSceneReconstructionEngine_CPU<ITMVoxelLive, ITMVoxelIndex>;
	template class ITMSceneMotionTracker<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>;
	template class ITMSceneMotionTracker_CPU<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>;
	template class ITMVisualisationEngine_CPU<ITMVoxelCanonical, ITMVoxelIndex>;
	template class ITMVisualisationEngine_CPU<ITMVoxelLive, ITMVoxelIndex>;
	template class ITMMeshingEngine_CPU<ITMVoxelCanonical, ITMVoxelIndex>;
	template class ITMSceneLogger<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>;

	//surfel fusion
	template class ITMDenseSurfelMapper<ITMSurfel_grey>;
	template class ITMDenseSurfelMapper<ITMSurfel_rgb>;
	template class ITMSurfelSceneReconstructionEngine<ITMSurfel_grey>;
	template class ITMSurfelSceneReconstructionEngine<ITMSurfel_rgb>;
	template class ITMSurfelSceneReconstructionEngine_CPU<ITMSurfel_grey>;
	template class ITMSurfelSceneReconstructionEngine_CPU<ITMSurfel_rgb>;
	template struct ITMSurfelSceneReconstructionEngineFactory<ITMSurfel_grey>;
	template struct ITMSurfelSceneReconstructionEngineFactory<ITMSurfel_rgb>;
	template class ITMSurfelVisualisationEngine<ITMSurfel_grey>;
	template class ITMSurfelVisualisationEngine<ITMSurfel_rgb>;
	template class ITMSurfelVisualisationEngine_CPU<ITMSurfel_grey>;
	template class ITMSurfelVisualisationEngine_CPU<ITMSurfel_rgb>;
	template struct ITMSurfelVisualisationEngineFactory<ITMSurfel_grey>;
	template struct ITMSurfelVisualisationEngineFactory<ITMSurfel_rgb>;

	//TODO: Cleanup -Greg (GitHub: Algomorph)
	//dynamic fusion utilities
	template class ITMSceneSliceRasterizer<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>;
	template class ITMSceneStatisticsCalculator<ITMVoxelCanonical,ITMVoxelIndex>;
	template class ITMSceneStatisticsCalculator<ITMVoxelLive,ITMVoxelIndex>;

	//scene manipulation functions
	template void CopySceneWithOffset_CPU<ITMVoxelCanonical,ITMVoxelIndex>(
			ITMScene<ITMVoxelCanonical, ITMVoxelIndex>& destination,
			ITMScene<ITMVoxelCanonical, ITMVoxelIndex>& source, Vector3i offset);
	template bool SetVoxel_CPU<ITMVoxelCanonical,ITMVoxelIndex>(ITMScene<ITMVoxelCanonical,
			ITMVoxelIndex>& scene, Vector3i at, ITMVoxelCanonical voxel);
	template bool SetVoxel_CPU<ITMVoxelLive,ITMVoxelIndex>(ITMScene<ITMVoxelLive, ITMVoxelIndex>& scene,
	                                                       Vector3i at, ITMVoxelLive voxel);
	//template ITMVoxel ReadVoxel<ITMVoxelCanonical,ITMVoxelIndex>(ITMScene<ITMVoxelCanonical, ITMVoxelIndex>& scene, Vector3i at);
}
