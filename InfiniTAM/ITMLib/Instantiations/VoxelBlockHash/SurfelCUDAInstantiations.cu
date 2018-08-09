#include "../../ITMLibDefines.h"

#include "../../Engines/Visualisation/CUDA/ITMSurfelVisualisationEngine_CUDA.tcu"
#include "../../Engines/Reconstruction/CUDA/ITMSurfelSceneReconstructionEngine_CUDA.tcu"

namespace ITMLib
{
	template class ITMSurfelSceneReconstructionEngine_CUDA<ITMSurfel_grey>;
	template class ITMSurfelSceneReconstructionEngine_CUDA<ITMSurfel_rgb>;
	template class ITMSurfelVisualisationEngine_CUDA<ITMSurfel_grey>;
	template class ITMSurfelVisualisationEngine_CUDA<ITMSurfel_rgb>;

}