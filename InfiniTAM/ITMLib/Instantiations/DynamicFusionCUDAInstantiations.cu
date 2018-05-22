//  ================================================================
//  Created by Gregory Kramida on 5/22/18.
//  Copyright (c) 2018-2025 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================
#include "../ITMLibDefines.h"

#include "../Engines/Reconstruction/CUDA/ITMSceneReconstructionEngine_CUDA.tcu"
#include "../Engines/Swapping/CUDA/ITMSwappingEngine_CUDA.tcu"
#include "../Engines/Visualisation/CUDA/ITMVisualisationEngine_CUDA.tcu"
#include "../Engines/Meshing/CUDA/ITMMeshingEngine_CUDA.tcu"

namespace ITMLib {

//KillingFusion stuff
template
class ITMSceneReconstructionEngine_CUDA<ITMVoxelCanonical, ITMVoxelIndex>;
template
class ITMSceneReconstructionEngine_CUDA<ITMVoxelLive, ITMVoxelIndex>;
template
class ITMSwappingEngine_CUDA<ITMVoxelCanonical, ITMVoxelIndex>;

template
class ITMVisualisationEngine_CUDA<ITMVoxelLive, ITMVoxelIndex>;
template
class ITMVisualisationEngine_CUDA<ITMVoxelCanonical, ITMVoxelIndex>;
template
class ITMMeshingEngine_CUDA<ITMVoxelCanonical, ITMVoxelIndex>;

}