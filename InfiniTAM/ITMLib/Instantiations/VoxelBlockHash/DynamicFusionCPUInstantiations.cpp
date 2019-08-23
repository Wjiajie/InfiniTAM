//  ================================================================
//  Created by Gregory Kramida on 1/31/18.
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
#include "../../ITMLibDefines.h"
//Note: ".tpp" files have to be included for all explicit instantiations in order to link properly

#include "../../Core/ITMDynamicEngine.tpp"

#include "../../Objects/Scene/ITMVoxelVolume.h"


#include "../../Engines/Swapping/CPU/ITMSwappingEngine_CPU.tpp"
#include "../../Engines/Visualisation/CPU/ITMVisualisationEngine_CPU.tpp"
#include "../../Engines/Meshing/CPU/ITMMeshingEngine_CPU.tpp"
#include "../../Engines/SceneFileIO/ITMSceneFileIOEngine.tpp"
#include "../../Utils/Analytics/ITMSceneStatisticsCalculator.tpp"
#include "../../Utils/Analytics/ITMNeighborVoxelIterationInfo.h"
#include "../../Utils/Visualization/ITMSceneSliceVisualizer2D.tpp"
#include "../../Utils/Visualization/ITMSceneSliceVisualizer1D.tpp"
#include "../../Utils/FileIO/ITMWarpFieldLogger.tpp"
#include "../../Utils/FileIO/ITMSceneLogger.tpp"
#include "../../Utils/Collections/ITM3DNestedMapOfArrays.tpp"
#include "../../Utils/Collections/ITM3DNestedMap.tpp"



template class ITMSwappingEngine_CPU<ITMVoxel, ITMVoxelBlockHash>;
template class ITMDynamicEngine<ITMVoxel, ITMWarp, ITMVoxelBlockHash>;
template class ITMSceneFileIOEngine<ITMVoxel,ITMVoxelBlockHash>;
template class ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelBlockHash>;
template class ITMMeshingEngine_CPU<ITMVoxel, ITMVoxelBlockHash>;
template class ITMWarpFieldLogger<ITMWarp, ITMVoxelBlockHash>;
template class ITMSceneLogger<ITMVoxel, ITMWarp, ITMVoxelBlockHash>;

//TODO: Cleanup -Greg (GitHub: Algomorph)
//dynamic fusion utility classes
template class ITMSceneSliceVisualizer2D<ITMVoxel, ITMWarp, ITMVoxelBlockHash>;

template class ITMSceneStatisticsCalculator<ITMVoxel, ITMVoxelBlockHash>;;
template class ITM3DNestedMap<int>;
template class ITM3DNestedMap<std::tuple<int,int>>;
template class ITM3DNestedMapOfArrays<int>;
template class ITM3DNestedMapOfArrays<ITMHighlightIterationInfo>;

