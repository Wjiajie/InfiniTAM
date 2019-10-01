//  ================================================================
//  Created by Gregory Kramida on 11/3/17.
//  Copyright (c) 2017-2025 Gregory Kramida
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
#include "TestUtils.h"
#include "TestUtils.tpp"


#include "../ITMLib/Utils/ITMLibSettings.h"
#include "../ITMLib/Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../ITMLib/Utils/FileIO/ITMSceneLogger.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.h"

using namespace ITMLib;

template void GenerateTestScene01<ITMVoxel,ITMVoxelBlockHash>(ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* destination);
template void GenerateTestScene01<ITMVoxel,ITMPlainVoxelArray>(ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* destination);

template void simulateVoxelAlteration<ITMVoxel>(ITMVoxel& voxel, float newSdfValue);
template void simulateRandomVoxelAlteration<ITMVoxel>(ITMVoxel& voxel);
template void simulateRandomVoxelAlteration<ITMWarp>(ITMWarp& voxel);

