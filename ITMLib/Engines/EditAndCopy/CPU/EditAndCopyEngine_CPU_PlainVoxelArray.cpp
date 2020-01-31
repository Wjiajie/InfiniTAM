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

#include "../../../ITMLibDefines.h"

//Note: ".tpp" files have to be included for all explicit instantiations in order to link properly
#include "EditAndCopyEngine_CPU_PlainVoxelArray.tpp"
#include "../../../Objects/Scene/VoxelVolume.h"

using namespace ITMLib;

namespace ITMLib {
template
class EditAndCopyEngine_CPU<ITMVoxel, PlainVoxelArray>;
template
class EditAndCopyEngine_CPU<ITMWarp, PlainVoxelArray>;
} // namespace ITMLib