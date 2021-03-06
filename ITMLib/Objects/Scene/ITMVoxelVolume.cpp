//  ================================================================
//  Created by Gregory Kramida on 5/8/19.
//  Copyright (c) 2019 Gregory Kramida
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

#include "ITMVoxelVolume.tpp"
#include "../../ITMLibDefines.h"

// Explicit template instantiations for ITMVoxelVolume
namespace ITMLib {
	template class ITMVoxelVolume<ITMVoxel, PlainVoxelArray>;
	template class ITMVoxelVolume<ITMVoxel, VoxelBlockHash>;
	template class ITMVoxelVolume<ITMWarp, PlainVoxelArray>;
	template class ITMVoxelVolume<ITMWarp, VoxelBlockHash>;
}  // namespace ITMLib