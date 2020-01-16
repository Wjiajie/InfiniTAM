//  ================================================================
//  Created by Gregory Kramida on 8/28/19.
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
#include "../../../ITMLibDefines.h"
#include "ITMVoxelVolumeComparison_CPU.tpp"

namespace ITMLib {

// region ======================= Instantiations with ITMVoxel =========================================================

template
bool contentAlmostEqual_CPU<ITMVoxel, PlainVoxelArray, PlainVoxelArray, float>(
		ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* a, ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU_Verbose<ITMVoxel, PlainVoxelArray, PlainVoxelArray, float>(
		ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* a, ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU<ITMVoxel, VoxelBlockHash, VoxelBlockHash, float>(
		ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* a, ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU_Verbose<ITMVoxel, VoxelBlockHash, VoxelBlockHash, float>(
		ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* a, ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU<ITMVoxel, PlainVoxelArray, VoxelBlockHash, float>(
		ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* a, ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU_Verbose<ITMVoxel, PlainVoxelArray, VoxelBlockHash, float>(
		ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* a, ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU<ITMVoxel, VoxelBlockHash, PlainVoxelArray, float>(
		ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* a, ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* b,
		float tolerance);

template
bool contentForFlagsAlmostEqual_CPU<ITMVoxel, PlainVoxelArray, VoxelBlockHash, float>(
		ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* a, ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* b,
		VoxelFlags flags, float tolerance);

template
bool contentForFlagsAlmostEqual_CPU_Verbose<ITMVoxel, PlainVoxelArray, VoxelBlockHash, float>(
		ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* a, ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* b,
		VoxelFlags flags, float tolerance);


template
bool allocatedContentAlmostEqual_CPU<ITMVoxel, PlainVoxelArray, PlainVoxelArray, float>(
		ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* a, ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CPU_Verbose<ITMVoxel, PlainVoxelArray, PlainVoxelArray, float>(
		ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* a, ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CPU<ITMVoxel, PlainVoxelArray, VoxelBlockHash, float>(
		ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* a, ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CPU_Verbose<ITMVoxel, PlainVoxelArray, VoxelBlockHash, float>(
		ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* a, ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CPU<ITMVoxel, VoxelBlockHash, PlainVoxelArray, float>(
		ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* a, ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* b,
		float tolerance);

//endregion
// region =================== Instantiations with ITMWarp ==============================================================

template
bool contentAlmostEqual_CPU<ITMWarp, PlainVoxelArray, PlainVoxelArray, float>(
		ITMVoxelVolume<ITMWarp, PlainVoxelArray>* a, ITMVoxelVolume<ITMWarp, PlainVoxelArray>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU_Verbose<ITMWarp, PlainVoxelArray, PlainVoxelArray, float>(
		ITMVoxelVolume<ITMWarp, PlainVoxelArray>* a, ITMVoxelVolume<ITMWarp, PlainVoxelArray>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU<ITMWarp, VoxelBlockHash, VoxelBlockHash, float>(
		ITMVoxelVolume<ITMWarp, VoxelBlockHash>* a, ITMVoxelVolume<ITMWarp, VoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU_Verbose<ITMWarp, VoxelBlockHash, VoxelBlockHash, float>(
		ITMVoxelVolume<ITMWarp, VoxelBlockHash>* a, ITMVoxelVolume<ITMWarp, VoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU<ITMWarp, PlainVoxelArray, VoxelBlockHash, float>(
		ITMVoxelVolume<ITMWarp, PlainVoxelArray>* a, ITMVoxelVolume<ITMWarp, VoxelBlockHash>* b,
		float tolerance);

template
bool contentAlmostEqual_CPU<ITMWarp, VoxelBlockHash, PlainVoxelArray, float>(
		ITMVoxelVolume<ITMWarp, VoxelBlockHash>* a, ITMVoxelVolume<ITMWarp, PlainVoxelArray>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CPU<ITMWarp, PlainVoxelArray, VoxelBlockHash, float>(
		ITMVoxelVolume<ITMWarp, PlainVoxelArray>* a, ITMVoxelVolume<ITMWarp, VoxelBlockHash>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CPU_Verbose<ITMWarp, PlainVoxelArray, VoxelBlockHash, float>(
		ITMVoxelVolume<ITMWarp, PlainVoxelArray>* a, ITMVoxelVolume<ITMWarp, VoxelBlockHash>* b,
		float tolerance);

template
bool allocatedContentAlmostEqual_CPU<ITMWarp, VoxelBlockHash, PlainVoxelArray, float>(
		ITMVoxelVolume<ITMWarp, VoxelBlockHash>* a, ITMVoxelVolume<ITMWarp, PlainVoxelArray>* b,
		float tolerance);

// endregion

} // namespace ITMLib 