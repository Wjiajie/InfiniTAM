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
#pragma once

#include "ITMVoxelVolumeComparison_CPU.h"
#include "../Objects/Scene/ITMPlainVoxelArray.h"
#include "../Objects/Scene/ITMVoxelBlockHash.h"
#include "Analytics/ITMAlmostEqual.h"
#include "../../ORUtils/MemoryDeviceType.h"
#include "../Engines/Manipulation/CPU/ITMSceneTraversal_CPU_PlainVoxelArray.h"
#include "../Engines/Manipulation/CPU/ITMSceneTraversal_CPU_VoxelBlockHash.h"
#include "../Engines/Manipulation/CPU/ITMSceneTraversal_CPU_PVA_to_VBH.h"


namespace ITMLib {
//region ================================= VOXEL VOLUME CONTENT COMPARISON FUNCTIONS ==================================
template<typename TVoxel, typename ToleranceType>
struct VoxelEqualFunctor {
	explicit VoxelEqualFunctor(ToleranceType tolerance) : tolerance(tolerance) {}

	bool operator()(TVoxel& a, TVoxel& b) {
		return almostEqual(a, b, tolerance);
	}

	ToleranceType tolerance;
};
//
//template<typename TVoxel, typename TIndexA, typename TIndexB, typename ToleranceType>
//struct ContentAlmostEqualFunctor_CPU;
//
//template<typename TVoxel, typename ToleranceType>
//struct ContentAlmostEqualFunctor_CPU<TVoxel, ITMPlainVoxelArray, ITMVoxelBlockHash, ToleranceType> {
//	static inline
//	bool evaluate(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* a, ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* b,
//	              ToleranceType tolerance) {
//
//	}
//};
//
//template<typename TVoxel, typename ToleranceType>
//struct ContentAlmostEqualFunctor_CPU<TVoxel, ITMVoxelBlockHash, ITMPlainVoxelArray, ToleranceType> {
//	static inline
//	bool evaluate(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* a, ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* b,
//	              ToleranceType tolerance) {
//		// a simple argument reordering
//		return ContentAlmostEqualFunctor_CPU<TVoxel, ITMPlainVoxelArray, ITMVoxelBlockHash, ToleranceType>
//		::evaluate(b, a, tolerance);
//	}
//};
//


// matching index type case
//template<typename TVoxel, typename ToleranceType, typename TIndex>
//struct ContentAlmostEqualFunctor_CPU<TVoxel, TIndex, TIndex, ToleranceType> {
//	static inline
//	bool evaluate(ITMVoxelVolume<TVoxel, TIndex>* a, ITMVoxelVolume<TVoxel, TIndex>* b,
//	              ToleranceType tolerance) {
//		VoxelEqualFunctor<TVoxel, ToleranceType> functor(tolerance);
//		return ITMDualSceneTraversalEngine<TVoxel, TVoxel, TIndex, TIndex, ITMLibSettings::DEVICE_CPU>
//		::template DualVoxelTraversal_AllTrue(a, b, functor);
//	}
//};



template<typename TVoxel, typename TIndexA, typename TIndexB, typename ToleranceType>
bool contentAlmostEqual_CPU(ITMVoxelVolume<TVoxel, TIndexA>* a, ITMVoxelVolume<TVoxel, TIndexB>* b,
                            ToleranceType tolerance) {
	VoxelEqualFunctor<TVoxel, ToleranceType> functor(tolerance);
	return ITMDualSceneTraversalEngine<TVoxel, TVoxel, TIndexA, TIndexB, ITMLibSettings::DEVICE_CPU>
	::template DualVoxelTraversal_AllTrue(a, b, functor);
}

//endregion
}