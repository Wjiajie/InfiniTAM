//  ================================================================
//  Created by Gregory Kramida on 11/18/19.
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


//local
#include "SurfaceTracker.h"

#include "../Shared/SurfaceTrackerSharedFunctors.h"
#include "../WarpGradientFunctors/WarpGradientFunctor_SlavchevaOptimized.h"
#include "../WarpGradientFunctors/WarpGradientFunctor_SlavchevaDiagnostic.h"
#include "../../Engines/Indexing/Interface/ITMIndexingEngine.h"

#include "../../Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.h"

#include "../../Engines/Traversal/CPU/ITMSceneTraversal_CPU_VoxelBlockHash.h"
#include "../../Engines/Traversal/CPU/ITMSceneTraversal_CPU_PlainVoxelArray.h"
#include "../../Engines/Indexing/VBH/CPU/ITMIndexingEngine_CPU_VoxelBlockHash.h"
#include "../../Engines/VolumeEditAndCopy/CPU/VolumeEditAndCopyEngine_CPU.h"

#ifdef __CUDACC__
#include "../../Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA.h"

#include "../../Engines/Traversal/CUDA/ITMSceneTraversal_CUDA_VoxelBlockHash.h"
#include "../../Engines/Traversal/CUDA/ITMSceneTraversal_CUDA_PlainVoxelArray.h"
#include "../../Engines/Indexing/VBH/CUDA/ITMIndexingEngine_CUDA_VoxelBlockHash.h"
#include "../../Engines/VolumeEditAndCopy/CUDA/VolumeEditAndCopyEngine_CUDA.h"
#endif


using namespace ITMLib;

// region ===================================== HOUSEKEEPING ===========================================================


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
void SurfaceTracker<TVoxel, TWarp, TIndex, TMemoryDeviceType, TGradientFunctorType>::ResetWarps(
		ITMVoxelVolume<TWarp, TIndex>* warpField) {
	ITMSceneTraversalEngine<TWarp, TIndex, TMemoryDeviceType>::template
	StaticVoxelTraversal<WarpClearFunctor<TWarp, TWarp::hasCumulativeWarp>>(warpField);
};


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
void SurfaceTracker<TVoxel, TWarp, TIndex, TMemoryDeviceType, TGradientFunctorType>::ClearOutFramewiseWarp(
		ITMVoxelVolume<TWarp, TIndex>* warpField) {
	ITMSceneTraversalEngine<TWarp, TIndex, TMemoryDeviceType>::template
	StaticVoxelTraversal<ClearOutFramewiseWarpStaticFunctor<TWarp>>(warpField);
}


// endregion ===========================================================================================================

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
inline static void PrintSceneStatistics(
		ITMVoxelVolume<TVoxel, TIndex>* scene,
		std::string description) {
	ITMSceneStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>& calculator =
			ITMSceneStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::Instance();
	std::cout << green << "=== Stats for scene '" << description << "' ===" << reset << std::endl;
	std::cout << "    Total voxel count: " << calculator.ComputeAllocatedVoxelCount(scene) << std::endl;
	std::cout << "    NonTruncated voxel count: " << calculator.ComputeNonTruncatedVoxelCount(scene) << std::endl;
	std::cout << "    +1.0 voxel count: " << calculator.CountVoxelsWithSpecificSdfValue(scene, 1.0f) << std::endl;
	std::vector<int> allocatedHashes = calculator.GetFilledHashBlockIds(scene);
	std::cout << "    Allocated hash count: " << allocatedHashes.size() << std::endl;
	std::cout << "    NonTruncated SDF sum: " << calculator.ComputeNonTruncatedVoxelAbsSdfSum(scene) << std::endl;
	std::cout << "    Truncated SDF sum: " << calculator.ComputeTruncatedVoxelAbsSdfSum(scene) << std::endl;
};

// region ===================================== CALCULATE WARPS ========================================================


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
void
SurfaceTracker<TVoxel, TWarp, TIndex, TMemoryDeviceType, TGradientFunctorType>::CalculateWarpGradient(
		ITMVoxelVolume<TVoxel, TIndex>* canonicalScene,
		ITMVoxelVolume<TVoxel, TIndex>* liveScene,
		ITMVoxelVolume<TWarp, TIndex>* warpField) {

	// manage hash
	ITMSceneTraversalEngine<TWarp, TIndex, TMemoryDeviceType>::template
	StaticVoxelTraversal<ClearOutGradientStaticFunctor<TWarp>>(warpField);

	ITMIndexingEngine<TVoxel, TIndex, TMemoryDeviceType>::Instance()
			.AllocateUsingOtherVolume(canonicalScene, liveScene);

	ITMIndexingEngine<TVoxel, TIndex, TMemoryDeviceType>::Instance()
			.AllocateUsingOtherVolume(warpField, liveScene);

	WarpGradientFunctor<TVoxel, TWarp, TIndex, TMemoryDeviceType, TGradientFunctorType>
			calculateGradientFunctor(this->parameters, this->switches,
			                         liveScene, canonicalScene, warpField,
			                         canonicalScene->sceneParams->voxel_size, canonicalScene->sceneParams->narrow_band_half_width);

	ITMDualSceneWarpTraversalEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType>::
	DualVoxelPositionTraversal(liveScene, canonicalScene, warpField, calculateGradientFunctor);
	calculateGradientFunctor.PrintStatistics();
}

// endregion ===========================================================================================================
// region ========================================== SOBOLEV GRADIENT SMOOTHING ========================================


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
void SurfaceTracker<TVoxel, TWarp, TIndex, TMemoryDeviceType, TGradientFunctorType>::SmoothWarpGradient(
		ITMVoxelVolume<TVoxel, TIndex>* canonicalScene,
		ITMVoxelVolume<TVoxel, TIndex>* liveScene,
		ITMVoxelVolume<TWarp, TIndex>* warpField) {

	if (this->switches.enable_sobolev_gradient_smoothing) {
		SmoothWarpGradient_common<TVoxel, TWarp, TIndex, TMemoryDeviceType>
				(liveScene, canonicalScene, warpField);
	}
}

// endregion ===========================================================================================================

// region ============================= UPDATE FRAMEWISE & GLOBAL (CUMULATIVE) WARPS ===================================
template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
float SurfaceTracker<TVoxel, TWarp, TIndex, TMemoryDeviceType, TGradientFunctorType>::UpdateWarps(
		ITMVoxelVolume<TVoxel, TIndex>* canonicalScene,
		ITMVoxelVolume<TVoxel, TIndex>* liveScene,
		ITMVoxelVolume<TWarp, TIndex>* warpField) {
	return UpdateWarps_common<TVoxel, TWarp, TIndex, TMemoryDeviceType>(
			canonicalScene, liveScene, warpField, this->parameters.learning_rate,
			this->switches.enable_sobolev_gradient_smoothing,
			this->histograms_enabled);
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
void SurfaceTracker<TVoxel, TWarp, TIndex, TMemoryDeviceType, TGradientFunctorType>::AddFramewiseWarpToWarp(
		ITMVoxelVolume<TWarp, TIndex>* warpField, bool clearFramewiseWarp) {
	AddFramewiseWarpToWarp_common<TWarp, TIndex, TMemoryDeviceType>(warpField, clearFramewiseWarp);
}

//endregion ============================================================================================================