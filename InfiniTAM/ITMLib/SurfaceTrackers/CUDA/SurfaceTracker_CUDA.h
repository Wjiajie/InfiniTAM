//  ================================================================
//  Created by Gregory Kramida on 7/24/18.
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
#pragma once

#include "../Interface/SurfaceTrackerInterface.h"
#include "../../Engines/Indexing/VBH/CUDA/ITMIndexingEngine_CUDA.h"

namespace ITMLib {

// region ================================= VOXEL BLOCK HASH ===========================================================

template<typename TVoxel, typename TWarp>
class SurfaceTracker<TVoxel, TWarp, ITMVoxelBlockHash, MEMORYDEVICE_CUDA> :
		public SurfaceTrackerInterface<TVoxel, TWarp, ITMVoxelBlockHash>, public SlavchevaSurfaceTracker {
public:
	explicit SurfaceTracker();
	virtual ~SurfaceTracker() = default;

	void ClearOutFlowWarp(ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField) override;
	void AddFlowWarpToWarp(
			ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField, bool clearFlowWarp) override;
	void CalculateWarpGradient(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* canonicalScene,
	                           ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* liveScene,
	                           ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField) override;
	void SmoothWarpGradient(
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* canonicalScene,
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* liveScene,
			ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField) override;
	float UpdateWarps(
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* canonicalScene,
			ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* liveScene,
			ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField) override;
	void ResetWarps(ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField) override;
};

// endregion ===========================================================================================================
// region ================================= PLAIN VOXEL ARRAY ==========================================================

template<typename TVoxel, typename TWarp>
class SurfaceTracker<TVoxel, TWarp, ITMPlainVoxelArray, MEMORYDEVICE_CUDA> :
		public SurfaceTrackerInterface<TVoxel, TWarp, ITMPlainVoxelArray>, public SlavchevaSurfaceTracker {
public:
	explicit SurfaceTracker();
	virtual ~SurfaceTracker() = default;

	void ClearOutFlowWarp(ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField) override;
	void AddFlowWarpToWarp(
			ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* canonicalScene, bool clearFlowWarp) override;
	void CalculateWarpGradient(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* canonicalScene,
	                           ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* liveScene,
	                           ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField) override;
	void SmoothWarpGradient(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* liveScene,
	                        ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* canonicalScene,
	                        ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField) override;
	float UpdateWarps(
			ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* canonicalScene,
			ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* liveScene,
			ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField) override;

	void ResetWarps(ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField) override;

};
// endregion ===========================================================================================================
}//