//  ================================================================
//  Created by Gregory Kramida on 10/18/17.
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
#pragma once

#include "../Interface/SurfaceTrackerInterface.h"
#include "../../Utils/ITMHashBlockProperties.h"
#include "../../Engines/Indexing/VBH/CPU/ITMIndexingEngine_CPU_VoxelBlockHash.h"


namespace ITMLib {

//region ======================================== VOXEL BLOCK HASH =====================================================

template<typename TVoxel, typename TWarp>
class SurfaceTracker<TVoxel, TWarp, ITMVoxelBlockHash, MEMORYDEVICE_CPU> :
		public SurfaceTrackerInterface<TVoxel, TWarp, ITMVoxelBlockHash>, public SlavchevaSurfaceTracker {
public:
	using SlavchevaSurfaceTracker::SlavchevaSurfaceTracker;
	virtual ~SurfaceTracker() = default;

	void ClearOutFlowWarp(ITMVoxelVolume <TWarp, ITMVoxelBlockHash>* warpField) override;
	void AddFlowWarpToWarp(
			ITMVoxelVolume <TWarp, ITMVoxelBlockHash>* warpField, bool clearFlowWarp) override;
	void CalculateWarpGradient(ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* canonicalScene,
	                           ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* liveScene,
	                           ITMVoxelVolume <TWarp, ITMVoxelBlockHash>* warpField) override;
	void SmoothWarpGradient(
			ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* canonicalScene,
			ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* liveScene,
			ITMVoxelVolume <TWarp, ITMVoxelBlockHash>* warpField) override;

	float UpdateWarps(
			ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* canonicalScene,
			ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* liveScene,
			ITMVoxelVolume <TWarp, ITMVoxelBlockHash>* warpField) override;
	void ResetWarps(ITMVoxelVolume <TWarp, ITMVoxelBlockHash>* warpField) override;
};

// endregion ===========================================================================================================
//region ======================================== PLAIN VOXEL ARRAY ====================================================

template<typename TVoxel, typename TWarp>
class SurfaceTracker<TVoxel, TWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU> :
		public SurfaceTrackerInterface<TVoxel, TWarp, ITMPlainVoxelArray>, public SlavchevaSurfaceTracker  {
public:
	using SlavchevaSurfaceTracker::SlavchevaSurfaceTracker;
	virtual ~SurfaceTracker() = default;

	void ClearOutFlowWarp(ITMVoxelVolume <TWarp, ITMPlainVoxelArray>* warpField) override;
	void AddFlowWarpToWarp(
			ITMVoxelVolume <TWarp, ITMPlainVoxelArray>* warpField, bool clearFlowWarp) override;
	void CalculateWarpGradient(ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* canonicalScene,
	                           ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* liveScene,
	                           ITMVoxelVolume <TWarp, ITMPlainVoxelArray>* warpField) override;
	void SmoothWarpGradient(ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* canonicalScene,
	                        ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* liveScene,
	                        ITMVoxelVolume <TWarp, ITMPlainVoxelArray>* warpField) override;
	float UpdateWarps(
			ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* canonicalScene,
			ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* liveScene,
			ITMVoxelVolume <TWarp, ITMPlainVoxelArray>* warpField) override;

	void ResetWarps(ITMVoxelVolume <TWarp, ITMPlainVoxelArray>* warpField) override;


};

// endregion ===========================================================================================================

}//namespace ITMLib


