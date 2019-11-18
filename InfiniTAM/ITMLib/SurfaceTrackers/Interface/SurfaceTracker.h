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

#include "SurfaceTrackerInterface.h"
#include "../WarpGradientFunctors/WarpGradientFunctor.h"

namespace ITMLib{



template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
class SurfaceTracker :
		public SurfaceTrackerInterface<TVoxel, TWarp, TIndex>, public SlavchevaSurfaceTracker {
public:
	using SlavchevaSurfaceTracker::SlavchevaSurfaceTracker;
	virtual ~SurfaceTracker() = default;

	void ClearOutFlowWarp(ITMVoxelVolume <TWarp, TIndex>* warpField) override;
	void AddFlowWarpToWarp(
			ITMVoxelVolume <TWarp, TIndex>* warpField, bool clearFlowWarp) override;
	void CalculateWarpGradient(ITMVoxelVolume <TVoxel, TIndex>* canonicalScene,
	                           ITMVoxelVolume <TVoxel, TIndex>* liveScene,
	                           ITMVoxelVolume <TWarp, TIndex>* warpField) override;
	void SmoothWarpGradient(
			ITMVoxelVolume <TVoxel, TIndex>* canonicalScene,
			ITMVoxelVolume <TVoxel, TIndex>* liveScene,
			ITMVoxelVolume <TWarp, TIndex>* warpField) override;

	float UpdateWarps(
			ITMVoxelVolume <TVoxel, TIndex>* canonicalScene,
			ITMVoxelVolume <TVoxel, TIndex>* liveScene,
			ITMVoxelVolume <TWarp, TIndex>* warpField) override;
	void ResetWarps(ITMVoxelVolume <TWarp, TIndex>* warpField) override;
};


} //namespace ITMLib

