//  ================================================================
//  Created by Gregory Kramida on 5/25/18.
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

#include "../Interface/ITMHashAllocationEngine.h"
#include "../../../Objects/Scene/ITMVoxelBlockHash.h"
#include "../../../Objects/Tracking/ITMTrackingState.h"
#include "../../../Objects/RenderStates/ITMRenderState.h"
#include "../../../Objects/Views/ITMView.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "../../../Utils/ITMHashBlockProperties.h"
#include "../../Common/ITMWarpEnums.h"

namespace ITMLib {

template<typename TVoxel, typename TWarp>
class ITMHashAllocationEngine_CPU :
		public ITMHashAllocationEngine<TVoxel, TWarp> {
public:
	ITMHashAllocationEngine_CPU() = default;
	~ITMHashAllocationEngine_CPU() = default;

	void AllocateFromDepth(
			ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* scene, const ITMView* view,
			const ITMTrackingState* trackingState, const ITMRenderState* renderState,
			bool onlyUpdateVisibleList, bool resetVisibleList) override;

	void AllocateTSDFVolumeFromTSDFVolume(ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* targetVolume,
	                                      ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* sourceVolume) override;

	void AllocateWarpVolumeFromTSDFVolume(ITMVoxelVolume <TWarp, ITMVoxelBlockHash>* targetVolume,
	                                      ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* sourceVolume) override;

	template<WarpType TWarpType>
	void AllocateFromWarpedVolume(
			ITMVoxelVolume <TWarp, ITMVoxelBlockHash>* warpField,
			ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* sourceTSDF,
			ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* targetTSDF);

private:

	template<typename TVoxelTarget, typename TVoxelSource>
	void AllocateFromVolumeGeneric(ITMVoxelVolume <TVoxelTarget, ITMVoxelBlockHash>* targetVolume,
	                               ITMVoxelVolume <TVoxelSource, ITMVoxelBlockHash>* sourceVolume);

};


}// namespace ITMLib


