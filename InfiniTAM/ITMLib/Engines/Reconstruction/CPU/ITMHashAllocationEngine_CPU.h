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

#include "../../../Objects/Scene/ITMVoxelBlockHash.h"
#include "../../../Objects/Tracking/ITMTrackingState.h"
#include "../../../Objects/RenderStates/ITMRenderState.h"
#include "../../../Objects/Views/ITMView.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "../../../Utils/ITMHashBlockProperties.h"
#include "../../Common/ITMWarpEnums.h"

namespace ITMLib {

template<typename TVoxel, typename TWarp>
class ITMHashAllocationEngine_CPU {
public:
	ITMHashAllocationEngine_CPU();
	~ITMHashAllocationEngine_CPU();
	/**
	 * \brief Given a view with a new depth image, compute the
		visible blocks, allocate them and update the hash
		table so that the new image data can be integrated.
	 * \param scene [out] the scene whose hash needs additional allocations
	 * \param view [in] a view with a new depth image
	 * \param trackingState [in] tracking state from previous frame to new frame that corresponds to the given view
	 * \param renderState [in] the current renderState with information about which hash entries are visible
	 * \param onlyUpdateVisibleList [in] whether we want to allocate only the hash entry blocks currently visible
	 * \param resetVisibleList  [in] reset visibility list upon completion
	 */
	void AllocateFromDepth(
			ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* scene, const ITMView* view,
			const ITMTrackingState* trackingState, const ITMRenderState* renderState,
			bool onlyUpdateVisibleList = false, bool resetVisibleList = false);

	void AllocateTSDFVolumeFromTSDFVolume(ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* targetVolume,
	                                      ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* sourceVolume);

	void AllocateWarpVolumeFromTSDFVolume(ITMVoxelVolume <TWarp, ITMVoxelBlockHash>* targetVolume,
	                                      ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* sourceVolume);

	template<WarpType TWarpType>
	void AllocateFromWarpedVolume(
			ITMVoxelVolume <TWarp, ITMVoxelBlockHash>* warpField,
			ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* sourceTSDF,
			ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* targetTSDF);

private:

	template<typename TVoxelTarget, typename TVoxelSource>
	void AllocateFromVolumeGeneric(ITMVoxelVolume <TVoxelTarget, ITMVoxelBlockHash>* targetVolume,
	                               ITMVoxelVolume <TVoxelSource, ITMVoxelBlockHash>* sourceVolume);

	ORUtils::MemoryBlock<unsigned char>* sourceSceneHashBlockStates;
	ORUtils::MemoryBlock<unsigned char>* targetSceneHashBlockStates;
	ORUtils::MemoryBlock<Vector3s>* targetSceneHashBlockCoordinates;
};


}// namespace ITMLib


