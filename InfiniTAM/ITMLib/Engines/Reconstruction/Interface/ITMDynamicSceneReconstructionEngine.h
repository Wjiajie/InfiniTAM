//  ================================================================
//  Created by Gregory Kramida on 5/7/18.
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

//TODO: add to CMake & complete implementations or remove -Greg Github(Algomorph)

#pragma once

#include <math.h>

#include "../../../Objects/RenderStates/ITMRenderState.h"
#include "../../../Objects/Scene/ITMScene.h"
#include "../../../Objects/Tracking/ITMTrackingState.h"
#include "../../../Objects/Views/ITMView.h"

namespace ITMLib
{
/** \brief
	Interface to engines implementing the main KinectFusion
	depth integration process in dynamic fusion settings that involve 2 different SDFs with different voxel types.

	These classes basically manage
	an ITMLib::Objects::ITMScene and fuse new image information
	into them.
*/
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMDynamicSceneReconstructionEngine
{

public:

	ITMDynamicSceneReconstructionEngine(void) = default;
	virtual ~ITMDynamicSceneReconstructionEngine(void) = default;

	/**
	 * \brief Clears given scene, then uses the depth image from provided live view to generate an SDF
	 * voxel representation
	 * \param scene output scene
	 * \param view input view
	 * \param trackingState state of tracking
	 * \param renderState state of rendering the stuff
	 */
	virtual void GenerateRawLiveSceneFromView(ITMScene<TVoxelLive,TIndex> *scene, const ITMView *view,
	                                          const ITMTrackingState *trackingState, const ITMRenderState *renderState) = 0;

	/**
	 * \brief Fuses the live scene into the canonical scene based on the motion warp of the canonical scene
	 * \details Typically called after TrackMotion is called
	 * \param canonicalScene the canonical voxel grid, representing the state at the beginning of the sequence
	 * \param liveScene the live voxel grid, a TSDF generated from a single recent depth image
	 */
	virtual void FuseFrame(ITMScene <TVoxelCanonical, TIndex>* canonicalScene, ITMScene <TVoxelLive, TIndex>* liveScene,
	                       int liveSourceFieldIndex) = 0;
protected:


	/** Update the voxel blocks by integrating depth and
	possibly colour information from the given view.
*/
	virtual void IntegrateIntoScene(ITMScene<TVoxelLive,TIndex> *scene, const ITMView *view,
	                                const ITMTrackingState *trackingState, const ITMRenderState *renderState) = 0;

};
}//namespace ITMLib