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
	//TODO: move Fusion here (?) -Greg (GitHub: Algomorph)
	/** Clear and reset a scene to set up a new empty
		one.
	*/
	virtual void ResetCanonicalScene(ITMScene<TVoxelCanonical, TIndex> *scene) = 0;
	virtual void ResetLiveScene(ITMScene<TVoxelCanonical, TIndex> *scene) = 0;


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
	virtual void AllocateSceneFromDepth(ITMScene<TVoxelLive,TIndex> *scene, const ITMView *view, const ITMTrackingState *trackingState,
	                                    const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false) = 0;

	/** Update the voxel blocks by integrating depth and
		possibly colour information from the given view.
	*/
	virtual void IntegrateIntoScene(ITMScene<TVoxelLive,TIndex> *scene, const ITMView *view, const ITMTrackingState *trackingState,
	                                const ITMRenderState *renderState) = 0;

	ITMDynamicSceneReconstructionEngine(void) { }
	virtual ~ITMDynamicSceneReconstructionEngine(void) { }
};
}