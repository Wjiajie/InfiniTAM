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

#include <cmath>

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

	ITMDynamicSceneReconstructionEngine() = default;
	virtual ~ITMDynamicSceneReconstructionEngine() = default;


	virtual void UpdateVisibleList(ITMScene<TVoxelLive, TIndex>* scene, const ITMView* view, const ITMTrackingState* trackingState,const ITMRenderState* renderState, bool resetVisibleList) = 0;
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
	 * \param liveSourceFieldIndex index of the sdf field to use at live scene voxels
	 */
	virtual void FuseLiveIntoCanonicalSdf(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                                      ITMScene<TVoxelLive, TIndex>* liveScene,
	                                      int liveSourceFieldIndex) = 0;

	/**
	 * \brief apply warp vectors to live scene: compute the the target SDF fields in live scene using trilinear lookup
	 * at corresponding warps from the source SDF fields in live scene
	 * \param canonicalScene canonical scene, where the warp update vectors are specified
	 * \param liveScene live scene (voxel grid)
	 * \param sourceSdfIndex index of the source SDF field in each live voxel
	 * \param targetSdfIndex index of the target SDF field in each live voxel
	 */
	virtual void WarpScene(ITMScene <TVoxelCanonical, TIndex>* canonicalScene,
	                       ITMScene <TVoxelLive, TIndex>* liveScene, int sourceSdfIndex, int targetSdfIndex) = 0;

	/**
	 * \brief apply warp update vectors to live scene: compute the the target SDF fields in live scene using trilinear lookup
	 * at corresponding warp updates from the source SDF fields in live scene
	 * \param canonicalScene canonical scene, where the warp update vectors are specified
	 * \param liveScene live scene (voxel grid)
	 * \param sourceSdfIndex index of the source SDF field in each live voxel
	 * \param targetSdfIndex index of the target SDF field in each live voxel
	 */
	virtual void UpdateWarpedScene(ITMScene <TVoxelCanonical, TIndex>* canonicalScene,
	                               ITMScene <TVoxelLive, TIndex>* liveScene, int sourceSdfIndex,
	                               int targetSdfIndex) = 0;
protected:


	/** Update the voxel blocks by integrating depth and
	possibly colour information from the given view.*/
	virtual void IntegrateIntoScene(ITMScene<TVoxelLive,TIndex> *scene, const ITMView *view,
	                                const ITMTrackingState *trackingState, const ITMRenderState *renderState) = 0;

};
}//namespace ITMLib