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

#include "../Engines/Reconstruction/Interface/ITMDynamicSceneReconstructionEngine.h"
#include "../Utils/ITMLibSettings.h"
#include "../Engines/Swapping/Interface/ITMSwappingEngine.h"
#include "../Trackers/Interface/ITMSceneMotionTracker.h"

namespace ITMLib {
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMDenseDynamicMapper {

public:
	// region ============================================ CONSTRUCTORS / DESTRUCTORS ==================================

	/** \brief Constructor
		Ommitting a separate image size for the depth images
		will assume same resolution as for the RGB images.
	*/
	explicit ITMDenseDynamicMapper(const ITMLibSettings* settings);
	~ITMDenseDynamicMapper();
	// endregion
	// region ========================================== MEMBER FUNCTIONS ==============================================

	void ResetCanonicalScene(ITMScene<TVoxelCanonical, TIndex>* scene) const;
	void ResetLiveScene(ITMScene<TVoxelLive, TIndex>* live_scene) const;

	/**
	* \brief Tracks motions of all points between frames and fuses the new data into the canonical frame
	* 1) Generates new SDF from current view data/point cloud
	* 2) Maps the new SDF to the previous SDF to generate the warp field delta
	* 3) Updates the warp field with the warp field delta
	* 4) Fuses the new data using the updated warp field into the canonical frame
	* 5) Re-projects the (updated) canonical data into the live frame using the updated warp field
	* \tparam TVoxel type of voxel in the voxel grid / implicit function
	* \tparam TIndex type of index used by the voxel grid
	* \param view view with the new (incoming) depth/color data
	* \param trackingState best estimate of the camera tracking from previous to new frame so far
	* \param canonicalScene - canonical/reference 3D scene where all data is fused/aggregated
	* \param liveScene - live/target 3D scene generated from the incoming single frame of the video
	* \param renderState
	*/
	void ProcessFrame(const ITMView* view,
	                  const ITMTrackingState* trackingState,
	                  ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                  ITMScene<TVoxelLive, TIndex>*& liveScene,
	                  ITMRenderState* renderState);

	//_DEBUG
	void SaveScenes(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                ITMScene<TVoxelLive, TIndex>* liveScene);

	//_DEBUG
	void BeginProcessingFrame(const ITMView* view,
	                          const ITMTrackingState* trackingState,
	                          ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                          ITMScene<TVoxelLive, TIndex>*& liveScene,
	                          ITMRenderState* renderState_live);

	bool UpdateCurrentFrame(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                        ITMScene<TVoxelLive, TIndex>*& liveScene, ITMRenderState* renderState);

	/// Update the visible list (this can be called to update the visible list when fusion is turned off)
	void
	UpdateVisibleList(const ITMView* view, const ITMTrackingState* trackingState, ITMScene<TVoxelLive, TIndex>* scene,
	                  ITMRenderState* renderState, bool resetVisibleList = false);
	// endregion

	// on next call to ProcessFrame, record the scenes before warp update, and then record the warp update itself
	bool recordNextFrameWarps;
private:
	// region =========================================== MEMBER VARIABLES =============================================

	ITMDynamicSceneReconstructionEngine<TVoxelCanonical, TVoxelLive, TIndex>* sceneReconstructor;
	ITMSwappingEngine<TVoxelCanonical, TIndex>* swappingEngine;
	ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>* sceneMotionTracker;
	ITMLibSettings::SwappingMode swappingMode;
	// endregion

};
}//namespace ITMLib

