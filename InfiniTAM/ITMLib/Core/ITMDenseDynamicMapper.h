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

#include "../Engines/Reconstruction/Interface/ITMSceneReconstructionEngine.h"
#include "../Utils/ITMLibSettings.h"
#include "../Engines/Swapping/Interface/ITMSwappingEngine.h"
#include "../Trackers/Interface/ITMSceneMotionTracker.h"

namespace ITMLib{
template<class TVoxel, class TIndex>
class ITMDenseDynamicMapper {

	private:
		ITMSceneReconstructionEngine<TVoxel,TIndex> *sceneRecoEngine;
		ITMSwappingEngine<TVoxel,TIndex> *swappingEngine;
		ITMSceneMotionTracker<TVoxel,TIndex> *sceneMotionTracker;
		ITMLibSettings::SwappingMode swappingMode;

	public:
		void ResetScene(ITMScene<TVoxel,TIndex> *scene) const;

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
		* \param canonical_scene
		* \param canonical_scene
		* \param renderState
		*/
		void ProcessFrame(const ITMView* view,
		                  const ITMTrackingState* trackingState,
		                  ITMScene <TVoxel, TIndex>* canonical_scene,
		                  ITMScene <TVoxel, TIndex>* live_scene,
		                  ITMRenderState* renderState_live);

		/// Update the visible list (this can be called to update the visible list when fusion is turned off)
		void UpdateVisibleList(const ITMView *view, const ITMTrackingState *trackingState, ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState, bool resetVisibleList = false);

		/** \brief Constructor
		    Ommitting a separate image size for the depth images
		    will assume same resolution as for the RGB images.
		*/
		explicit ITMDenseDynamicMapper(const ITMLibSettings *settings);
		~ITMDenseDynamicMapper();

};
}//namespace ITMLib

