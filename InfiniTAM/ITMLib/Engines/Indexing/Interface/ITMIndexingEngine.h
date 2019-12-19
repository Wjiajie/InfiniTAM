//  ================================================================
//  Created by Gregory Kramida on 11/1/19.
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

#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "../../../Utils/ITMHashBlockProperties.h"
#include "../../../Objects/Views/ITMView.h"
#include "../../../Objects/Tracking/ITMTrackingState.h"
#include "../../../Objects/RenderStates/ITMRenderState.h"
#include "../../Common/ITMWarpEnums.h"

namespace ITMLib {
/**
 * \brief A utility for allocating additional space within or around a voxel volume (expanding it) based on various inputs
 * \details Note: if a volume index has strict bounds, as in the case of plain voxel array, does not grow or shrink those bounds.
 * \tparam TVoxel type of voxels
 * \tparam TIndex type of index
 */
template<typename TVoxel, typename TIndex>
class ITMIndexingEngineInterface {

	/**
	 * \brief Given a view with a new depth image, compute the
		visible blocks, allocate them and update the hash
		table, as well as the visible block list,
	    so that the new image data can be integrated.
	 * \param scene [out] the scene whose hash needs additional allocations
	 * \param view [in] a view with a new depth image
	 * \param trackingState [in] tracking state that corresponds to the given view
	 * \param renderState [in] the current renderState with information about which hash entries are visible
	 * \param onlyUpdateVisibleList [in] whether we want to allocate only the hash entry blocks currently visible
	 * \param resetVisibleList  [in] reset visibility list upon completion
	 */
	virtual void
	AllocateFromDepth(ITMVoxelVolume<TVoxel, TIndex>* scene, const ITMView* view, const ITMTrackingState* trackingState,
	                  bool onlyUpdateVisibleList, bool resetVisibleList) = 0;

	/**
	 * \brief Given a view with a new depth image, compute the
		visible blocks, allocate them and update the hash
		table so that the new image data can be integrated.
	 * \param scene [out] the scene whose hash needs additional allocations
	 * \param view [in] a view with a new depth image
	 * \param depth_camera_matrix [in] transformation of the camera from world origin (initial position) to
	 * where the camera was at the given view's frame
	 * \param renderState [in] the current renderState with information about which hash entries are visible
	 * \param onlyUpdateVisibleList [in] whether we want to allocate only the hash entry blocks currently visible
	 * \param resetVisibleList  [in] reset visibility list upon completion
	 */
	virtual void
	AllocateFromDepth(ITMVoxelVolume<TVoxel, TIndex>* scene, const ITMView* view,
	                  const Matrix4f& depth_camera_matrix = Matrix4f::Identity(),
	                  bool onlyUpdateVisibleList = false, bool resetVisibleList = false) = 0;
};

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
class ITMIndexingEngine :
		public ITMIndexingEngineInterface<TVoxel, TIndex> {
private:
	ITMIndexingEngine() = default;
public:
	static ITMIndexingEngine& Instance() {
		static ITMIndexingEngine instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	ITMIndexingEngine(ITMIndexingEngine const&) = delete;
	void operator=(ITMIndexingEngine const&) = delete;

	virtual void AllocateFromDepth(ITMVoxelVolume<TVoxel, TIndex>* scene, const ITMView* view,
	                               const ITMTrackingState* trackingState, bool onlyUpdateVisibleList,
	                               bool resetVisibleList) override;

	virtual void AllocateFromDepth(ITMVoxelVolume<TVoxel, TIndex>* volume, const ITMView* view,
	                               const Matrix4f& depth_camera_matrix = Matrix4f::Identity(),
	                               bool onlyUpdateVisibleList = false, bool resetVisibleList = false) override;


	template<typename TVoxelTarget, typename TVoxelSource>
	void AllocateUsingOtherVolume(ITMVoxelVolume<TVoxelTarget, TIndex>* targetVolume,
	                              ITMVoxelVolume<TVoxelSource, TIndex>* sourceVolume);

	template<typename TVoxelTarget, typename TVoxelSource>
	void AllocateUsingOtherVolumeExpanded(ITMVoxelVolume<TVoxelTarget, TIndex>* targetVolume,
	                                      ITMVoxelVolume<TVoxelSource, TIndex>* sourceVolume);


	template<typename TVoxelTarget, typename TVoxelSource>
	void AllocateUsingOtherVolumeAndSetVisibilityExpanded(ITMVoxelVolume<TVoxelTarget, TIndex>* targetVolume,
	                                                      ITMVoxelVolume<TVoxelSource, TIndex>* sourceVolume,
	                                                      ITMView* view, const Matrix4f& depth_camera_matrix = Matrix4f::Identity());

	template<WarpType TWarpType, typename TWarp>
	void AllocateFromWarpedVolume(
			ITMVoxelVolume<TWarp, TIndex>* warpField,
			ITMVoxelVolume<TVoxel, TIndex>* sourceTSDF,
			ITMVoxelVolume<TVoxel, TIndex>* targetTSDF);

};


}//namespace ITMLib

