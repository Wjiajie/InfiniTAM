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
#include <limits>
#include "ITMDenseDynamicMapper.h"
#include "../Engines/Reconstruction/ITMSceneReconstructionEngineFactory.h"
#include "../Engines/Swapping/ITMSwappingEngineFactory.h"
#include "../Trackers/ITMTrackerFactory.h"

using namespace ITMLib;

template<class TVoxel, class TIndex>
ITMDenseDynamicMapper<TVoxel, TIndex>::ITMDenseDynamicMapper(const ITMLibSettings* settings) {
	sceneRecoEngine = ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxel, TIndex>(
			settings->deviceType);
	swappingEngine = settings->swappingMode != ITMLibSettings::SWAPPINGMODE_DISABLED
	                 ? ITMSwappingEngineFactory::MakeSwappingEngine<TVoxel, TIndex>(settings->deviceType) : NULL;
	sceneMotionTracker = ITMTrackerFactory::MakeSceneMotionTracker<TVoxel, TIndex>(settings->deviceType);
	swappingMode = settings->swappingMode;
}

template<class TVoxel, class TIndex>
ITMDenseDynamicMapper<TVoxel, TIndex>::~ITMDenseDynamicMapper() {
	delete sceneRecoEngine;
	delete swappingEngine;
}

template<class TVoxel, class TIndex>
void ITMDenseDynamicMapper<TVoxel, TIndex>::ResetScene(ITMScene<TVoxel, TIndex>* scene) const {
	sceneRecoEngine->ResetScene(scene);
}

template<class TVoxel, class TIndex>
void ITMDenseDynamicMapper<TVoxel, TIndex>::ProcessFrame(const ITMView* view,
                                                         const ITMTrackingState* trackingState,
                                                         ITMScene<TVoxel, TIndex>* canonical_scene,
                                                         ITMScene<TVoxel, TIndex>* live_scene,
                                                         ITMRenderState* renderState) {
	// clear out the live-frame SDF
	sceneRecoEngine->ResetScene(live_scene);

	//** construct the live-frame SDF
	// allocation
	sceneRecoEngine->AllocateSceneFromDepth(live_scene, view, trackingState, renderState);
	// integration
	sceneRecoEngine->IntegrateIntoScene(live_scene, view, trackingState, renderState);


	sceneMotionTracker->ProcessFrame(canonical_scene, live_scene);


	//TODO: update canonical_scene from live_scene

	if (swappingEngine != NULL) {
		// swapping: CPU -> GPU
		if (swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED)
			swappingEngine->IntegrateGlobalIntoLocal(canonical_scene, renderState);

		// swapping: GPU -> CPU
		switch (swappingMode) {
			case ITMLibSettings::SWAPPINGMODE_ENABLED:
				swappingEngine->SaveToGlobalMemory(canonical_scene, renderState);
				break;
			case ITMLibSettings::SWAPPINGMODE_DELETE:
				swappingEngine->CleanLocalMemory(canonical_scene, renderState);
				break;
			case ITMLibSettings::SWAPPINGMODE_DISABLED:
				break;
		}
	}
}

template<class TVoxel, class TIndex>
void
ITMDenseDynamicMapper<TVoxel, TIndex>::UpdateVisibleList(const ITMView* view, const ITMTrackingState* trackingState,
                                                         ITMScene<TVoxel, TIndex>* scene, ITMRenderState* renderState,
                                                         bool resetVisibleList) {
	sceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState, renderState, true, resetVisibleList);
}