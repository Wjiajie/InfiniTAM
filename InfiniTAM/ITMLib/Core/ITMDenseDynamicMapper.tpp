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
//stdlib
//#include <limits>
#include <chrono>


//local
#include "ITMDenseDynamicMapper.h"
#include "../Engines/Reconstruction/ITMSceneReconstructionEngineFactory.h"
#include "../Engines/Swapping/ITMSwappingEngineFactory.h"
#include "../Trackers/ITMTrackerFactory.h"
#include "../Objects/Scene/ITMSceneManipulation.h"

using namespace ITMLib;

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::ITMDenseDynamicMapper(const ITMLibSettings* settings) {
	sceneRecoEngine = ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxelCanonical, TIndex>(
			settings->deviceType);
	liveSceneRecoEngine = ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxelLive, TIndex>(
			settings->deviceType);
	swappingEngine = settings->swappingMode != ITMLibSettings::SWAPPINGMODE_DISABLED
	                 ? ITMSwappingEngineFactory::MakeSwappingEngine<TVoxelCanonical, TIndex>(settings->deviceType) : NULL;
	sceneMotionTracker = ITMTrackerFactory::MakeSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>(settings->deviceType, settings->sceneParams);
	swappingMode = settings->swappingMode;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::~ITMDenseDynamicMapper() {
	delete sceneRecoEngine;
	delete swappingEngine;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::ResetScene(ITMScene<TVoxelCanonical, TIndex>* scene) const {
	sceneRecoEngine->ResetScene(scene);
}
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::ResetLiveScene(ITMScene<TVoxelLive, TIndex>* live_scene) const {
	liveSceneRecoEngine->ResetScene(live_scene);
}




template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::ProcessFrame(const ITMView* view,
                                                         const ITMTrackingState* trackingState,
                                                         ITMScene<TVoxelCanonical, TIndex>* canonical_scene,
                                                         ITMScene<TVoxelLive, TIndex>* live_scene,
                                                         ITMRenderState* renderState) {


	// clear out the live-frame SDF
	liveSceneRecoEngine->ResetScene(live_scene);


	//** construct the new live-frame SDF
	// allocation
	liveSceneRecoEngine->AllocateSceneFromDepth(live_scene, view, trackingState, renderState);
	// integration
	liveSceneRecoEngine->IntegrateIntoScene(live_scene, view, trackingState, renderState);

	sceneMotionTracker->ProcessFrame(canonical_scene, live_scene);

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

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void
ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::UpdateVisibleList(const ITMView* view, const ITMTrackingState* trackingState,
                                                         ITMScene<TVoxelLive, TIndex>* scene, ITMRenderState* renderState,
                                                         bool resetVisibleList) {
	liveSceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState, renderState, true, resetVisibleList);
}

//TODO: remove
//template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
//void
//ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::ProcessInitialFrame(const ITMView* view, const ITMTrackingState* trackingState,
//                                                           ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
//                                                           ITMRenderState* renderState) {
//
//	//** construct the new live-frame SDF
//	// allocation
//	sceneRecoEngine->AllocateSceneFromDepth(canonicalScene, view, trackingState, renderState);
//
//	// integration
//	sceneRecoEngine->IntegrateIntoScene(canonicalScene, view, trackingState, renderState);
//
//	if (swappingEngine != NULL) {
//		// swapping: CPU -> GPU
//		if (swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED)
//			swappingEngine->IntegrateGlobalIntoLocal(canonicalScene, renderState);
//
//		// swapping: GPU -> CPU
//		switch (swappingMode) {
//			case ITMLibSettings::SWAPPINGMODE_ENABLED:
//				swappingEngine->SaveToGlobalMemory(canonicalScene, renderState);
//				break;
//			case ITMLibSettings::SWAPPINGMODE_DELETE:
//				swappingEngine->CleanLocalMemory(canonicalScene, renderState);
//				break;
//			case ITMLibSettings::SWAPPINGMODE_DISABLED:
//				break;
//		}
//	}
//}

