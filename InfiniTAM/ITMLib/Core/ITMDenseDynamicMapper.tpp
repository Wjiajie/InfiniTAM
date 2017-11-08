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
#include "../ITMLibDefines.h"
#include "../Engines/Reconstruction/ITMSceneReconstructionEngineFactory.h"
#include "../Engines/Swapping/ITMSwappingEngineFactory.h"
#include "../Trackers/ITMTrackerFactory.h"
#include "../Objects/Scene/ITMSceneManipulation.h"

using namespace ITMLib;

template<class TVoxel, class TIndex>
ITMDenseDynamicMapper<TVoxel, TIndex>::ITMDenseDynamicMapper(const ITMLibSettings* settings) {
	sceneRecoEngine = ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxel, TIndex>(
			settings->deviceType);
	liveSceneRecoEngine = ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<ITMVoxelAux, TIndex>(
			settings->deviceType);
	swappingEngine = settings->swappingMode != ITMLibSettings::SWAPPINGMODE_DISABLED
	                 ? ITMSwappingEngineFactory::MakeSwappingEngine<TVoxel, TIndex>(settings->deviceType) : NULL;
	sceneMotionTracker = ITMTrackerFactory::MakeSceneMotionTracker<TVoxel, TIndex>(settings->deviceType, settings->sceneParams);
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
void ITMDenseDynamicMapper<TVoxel, TIndex>::ResetLiveScene(ITMScene<ITMVoxelAux, TIndex>* live_scene) const {
	liveSceneRecoEngine->ResetScene(live_scene);
}




template<class TVoxel, class TIndex>
void ITMDenseDynamicMapper<TVoxel, TIndex>::ProcessFrame(const ITMView* view,
                                                         const ITMTrackingState* trackingState,
                                                         ITMScene<TVoxel, TIndex>* canonical_scene,
                                                         ITMScene<ITMVoxel_f_rgb_conf, TIndex>* live_scene,
                                                         ITMRenderState* renderState) {


	// clear out the live-frame SDF
	liveSceneRecoEngine->ResetScene(live_scene);

	//** construct the new live-frame SDF
	//_DEBUG -- restore
	// allocation
	//liveSceneRecoEngine->AllocateSceneFromDepth(live_scene, view, trackingState, renderState);
	// integration
	//liveSceneRecoEngine->IntegrateIntoScene(live_scene, view, trackingState, renderState);
	//_DEBUG
	auto start = std::chrono::steady_clock::now();
	CopySceneWithOffset_CPU(*live_scene,*canonical_scene, Vector3i(5,5,0));
	auto end = std::chrono::steady_clock::now();
	auto diff = end - start;
	std::cout << "Scene copy time: " << std::chrono::duration <double, std::milli> (diff).count() << " ms" << std::endl;

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

template<class TVoxel, class TIndex>
void
ITMDenseDynamicMapper<TVoxel, TIndex>::UpdateVisibleList(const ITMView* view, const ITMTrackingState* trackingState,
                                                         ITMScene<TVoxel, TIndex>* scene, ITMRenderState* renderState,
                                                         bool resetVisibleList) {
	sceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState, renderState, true, resetVisibleList);
}

template<class TVoxel, class TIndex>
void
ITMDenseDynamicMapper<TVoxel, TIndex>::ProcessInitialFrame(const ITMView* view, const ITMTrackingState* trackingState,
                                                           ITMScene<TVoxel, TIndex>* canonical_scene,
                                                           ITMRenderState* renderState) {

	//** construct the new live-frame SDF
	// allocation
	sceneRecoEngine->AllocateSceneFromDepth(canonical_scene, view, trackingState, renderState);

	// integration
	sceneRecoEngine->IntegrateIntoScene(canonical_scene, view, trackingState, renderState);

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

