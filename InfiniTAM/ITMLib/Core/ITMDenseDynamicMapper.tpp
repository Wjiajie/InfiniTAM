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
#include "../Utils/ITMSceneStatisticsCalculator.h"
#include "../Utils/ITMPrintHelpers.h"

using namespace ITMLib;

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::ITMDenseDynamicMapper(const ITMLibSettings* settings) :
	recordNextFrameWarps(false)
	{
	canonicalSceneReconstructor = ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxelCanonical, TIndex>(
			settings->deviceType);
	liveSceneReconstructor = ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxelLive, TIndex>(
			settings->deviceType);
	swappingEngine = settings->swappingMode != ITMLibSettings::SWAPPINGMODE_DISABLED
	                 ? ITMSwappingEngineFactory::MakeSwappingEngine<TVoxelCanonical, TIndex>(settings->deviceType)
	                 : NULL;

	sceneMotionTracker = ITMTrackerFactory::MakeSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>(settings);
	swappingMode = settings->swappingMode;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::~ITMDenseDynamicMapper() {
	delete canonicalSceneReconstructor;
	delete swappingEngine;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void
ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::ResetCanonicalScene(
		ITMScene<TVoxelCanonical, TIndex>* scene) const {
	canonicalSceneReconstructor->ResetScene(scene);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::ResetLiveScene(
		ITMScene<TVoxelLive, TIndex>* live_scene) const {
	liveSceneReconstructor->ResetScene(live_scene);
}

//BEGIN _DEBUG
template<typename TVoxel, typename TIndex>
static void PrintSceneStats(ITMScene<TVoxel, TIndex>* scene, const char* sceneName){
	ITMSceneStatisticsCalculator<TVoxel, TIndex> calculatorLive;
	std::cout << "=== Stats for scene '" << sceneName << "' ===" << std::endl;
	std::cout << "    Total voxel count: " << calculatorLive.ComputeAllocatedVoxelCount(scene) << std::endl;
	std::cout << "    Non-truncated voxel count: " << calculatorLive.ComputeNonTruncatedVoxelCount(scene) << std::endl;
	std::cout << "    +1.0 voxel count: " << calculatorLive.ComputeVoxelWithValueCount(scene,1.0f)  << std::endl;
	std::vector<int> allocatedHashes = calculatorLive.GetFilledHashBlockIds(scene);
	std::cout << "    Allocated hash count: " << allocatedHashes.size() << std::endl;
	std::cout << "    Non-truncated SDF sum: " << calculatorLive.ComputeNonTruncatedVoxelSdfSum(scene) << std::endl;
	std::cout << "    Truncated SDF sum: " << calculatorLive.ComputeTruncatedVoxelSdfSum(scene) << std::endl;
};
//END _DEBUG

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::ProcessFrame(const ITMView* view,
                                                                              const ITMTrackingState* trackingState,
                                                                              ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
                                                                              ITMScene<TVoxelLive, TIndex>*& liveScene,
                                                                              ITMRenderState* renderState) {
  //_DEBUG
//	if(sceneMotionTracker->currentFrameIx == 0){
//		// clear out the live-frame SDF and the target-live-frame SDF
//		canonicalSceneReconstructor->ResetScene(canonicalScene);
//		//** construct the new live-frame SDF
//		// allocation
//		canonicalSceneReconstructor->AllocateSceneFromDepth(canonicalScene, view, trackingState, renderState);
//		// integration
//		canonicalSceneReconstructor->IntegrateIntoScene(canonicalScene, view, trackingState, renderState);
//		sceneMotionTracker->currentFrameIx++;
//		return;
//	}
	//BEGIN __DEBUG
	if(this->recordNextFrameWarps){
		std::cout << bright_cyan << "MAPPING FRAME " << sceneMotionTracker->GetFrameIndex() <<  " (WITH RECORDING ON)" << reset << std::endl;
	}else{
		std::cout << bright_cyan << "MAPPING FRAME " << sceneMotionTracker->GetFrameIndex() << reset << std::endl;
	}
	//END __DEBUG

	// clear out the live-frame SDF
	liveSceneReconstructor->ResetScene(liveScene);
	//** construct the new live-frame SDF
	// allocation
	liveSceneReconstructor->AllocateSceneFromDepth(liveScene, view, trackingState, renderState);
	// integration
	liveSceneReconstructor->IntegrateIntoScene(liveScene, view, trackingState, renderState);



	sceneMotionTracker->TrackMotion(canonicalScene, liveScene, this->recordNextFrameWarps, liveSceneReconstructor);
	sceneMotionTracker->FuseFrame(canonicalScene, liveScene);

	//_DEBUG
	PrintSceneStats(liveScene, "Live before fusion");
	PrintSceneStats(canonicalScene, "Canonical after fusion");

	// clear out the live-frame SDF, to prepare it for warped canonical
	//liveSceneRecoEngine->ResetScene(liveScene);
	//sceneMotionTracker->WarpCanonicalToLive(canonicalScene,liveScene);


// _DEBUG
//	std::cout << "Known voxels in canonical scene after fusion: "  << calculator2.ComputeKnownVoxelCount(canonicalScene) << std::endl;


	if (swappingEngine != NULL) {
		// swapping: CPU -> GPU
		if (swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED)
			swappingEngine->IntegrateGlobalIntoLocal(canonicalScene, renderState);

		// swapping: GPU -> CPU
		switch (swappingMode) {
			case ITMLibSettings::SWAPPINGMODE_ENABLED:
				swappingEngine->SaveToGlobalMemory(canonicalScene, renderState);
				break;
			case ITMLibSettings::SWAPPINGMODE_DELETE:
				swappingEngine->CleanLocalMemory(canonicalScene, renderState);
				break;
			case ITMLibSettings::SWAPPINGMODE_DISABLED:
				break;
		}
	}
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void
ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::UpdateVisibleList(
		const ITMView* view,
		const ITMTrackingState* trackingState,
		ITMScene<TVoxelLive, TIndex>* scene, ITMRenderState* renderState,
		bool resetVisibleList) {
	liveSceneReconstructor->AllocateSceneFromDepth(scene, view, trackingState, renderState, true, resetVisibleList);
}

