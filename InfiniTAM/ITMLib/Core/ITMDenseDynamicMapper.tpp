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

//boost
#include <boost/filesystem.hpp>

//local
#include "ITMDenseDynamicMapper.h"
#include "../Engines/Reconstruction/ITMDynamicSceneReconstructionEngineFactory.h"
#include "../Engines/Swapping/ITMSwappingEngineFactory.h"
#include "../SceneMotionTrackers/ITMSceneMotionTrackerFactory.h"
#include "../Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../Engines/Manipulation/CUDA/ITMSceneManipulationEngine_CUDA.h"
#include "../Utils/Analytics/ITMSceneStatisticsCalculator.h"
#include "../Utils/ITMPrintHelpers.h"
#include "../Utils/Visualization/ITMSceneSliceVisualizer2D.h"
#include "../Utils/Analytics/ITMBenchmarkUtils.h"
#include "../Utils/FileIO/ITMSceneLogger.h"
#include "../../Tests/TestUtils.h"


using namespace ITMLib;

namespace bench = ITMLib::Bench;

// region ========================================= DEBUG PRINTING =====================================================
//_DEBUG
template<typename TVoxel, typename TIndex>
inline static void PrintSceneStatistics(
		ITMScene<TVoxel, TIndex>* scene,
		std::string description) {
	ITMSceneStatisticsCalculator<TVoxel, TIndex>& calculator = ITMSceneStatisticsCalculator<TVoxel, TIndex>::Instance();
	std::cout << green << "=== Stats for scene '" << description << "' ===" << reset << std::endl;
	std::cout << "    Total voxel count: " << calculator.ComputeAllocatedVoxelCount(scene) << std::endl;
	std::cout << "    NonTruncated voxel count: " << calculator.ComputeNonTruncatedVoxelCount(scene) << std::endl;
	std::cout << "    +1.0 voxel count: " << calculator.ComputeVoxelWithValueCount(scene, 1.0f) << std::endl;
	std::vector<int> allocatedHashes = calculator.GetFilledHashBlockIds(scene);
	std::cout << "    Allocated hash count: " << allocatedHashes.size() << std::endl;
	std::cout << "    NonTruncated SDF sum: " << calculator.ComputeNonTruncatedVoxelAbsSdfSum(scene) << std::endl;
	std::cout << "    Truncated SDF sum: " << calculator.ComputeTruncatedVoxelAbsSdfSum(scene) << std::endl;

};

inline static void PrintOperationStatus(const char* status) {
	std::cout << bright_cyan << status << reset << std::endl;
}
// endregion ===========================================================================================================

// region ===================================== CONSTRUCTORS / DESTRUCTORS =============================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::ITMDenseDynamicMapper() :
		sceneReconstructor(
				ITMDynamicSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxelCanonical, TVoxelLive, TIndex>
						(ITMLibSettings::Instance().deviceType)),
		sceneMotionTracker(
				ITMSceneMotionTrackerFactory::MakeSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>()),
		swappingEngine(ITMLibSettings::Instance().swappingMode != ITMLibSettings::SWAPPINGMODE_DISABLED
		               ? ITMSwappingEngineFactory::MakeSwappingEngine<TVoxelCanonical, TIndex>(
						ITMLibSettings::Instance().deviceType)
		               : nullptr),
		swappingMode(ITMLibSettings::Instance().swappingMode),
		parameters{ITMLibSettings::Instance().sceneTrackingMaxOptimizationIterationCount,
		           ITMLibSettings::Instance().sceneTrackingOptimizationVectorUpdateThresholdMeters,
		           ITMLibSettings::Instance().sceneTrackingOptimizationVectorUpdateThresholdMeters /
		           ITMLibSettings::Instance().sceneParams.voxelSize},
		analysisFlags{ITMLibSettings::Instance().restrictZtrackingForDebugging,
		              ITMLibSettings::Instance().FocusCoordinatesAreSpecified()},
		focusCoordinates(ITMLibSettings::Instance().GetFocusCoordinates()) {}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::~ITMDenseDynamicMapper() {
	delete sceneReconstructor;
	delete swappingEngine;
	delete sceneMotionTracker;
}
//endregion ================================= C/D ======================================================================


//TODO: deprecate in favor of SceneManipulation
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void
ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::ResetCanonicalScene(
		ITMScene<TVoxelCanonical, TIndex>* scene) const {
	switch (ITMLibSettings::Instance().deviceType) {
		case ITMLibSettings::DEVICE_CPU:
			ITMSceneManipulationEngine_CPU<TVoxelCanonical, TIndex>::ResetScene(scene);
			break;
		case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			ITMSceneManipulationEngine_CUDA<TVoxelCanonical, TIndex>::ResetScene(scene);
#endif
			break;
		case ITMLibSettings::DEVICE_METAL:
			DIEWITHEXCEPTION_REPORTLOCATION("NOT IMPLEMENTED");
			break;
	}
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::ResetLiveScene(
		ITMScene<TVoxelLive, TIndex>* scene) const {
	switch (ITMLibSettings::Instance().deviceType) {
		case ITMLibSettings::DEVICE_CPU:
			ITMSceneManipulationEngine_CPU<TVoxelLive, TIndex>::ResetScene(scene);
			break;
		case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			ITMSceneManipulationEngine_CUDA<TVoxelLive, TIndex>::ResetScene(scene);
#endif
			break;
		case ITMLibSettings::DEVICE_METAL:
			DIEWITHEXCEPTION_REPORTLOCATION("NOT IMPLEMENTED");
			break;
	}
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::ProcessInitialFrame(
		const ITMView* view, const ITMTrackingState* trackingState,
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene,
		ITMRenderState* renderState) {
	PrintOperationStatus("Generating raw live frame from view...");
	bench::StartTimer("GenerateRawLiveFrame");
	sceneReconstructor->GenerateRawLiveSceneFromView(liveScene, view, trackingState, renderState);
	bench::StopTimer("GenerateRawLiveFrame");
	//** prepare canonical for new frame
	PrintOperationStatus("Fusing data from live frame into canonical frame...");
	//** fuse the live into canonical directly
	bench::StartTimer("FuseLiveIntoCanonicalSdf");
	sceneReconstructor->FuseLiveIntoCanonicalSdf(canonicalScene, liveScene, 0);
	bench::StopTimer("FuseLiveIntoCanonicalSdf");
	ProcessSwapping(canonicalScene, renderState);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::InitializeProcessing(
		const ITMView* view, const ITMTrackingState* trackingState, ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
		ITMScene<TVoxelLive, TIndex>* liveScene, ITMRenderState* renderState) {


	PrintOperationStatus("Generating raw live frame from view...");
	bench::StartTimer("GenerateRawLiveFrame");
	sceneReconstructor->GenerateRawLiveSceneFromView(liveScene, view, trackingState, renderState);
	bench::StopTimer("GenerateRawLiveFrame");


	PrintOperationStatus(
			"Initializing live frame SDF by mapping from raw live SDF to warped SDF based on previous-frame warp...");
	bench::StartTimer("TrackMotion_35_Initialize");
	//sceneReconstructor->WarpScene(canonicalScene, liveScene, 0, 1);
	sceneReconstructor->CopyIndexedScene(liveScene, 0, 1);
	//sceneReconstructor->UpdateWarpedScene(canonicalScene, liveScene, 0, 1);

	sceneMotionTracker->ClearOutFlowWarp(canonicalScene);
	bench::StopTimer("TrackMotion_35_Initialize");

	ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::Instance().InitializeFrameRecording();
	maxVectorUpdate = std::numeric_limits<float>::infinity();
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::FinalizeProcessing(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene,
		ITMRenderState* renderState) {

	//fuse warped live into canonical
	bench::StartTimer("FuseLiveIntoCanonicalSdf");
	sceneReconstructor->FuseLiveIntoCanonicalSdf(canonicalScene, liveScene, targetSdfIndex);
	bench::StopTimer("FuseLiveIntoCanonicalSdf");

	//bench::StartTimer("AddFlowWarpToWarp");
	//sceneMotionTracker->AddFlowWarpToWarp(canonicalScene, true);
	//bench::StopTimer("AddFlowWarpToWarp");

	ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::Instance().FinalizeFrameRecording();

	ProcessSwapping(canonicalScene, renderState);
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::ProcessFrame(const ITMView* view,
                                                                              const ITMTrackingState* trackingState,
                                                                              ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
                                                                              ITMScene<TVoxelLive, TIndex>* liveScene,
                                                                              ITMRenderState* renderState) {


	if (inStepByStepProcessingMode) {
		DIEWITHEXCEPTION_REPORTLOCATION("Cannot track motion for full frame when in step-by-step mode");
	}
	InitializeProcessing(view, trackingState, canonicalScene, liveScene, renderState);
	bench::StartTimer("TrackMotion");
	TrackFrameMotion(canonicalScene, liveScene);
	bench::StopTimer("TrackMotion");
	FinalizeProcessing(canonicalScene, liveScene, renderState);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::UpdateVisibleList(
		const ITMView* view,
		const ITMTrackingState* trackingState,
		ITMScene<TVoxelLive, TIndex>* scene, ITMRenderState* renderState,
		bool resetVisibleList) {
	sceneReconstructor->UpdateVisibleList(scene, view, trackingState, renderState, resetVisibleList);
}

// region =========================================== MOTION TRACKING ==================================================
/**
 * \brief Tracks motion of voxels from canonical frame to live frame.
 * \details The warp field representing motion of voxels in the canonical frame is updated such that the live frame maps
 * as closely as possible back to the canonical using the warp.
 * \param canonicalScene the canonical voxel grid
 * \param liveScene the live voxel grid (typcially obtained by integrating a single depth image into an empty TSDF grid)
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::TrackFrameMotion(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& liveScene) {
	if (analysisFlags.restrictZtrackingForDebugging) {
		std::cout << red << "WARNING: UPDATES IN Z DIRECTION HAVE BEEN DISABLED FOR DEBUGGING"
		                    " PURPOSES. DO NOT EXPECT PROPER RESULTS!" << reset << std::endl;
	}
	PrintOperationStatus("*** Optimizing warp based on difference between canonical and live SDF. ***");
	bench::StartTimer("TrackMotion_3_Optimization");
	for (iteration = 0; SceneMotionOptimizationConditionNotReached(); iteration++) {
		PerformSingleOptimizationStep(canonicalScene, liveScene);
	}
	bench::StopTimer("TrackMotion_3_Optimization");
	PrintOperationStatus("*** Warp optimization finished for current frame. ***");

}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::PerformSingleOptimizationStep(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {

	//ping-pong between the SDF field indices in the live frame (preserves memory locality during traversal)
	sourceSdfIndex = (iteration + 1) % 2;
	targetSdfIndex = iteration % 2;

	ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::Instance().SaveWarpSlices(iteration);
	std::cout << red << "Iteration: " << iteration << reset << std::endl;
	//** warp update gradient computation
	PrintOperationStatus("Calculating warp energy gradient...");
	bench::StartTimer("TrackMotion_31_CalculateWarpUpdate");


	sceneMotionTracker->CalculateWarpGradient(canonicalScene, liveScene, sourceSdfIndex,
	                                          analysisFlags.restrictZtrackingForDebugging);

	bench::StopTimer("TrackMotion_31_CalculateWarpUpdate");

	PrintOperationStatus("Applying Sobolev smoothing to energy gradient...");
	bench::StartTimer("TrackMotion_32_ApplySmoothingToGradient");
	sceneMotionTracker->SmoothWarpGradient(liveScene, canonicalScene, sourceSdfIndex);
	bench::StopTimer("TrackMotion_32_ApplySmoothingToGradient");

	PrintOperationStatus("Applying warp update (based on energy gradient) to the cumulative warp...");
	bench::StartTimer("TrackMotion_33_UpdateWarps");
	maxVectorUpdate = sceneMotionTracker->UpdateWarps(canonicalScene, liveScene, sourceSdfIndex);
	bench::StopTimer("TrackMotion_33_UpdateWarps");

	PrintOperationStatus(
			"Updating live frame SDF by mapping from raw live SDF to new warped SDF based on latest warp...");
	bench::StartTimer("TrackMotion_35_WarpLiveScene");
	sceneReconstructor->WarpScene_WarpUpdates(canonicalScene, liveScene, sourceSdfIndex, targetSdfIndex);
	bench::StopTimer("TrackMotion_35_WarpLiveScene");
	ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::Instance().SaveWarps();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::SceneMotionOptimizationConditionNotReached() {
	return maxVectorUpdate > parameters.maxVectorUpdateThresholdVoxels && iteration < parameters.maxIterationCount;
}

//endregion ============================================================================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::ProcessSwapping(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMRenderState* renderState) {
	if (swappingEngine != nullptr) {
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
// region ======================================= STEP-BY-STEP MODE ====================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void
ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::BeginProcessingFrameInStepByStepMode(
		const ITMView* view, const ITMTrackingState* trackingState,  ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
		ITMScene<TVoxelLive, TIndex>* liveScene, ITMRenderState* renderState) {
	if (inStepByStepProcessingMode) {
		DIEWITHEXCEPTION_REPORTLOCATION("Already in step-by-step tracking mode, cannot restart.");
	}
	inStepByStepProcessingMode = true;
	InitializeProcessing(view, trackingState, canonicalScene, liveScene, renderState);
	iteration = 0;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::TakeNextStepInStepByStepMode(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
		ITMScene<TVoxelLive, TIndex>*& liveScene, ITMRenderState* renderState) {
	if (!inStepByStepProcessingMode) {
		DIEWITHEXCEPTION_REPORTLOCATION("Step-by-step mode not initialized.");
	}
	if (SceneMotionOptimizationConditionNotReached()) {
		PerformSingleOptimizationStep(canonicalScene, liveScene);
		iteration++;
		return true;
	} else {
		FinalizeProcessing(canonicalScene, liveScene, renderState);
		inStepByStepProcessingMode = false;
		return false;
	}
}

// endregion

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::PrintSettings() {
	std::cout << bright_cyan << "*** ITMDenseDynamicMapper Settings: ***" << reset << std::endl;
	std::cout << "Max iteration count: " << this->parameters.maxIterationCount << std::endl;
	std::cout << "Warp vector update threshold: " << this->parameters.maxVectorUpdateThresholdMeters << " m, ";
	std::cout << this->parameters.maxVectorUpdateThresholdVoxels << " voxels" << std::endl;
	std::cout << bright_cyan << "*** *********************************** ***" << reset << std::endl;
}
