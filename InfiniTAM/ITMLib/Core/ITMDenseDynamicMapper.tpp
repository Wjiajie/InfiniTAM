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
#include "../Engines/Reconstruction/ITMDynamicSceneReconstructionEngineFactory.h"
#include "../Engines/Swapping/ITMSwappingEngineFactory.h"
#include "../SceneMotionTrackers/ITMSceneMotionTrackerFactory.h"
#include "../Objects/Scene/ITMSceneManipulation.h"
#include "../Utils/Analytics/ITMSceneStatisticsCalculator.h"
#include "../Utils/ITMPrintHelpers.h"
#include "../Utils/FileIO/ITMScene2DSliceLogger.h"
#include "../Utils/Analytics/ITMBenchmarkUtils.h"
#include "../Utils/FileIO/ITMSceneLogger.h"
#include "../../Tests/TestUtils.h"

//boost
#include <boost/filesystem.hpp>

//opencv
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>


using namespace ITMLib;

namespace fs = boost::filesystem;
namespace bench = ITMLib::Bench;

// region ========================================= DEBUG PRINTING =====================================================
template<typename TVoxel, typename TIndex>
inline static void PrintSceneStatistics(
		ITMScene<TVoxel, TIndex>* scene,
		std::string description) {
	ITMSceneStatisticsCalculator<TVoxel, TIndex> calculator;
	std::cout << std::setprecision(10) << "   Count of allocated blocks in " << description << ": "
	          << calculator.ComputeAllocatedHashBlockCount(scene) << std::endl;
	std::cout << "   Sum of non-truncated SDF magnitudes in " << description << ": "
	          << calculator.ComputeNonTruncatedVoxelAbsSdfSum(scene) << std::endl;
	std::cout << "   Count of non-truncated voxels in " << description << ": "
	          << calculator.ComputeNonTruncatedVoxelCount(scene) << std::endl;

};

inline static void PrintOperationStatus(const char* status) {
	std::cout << bright_cyan << status << reset << std::endl;
}
// endregion ===========================================================================================================

// region ===================================== CONSTRUCTORS / DESTRUCTORS =============================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::ITMDenseDynamicMapper(const ITMLibSettings* settings) :
		sceneReconstructor(
				ITMDynamicSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxelCanonical, TVoxelLive, TIndex>
						(settings->deviceType)),
		sceneMotionTracker(ITMSceneMotionTrackerFactory::MakeSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>(settings)),
		swappingEngine(settings->swappingMode != ITMLibSettings::SWAPPINGMODE_DISABLED
		               ? ITMSwappingEngineFactory::MakeSwappingEngine<TVoxelCanonical, TIndex>(settings->deviceType)
		               : nullptr),
		swappingMode(settings->swappingMode),
		parameters{settings->sceneTrackingMaxOptimizationIterationCount,
		           settings->sceneTrackingOptimizationVectorUpdateThresholdMeters,
		           settings->sceneTrackingOptimizationVectorUpdateThresholdMeters / settings->sceneParams.voxelSize},
		analysisFlags{settings->restrictZtrackingForDebugging, settings->simpleSceneExperimentModeEnabled,
		              settings->FocusCoordinatesAreSpecified()},
		focusCoordinates(settings->GetFocusCoordinates()) {}

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
	sceneReconstructor->ResetCanonicalScene(scene);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::ResetLiveScene(
		ITMScene<TVoxelLive, TIndex>* live_scene) const {
	sceneReconstructor->ResetLiveScene(live_scene);
}

//BEGIN _DEBUG
template<typename TVoxel, typename TIndex>
static void PrintSceneStats(ITMScene<TVoxel, TIndex>* scene, const char* sceneName) {
	ITMSceneStatisticsCalculator<TVoxel, TIndex> calculatorLive;
	std::cout << "=== Stats for scene '" << sceneName << "' ===" << std::endl;
	std::cout << "    Total voxel count: " << calculatorLive.ComputeAllocatedVoxelCount(scene) << std::endl;
	std::cout << "    NonTruncated voxel count: " << calculatorLive.ComputeNonTruncatedVoxelCount(scene) << std::endl;
	std::cout << "    +1.0 voxel count: " << calculatorLive.ComputeVoxelWithValueCount(scene, 1.0f) << std::endl;
	std::vector<int> allocatedHashes = calculatorLive.GetFilledHashBlockIds(scene);
	std::cout << "    Allocated hash count: " << allocatedHashes.size() << std::endl;
	std::cout << "    NonTruncated SDF sum: " << calculatorLive.ComputeNonTruncatedVoxelAbsSdfSum(scene) << std::endl;
	std::cout << "    Truncated SDF sum: " << calculatorLive.ComputeTruncatedVoxelAbsSdfSum(scene) << std::endl;
};
//END _DEBUG


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::GenerateRawLiveFrame(
		ITMScene<TVoxelLive, TIndex>* liveScene, ITMRenderState* renderState,
		const ITMTrackingState* trackingState,
		const ITMView* view) {
	PrintOperationStatus("Generating raw live frame...");
	bench::StartTimer("GenerateRawLiveFrame");
	// clear out the live-frame SDF
	sceneReconstructor->ResetLiveScene(liveScene);
	sceneReconstructor->AllocateLiveSceneFromDepth(liveScene, view, trackingState, renderState);
	// generate SDF from view images
	sceneReconstructor->IntegrateIntoScene(liveScene, view, trackingState, renderState);
	bench::StopTimer("GenerateRawLiveFrame");
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::ProcessInitialFrame(
		const ITMView* view, const ITMTrackingState* trackingState,
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene,
		ITMRenderState* renderState) {
	GenerateRawLiveFrame(liveScene, renderState, trackingState, view);
	//** prepare canonical for new frame
	PrintOperationStatus("Allocating canonical blocks based on live frame...");
	bench::StartTimer("AllocateCanonicalFromLive");
	sceneReconstructor->AllocateCanonicalFromLive(canonicalScene, liveScene);
	bench::StopTimer("AllocateCanonicalFromLive");
	//** fuse the live into canonical directly
	bench::StartTimer("FuseFrame");
	sceneReconstructor->FuseFrame(canonicalScene, liveScene, 0);
	bench::StopTimer("FuseFrame");
	ProcessSwapping(canonicalScene, renderState);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::InitializeProcessing(
		const ITMView* view, const ITMTrackingState* trackingState,
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene,
		ITMRenderState* renderState, bool recordWarps, bool recordWarp2DSlice, std::string outputDirectory) {

	if (analysisFlags.simpleSceneExperimentModeEnabled) {
		GenerateTestScene01(canonicalScene);
		CopySceneSDFandFlagsWithOffset_CPU(liveScene, canonicalScene, Vector3i(-5, 0, 0));
	} else {
		GenerateRawLiveFrame(liveScene, renderState, trackingState, view);
	}

	sceneMotionTracker->ClearOutWarps(canonicalScene);
	logger.InitializeRecording(canonicalScene, liveScene, false, false, recordWarp2DSlice,
	                           analysisFlags.hasFocusCoordinates, focusCoordinates, recordWarps, outputDirectory);
	maxVectorUpdate = std::numeric_limits<float>::infinity();
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::FinalizeProcessing(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene,
		ITMRenderState* renderState) {

	logger.FinalizeRecording(canonicalScene, liveScene);
	bench::StartTimer("AllocateCanonicalFromLive");
	sceneReconstructor->AllocateCanonicalFromLive(canonicalScene, liveScene);
	bench::StopTimer("AllocateCanonicalFromLive");
	//fuse warped live into canonical
	bench::StartTimer("FuseFrame");
	sceneReconstructor->FuseFrame(canonicalScene, liveScene, iteration % 2);
	bench::StopTimer("FuseFrame");

	ProcessSwapping(canonicalScene, renderState);
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::ProcessFrame(
		const ITMView* view, const ITMTrackingState* trackingState,
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene,
		ITMRenderState* renderState, bool recordWarps, bool recordWarp2DSlice, std::string outputPath) {
	if (inStepByStepProcessingMode) {
		DIEWITHEXCEPTION_REPORTLOCATION("Cannot track motion for full frame when in step-by-step mode");
	}
	InitializeProcessing(view, trackingState, canonicalScene, liveScene, renderState, recordWarps, recordWarp2DSlice,
	                     outputPath);
	bench::StartTimer("TrackMotion");
	TrackFrameMotion(canonicalScene, liveScene, recordWarp2DSlice);
	bench::StopTimer("TrackMotion");
	FinalizeProcessing(canonicalScene, liveScene, renderState);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::UpdateVisibleList(
		const ITMView* view,
		const ITMTrackingState* trackingState,
		ITMScene<TVoxelLive, TIndex>* scene, ITMRenderState* renderState,
		bool resetVisibleList) {
	sceneReconstructor->AllocateLiveSceneFromDepth(scene, view, trackingState, renderState, true, resetVisibleList);
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
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& liveScene,
		bool rasterizeWarpUpdates) {
	if (analysisFlags.restrictZtrackingForDebugging) {
		std::cout << red << "WARNING: UPDATES IN Z DIRECTION HAVE BEEN DISABLED FOR DEBUGGING"
		                    " PURPOSES. DO NOT EXPECT PROPER RESULTS!" << reset << std::endl;
	}
	PrintOperationStatus("*** Optimizing warp based on difference between canonical and live SDF. ***");
	bench::StartTimer("TrackMotion_3_Optimization");
	for (iteration = 0; SceneMotionOptimizationConditionNotReached(); iteration++) {
		PerformSingleOptimizationStep(canonicalScene, liveScene, rasterizeWarpUpdates);
	}
	bench::StopTimer("TrackMotion_3_Optimization");
	PrintOperationStatus("*** Warp optimization finished for current frame. ***");

}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::PerformSingleOptimizationStep(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& liveScene,
		bool recordWarpUpdates) {


	logger.SaveWarp2DSlice(iteration);
	std::cout << red << "Iteration: " << iteration << reset << std::endl;
	bench::StartTimer("AllocateCanonicalFromLive");
	sceneReconstructor->AllocateCanonicalFromLive(canonicalScene, liveScene);
	bench::StopTimer("AllocateCanonicalFromLive");
	//** warp update gradient computation
	PrintOperationStatus("Calculating warp energy gradient...");
	bench::StartTimer("TrackMotion_31_CalculateWarpUpdate");
	sceneMotionTracker->CalculateWarpGradient(canonicalScene, liveScene);
	bench::StopTimer("TrackMotion_31_CalculateWarpUpdate");

	PrintOperationStatus("Applying Sobolev smoothing to energy gradient...");
	bench::StartTimer("TrackMotion_32_ApplySmoothingToGradient");
	sceneMotionTracker->SmoothWarpGradient(canonicalScene, liveScene);
	bench::StopTimer("TrackMotion_32_ApplySmoothingToGradient");

	PrintOperationStatus("Applying warp update (based on energy gradient) to the cumulative warp...");
	bench::StartTimer("TrackMotion_33_ApplyWarpUpdateToWarp");
	maxVectorUpdate = sceneMotionTracker->ApplyWarpUpdateToWarp(canonicalScene, liveScene);
	bench::StopTimer("TrackMotion_33_ApplyWarpUpdateToWarp");

	PrintOperationStatus(
			"Updating live frame SDF by mapping from old live SDF to new live SDF based on latest warp update...");
	bench::StartTimer("TrackMotion_35_ApplyWarpUpdateToLive");
	sceneMotionTracker->ApplyWarpUpdateToLive(canonicalScene, liveScene);
	//ApplyWarpFieldToLive(canonicalScene, liveScene);
	bench::StopTimer("TrackMotion_35_ApplyWarpUpdateToLive");
	logger.SaveWarps();
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
void ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::BeginProcessingFrameInStepByStepMode(
		const ITMView* view, const ITMTrackingState* trackingState,
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene,
		ITMRenderState* renderState, bool recordWarps, bool recordWarp2DSlice,
		std::string outputPath) {
	if (inStepByStepProcessingMode) {
		DIEWITHEXCEPTION_REPORTLOCATION("Already in step-by-step tracking mode, cannot restart.");
	}
	inStepByStepProcessingMode = true;
	InitializeProcessing(view, trackingState, canonicalScene, liveScene, renderState, recordWarps, recordWarp2DSlice,
	                     outputPath);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>::TakeNextStepInStepByStepMode(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
		ITMScene<TVoxelLive, TIndex>*& liveScene, ITMRenderState* renderState) {
	if (!inStepByStepProcessingMode) {
		DIEWITHEXCEPTION_REPORTLOCATION("Step-by-step mode not initialized.");
	}
	if (SceneMotionOptimizationConditionNotReached()) {
		PerformSingleOptimizationStep(canonicalScene, liveScene, false);
		iteration++;
		return true;
	} else {
		FinalizeProcessing(canonicalScene, liveScene, renderState);
		return false;
	}
}

// endregion
