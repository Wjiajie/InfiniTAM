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
#include <limits>
#include <iomanip>
#include <memory>


//opencv
#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>

//local
#include "ITMSceneMotionTracker.h"
#include "../../Objects/Scene/ITMSceneManipulation.h"
#include "../../Utils/FileIO/ITMSceneLogger.h"
#include "../../Utils/ITMPrintHelpers.h"
#include "../../Utils/ITMSceneSliceRasterizer.h"
#include "../../Utils/ITMBenchmarkUtils.h"
#include "../../Utils/ITMSceneStatisticsCalculator.h"

//boost
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;
namespace bench = ITMLib::Bench;

using namespace ITMLib;

//region =========================================== CONSTRUCTORS / DESTRUCTORS ========================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::ITMSceneMotionTracker(const ITMLibSettings* settings)
		:
		maxVectorUpdateThresholdVoxels(
				settings->sceneTrackingOptimizationVectorUpdateThresholdMeters / settings->sceneParams.voxelSize),
		sceneLogger(nullptr),
		baseOutputDirectory(settings->outputPath),
		hasFocusCoordinates(settings->FocusCoordinatesAreSpecified()),
		focusCoordinates(settings->GetFocusCoordinates()),
		parameters{settings->sceneTrackingMaxOptimizationIterationCount,
		            settings->sceneTrackingOptimizationVectorUpdateThresholdMeters,
		            settings->sceneTrackingGradientDescentLearningRate,
		            settings->sceneTrackingRigidityEnforcementFactor,
		            settings->sceneTrackingWeightDataTerm,
		            settings->sceneTrackingWeightSmoothingTerm,
		            settings->sceneTrackingWeightLevelSetTerm,
		            settings->sceneTrackingLevelSetTermEpsilon,
		            settings->sceneParams.voxelSize / settings->sceneParams.mu
					},
		rasterizer(focusCoordinates){}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::~ITMSceneMotionTracker() {
	delete sceneLogger;
}

//endregion

// region =========================================== MOTION TRACKING ==================================================

//_DEBUG (?, verbose param?)
inline static void PrintOperationStatus(const char* status) {
	std::cout << bright_cyan << status << reset << std::endl;
}
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::InitializeTracking(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& liveScene,
		bool recordWarpUpdates) {
//PrintLiveSceneStatistics(sourceLiveScene, "raw live scene");//_DEBUG

	if(restrictZtrackingForDebugging){
		std::cout << red << "WARNING: UPDATES IN Z DIRECTION HAVE BEEN DISABLED FOR DEBUGGING"
					  " PURPOSES. DO NOT EXPECT PROPER RESULTS!" << reset << std::endl;
	}

	//** prepare canonical for new frame
	PrintOperationStatus("Allocating canonical blocks based on live frame...");
	bench::StartTimer("TrackMotion_0_AllocateNewCanonicalHashBlocks");
	AllocateNewCanonicalHashBlocks(canonicalScene, liveScene);
	bench::StopTimer("TrackMotion_0_AllocateNewCanonicalHashBlocks");

	if (trackedFrameCount <= startTrackingAfterFrame && !simpleSceneExperimentModeEnabled) return; //don't need to actually do tracking at first frame.

	ClearOutWarps(canonicalScene);

	//** initialize live frame
	PrintOperationStatus(
			"Initializing live frame by mapping the raw live scene to a blank scene using canonical voxel warp field...");

	// endregion

	// region ================================== DEBUG 2D VISUALIZATION ================================================


	if (hasFocusCoordinates) {
		if (rasterizeCanonical) {
			rasterizer.RenderCanonicalSceneSlices_AllDirections(canonicalScene);
		}
		if (rasterizeLive) {
			rasterizer.RenderLiveSceneSlices_AllDirections(liveScene);
		}

		if (rasterizeUpdates && rasterizationFrame == trackedFrameCount) {
			std::cout << "STARTING UPDATE RASTERIZATION" << std::endl;
			InitializeUpdate2DImageLogging(canonicalScene, liveScene, blank, liveImgTemplate, rasterizer);
		}
	}
	// endregion

	// region ========================= INTIALIZE RECORDING ============================================================
	bench::StartTimer("TrackMotion_2_RecordingEnergy");
	std::string currentFrameOutputPath = GenerateCurrentFrameOutputPath();
	const std::string energyStatFilePath = currentFrameOutputPath + "/energy.txt";
	energy_stat_file = std::ofstream(energyStatFilePath.c_str(), std::ios_base::out);
	energy_stat_file << "data" << "," << "level_set" << "," << "smoothness" << ","
	                 << "killing" << "," << "total" << std::endl;
	bench::StopTimer("TrackMotion_2_RecordingEnergy");

	if (recordWarpUpdates) {
		sceneLogger = new ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>(canonicalScene, liveScene,
		                                                                      currentFrameOutputPath);
		sceneLogger->SaveScenesCompact();
		sceneLogger->StartSavingWarpState(trackedFrameCount);
		if (hasFocusCoordinates) {
			sceneLogger->ClearHighlights();
		}
	}
	// endregion =======================================================================================================
	maxVectorUpdate = std::numeric_limits<float>::infinity();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::PerformSingleOptimizationStep(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& liveScene,
		bool recordWarpUpdates) {

// region ================================== DEBUG 2D VISUALIZATION FOR UPDATES ================================
	if (rasterizeUpdates && rasterizationFrame == trackedFrameCount) {
		LogWarpUpdateAs2DImage(canonicalScene, liveScene, blank, liveImgTemplate, rasterizer);
	}
	// endregion
	std::cout << red << "Iteration: " << iteration << reset << std::endl;
	PrintOperationStatus("Marking boundary voxels...");
	bench::StartTimer("TrackMotion_30_MarkBoundaryVoxels");
	MarkBoundaryVoxels(liveScene);//_DEBUG
	bench::StopTimer("TrackMotion_30_MarkBoundaryVoxels");

	//** warp update gradient computation
	PrintOperationStatus("Calculating warp energy gradient...");
	bench::StartTimer("TrackMotion_31_CalculateWarpUpdate");
	CalculateWarpGradient(canonicalScene, liveScene);
	bench::StopTimer("TrackMotion_31_CalculateWarpUpdate");

	PrintOperationStatus("Applying Sobolev smoothing to energy gradient...");
	bench::StartTimer("TrackMotion_32_ApplySmoothingToGradient");
	ApplySmoothingToGradient(canonicalScene, liveScene);
	bench::StopTimer("TrackMotion_32_ApplySmoothingToGradient");

	PrintOperationStatus("Applying warp update (based on energy gradient) to the cumulative warp...");
	bench::StartTimer("TrackMotion_33_ApplyWarpUpdateToWarp");
	maxVectorUpdate = ApplyWarpUpdateToWarp(canonicalScene, liveScene);
	bench::StopTimer("TrackMotion_33_ApplyWarpUpdateToWarp");

	PrintOperationStatus(
			"Updating live frame SDF by mapping from old live SDF to new live SDF based on latest warp update...");
	bench::StartTimer("TrackMotion_35_ApplyWarpUpdateToLive");
	ApplyWarpUpdateToLive(canonicalScene, liveScene);
	bench::StopTimer("TrackMotion_35_ApplyWarpUpdateToLive");

	if (recordWarpUpdates) {
		sceneLogger->SaveCurrentWarpState();
	}
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::FinalizeTracking(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& liveScene,
		bool recordWarpUpdates) {
	// region ================================== WARP UPDATE RECORDING (AFTER OPTIMIZATION) ============================
	if (recordWarpUpdates) {
		sceneLogger->StopSavingWarpState();
		if (hasFocusCoordinates) {
			Vector3i sliceMinPoint(focusCoordinates[0] - FOCUS_SLICE_RADIUS,
			                       focusCoordinates[1] - FOCUS_SLICE_RADIUS,
			                       focusCoordinates[2] - FOCUS_SLICE_RADIUS);
			Vector3i sliceMaxPoint(focusCoordinates[0] + FOCUS_SLICE_RADIUS,
			                       focusCoordinates[1] + FOCUS_SLICE_RADIUS,
			                       focusCoordinates[2] + FOCUS_SLICE_RADIUS);
			std::cout << "Making slice around voxel " << green << focusCoordinates << reset << " with l_0 radius of "
			          << FOCUS_SLICE_RADIUS << "...";
			std::string sliceId;
			sceneLogger->MakeSlice(sliceMinPoint, sliceMaxPoint, trackedFrameCount, sliceId);
			std::cout << "Slice finished." << std::endl;
			sceneLogger->SwitchActiveScene(sliceId);
		}
		delete sceneLogger;
	}
	// endregion =======================================================================================================
	energy_stat_file.close();
	trackedFrameCount++;
}

/**
 * \brief Tracks motion of voxels from canonical frame to live frame.
 * \details The warp field representing motion of voxels in the canonical frame is updated such that the live frame maps
 * as closely as possible back to the canonical using the warp.
 * \tparam TVoxelCanonical type of canonical voxels
 * \tparam TVoxelLive type of live voxels
 * \tparam TIndex type of voxel index used
 * \param canonicalScene the canonical voxel grid
 * \param liveScene the live voxel grid (typcially obtained by integrating a single depth image into an empty TSDF grid)
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::TrackMotion(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& liveScene,
		bool recordWarpUpdates) {

	if (inStepByStepProcessingMode) {
		DIEWITHEXCEPTION_REPORTLOCATION("Cannot track motion for full frame when in step-by-step mode");
	}
	InitializeTracking(canonicalScene, liveScene, recordWarpUpdates);
	if(trackedFrameCount <= startTrackingAfterFrame && !simpleSceneExperimentModeEnabled){
		// no tracking nescessary for first frame
		trackedFrameCount++;
		return;
	}


	// region ================================== WARP UPDATE RECORDING (BEFORE OPTIMIZATION) ===========================
	PrintOperationStatus("*** Optimizing warp based on difference between canonical and live SDF. ***");

	bench::StartTimer("TrackMotion_3_Optimization");
	for (iteration = 0; maxVectorUpdate > maxVectorUpdateThresholdVoxels && iteration < parameters.maxIterationCount;
	     iteration++) {
		PerformSingleOptimizationStep(canonicalScene,liveScene,recordWarpUpdates);
	}
	bench::StopTimer("TrackMotion_3_Optimization");

	PrintOperationStatus("*** Warp optimization finished for current frame. ***");

	FinalizeTracking(canonicalScene,liveScene,recordWarpUpdates);

}
//endregion ============================================================================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::PrintLiveSceneStatistics(
		ITMScene<TVoxelLive, TIndex>* scene,
		const char* desc){
	ITMSceneStatisticsCalculator<TVoxelLive, TIndex> liveCalculator;
	std::cout << std::setprecision(10) << "   Count of allocated blocks in " << desc << ": "
	          << liveCalculator.ComputeAllocatedHashBlockCount(scene) << std::endl;
	std::cout << "   Sum of non-truncated SDF magnitudes in " << desc << ": "
	          << liveCalculator.ComputeNonTruncatedVoxelAbsSdfSum(scene) << std::endl;
	std::cout << "   Count of non-truncated voxels in " << desc << ": "
	          << liveCalculator.ComputeNonTruncatedVoxelCount(scene) << std::endl;

};


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::GenerateCurrentFrameOutputPath() const {
	fs::path path(baseOutputDirectory + "/Frame_" + std::to_string(trackedFrameCount));
	if (!fs::exists(path)) {
		fs::create_directories(path);
	}
	return path.string();
}

// region ===================================== 2D IMAGE GENERATION FOR VISUAL DEBUGGING ===============================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void
ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::InitializeUpdate2DImageLogging(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* sourceLiveScene,
		cv::Mat& blank, cv::Mat& liveImgTemplate,
		ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>& rasterizer) {
	std::cout << "Desired warp update (voxels) below " << maxVectorUpdateThresholdVoxels << std::endl;
	cv::Mat canonicalImg = rasterizer.DrawCanonicalSceneImageAroundPoint(canonicalScene) * 255.0f;
	cv::Mat canonicalImgOut;
	canonicalImg.convertTo(canonicalImgOut, CV_8UC1);
	cv::cvtColor(canonicalImgOut, canonicalImgOut, cv::COLOR_GRAY2BGR);
	cv::imwrite(ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::iterationFramesFolder
	            + "canonical.png", canonicalImgOut);
	cv::Mat liveImg = rasterizer.DrawLiveSceneImageAroundPoint(sourceLiveScene, 0) * 255.0f;
	cv::Mat liveImgOut;
	liveImg.convertTo(liveImgTemplate, CV_8UC1);
	cv::cvtColor(liveImgTemplate, liveImgOut, cv::COLOR_GRAY2BGR);
	cv::imwrite(ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::iterationFramesFolder + "live.png",
	            liveImgOut);
	//TODO: this is kinda backwards. Just build this in the constructor using constants from rasterizer for size. -Greg (GitHub: Algomorph)
	blank = cv::Mat::zeros(liveImg.rows, liveImg.cols, CV_8UC1);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void
ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::LogWarpUpdateAs2DImage(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene,
		const cv::Mat& blank, const cv::Mat& liveImgTemplate,
		ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>& rasterizer) {
	cv::Mat warpImg = rasterizer.DrawWarpedSceneImageAroundPoint(canonicalScene) * 255.0f;
	cv::Mat warpImgChannel, warpImgOut, mask, liveImgChannel, markChannel;
	blank.copyTo(markChannel);
	rasterizer.MarkWarpedSceneImageAroundFocusPoint(canonicalScene, markChannel);
	liveImgChannel = cv::Mat::zeros(warpImg.rows, warpImg.cols, CV_8UC1);
	warpImg.convertTo(warpImgChannel, CV_8UC1);
	cv::threshold(warpImgChannel, mask, 1.0, 1.0, cv::THRESH_BINARY_INV);
	liveImgTemplate.copyTo(liveImgChannel, mask);

	cv::Mat channels[3] = {liveImgTemplate, warpImgChannel, markChannel};

	cv::merge(channels, 3, warpImgOut);
	std::stringstream numStringStream;
	numStringStream << std::setw(3) << std::setfill('0') << iteration;
	std::string image_name =
			ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::iterationFramesFolder + "warp" +
			numStringStream.str() + ".png";
	cv::imwrite(image_name, warpImgOut);
	cv::Mat liveImg = rasterizer.DrawLiveSceneImageAroundPoint(liveScene, iteration % 2) * 255.0f;
	cv::Mat liveImgOut;
	liveImg.convertTo(liveImgOut, CV_8UC1);
	cv::cvtColor(liveImgOut, liveImgOut, cv::COLOR_GRAY2BGR);
	cv::imwrite(ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::liveIterationFramesFolder + "live " +
	            numStringStream.str() + ".png", liveImgOut);
}
// endregion ===========================================================================================================

// region ======================================= STEP-BY-STEP MODE ====================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::SetUpStepByStepTracking(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& sourceLiveScene) {
	if (inStepByStepProcessingMode) {
		DIEWITHEXCEPTION_REPORTLOCATION("Already in step-by-step tracking mode, cannot restart.");
	}
	inStepByStepProcessingMode = true;
	InitializeTracking(canonicalScene,sourceLiveScene,false);

	PrintOperationStatus("*** Optimizing warp based on difference between canonical and live SDF. ***");
	iteration = 0;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::UpdateTrackingSingleStep(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& sourceLiveScene) {
	if ((trackedFrameCount <= startTrackingAfterFrame && !simpleSceneExperimentModeEnabled)
	    || iteration > parameters.maxIterationCount || maxVectorUpdate < maxVectorUpdateThresholdVoxels) {
		PrintOperationStatus("*** Warp optimization finished for current frame. ***");
		if(trackedFrameCount > startTrackingAfterFrame || simpleSceneExperimentModeEnabled){
			FinalizeTracking(canonicalScene,sourceLiveScene,false);
		}
		inStepByStepProcessingMode = false;
		return false;
	}
	if (!inStepByStepProcessingMode) {
		DIEWITHEXCEPTION_REPORTLOCATION("Attempted to make single optimization step while not in step-by-step mode!");
	}

	PerformSingleOptimizationStep(canonicalScene,sourceLiveScene, false);
	iteration++;
	return true;
}

// endregion


