//  ================================================================
//  Created by Gregory Kramida on 5/25/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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
#include "ITMDynamicFusionLogger.h"
#include "../Analytics/ITMBenchmarkUtils.h"
#include <opencv2/imgproc.hpp>

using namespace ITMLib;
namespace bench = ITMLib::Bench;

template<typename TVoxelCanonical, typename TVoxelLive, typename TVoxelIndex>
ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TVoxelIndex>::ITMDynamicFusionLogger() : focusSliceRadius(3) {}

// region ===================================== RECORDING ==============================================================


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::InitializeRecording(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& liveScene,
		bool saveLiveScene2DSlicesAsImages, bool saveCanonicalScene2DSlicesAsImages, bool recordWarp2DSlices,
		bool hasFocusCoordinates, Vector3i focusCoordinates, bool recordWarps, std::string outputDirectory) {

	this->canonicalScene = canonicalScene;
	this->liveScene = liveScene;

	//set flags
	this->recordWarp2DSlices = recordWarp2DSlices;
	this->hasFocusCoordinates = hasFocusCoordinates;
	this->focusCoordinates = focusCoordinates;
	this->recordWarps = recordWarps;
	this->outputDirectory = outputDirectory;

	// region ================================== 2D SLICES RECORDING ===================================================
	if (hasFocusCoordinates) {
		if (recordWarp2DSlices || saveCanonicalScene2DSlicesAsImages || saveLiveScene2DSlicesAsImages) {
			rasterizer = new ITMScene2DSliceLogger<TVoxelCanonical, TVoxelLive, TIndex>(focusCoordinates,
			                                                                            outputDirectory, 100, 16.0);
		}
		if (saveCanonicalScene2DSlicesAsImages) {
			rasterizer->SaveLiveSceneSlicesAs2DImages_AllDirections(canonicalScene);
		}
		if (saveLiveScene2DSlicesAsImages) {
			rasterizer->SaveLiveSceneSlicesAs2DImages_AllDirections(liveScene);
		}

		if (recordWarp2DSlices) {
			std::cout << "STARTING UPDATE RASTERIZATION" << std::endl;
			InitializeWarp2DSliceRecording(canonicalScene, liveScene);
		}
	}
	// endregion

	// region ========================= INTIALIZE WARP RECORDING =======================================================
	bench::StartTimer("TrackMotion_2_RecordingEnergy");

	const std::string energyStatFilePath = outputDirectory + "/energy.txt";
	energyStatisticsFile = std::ofstream(energyStatFilePath.c_str(), std::ios_base::out);
	energyStatisticsFile << "data" << "," << "level_set" << "," << "smoothness" << ","
	                     << "killing" << "," << "total" << std::endl;
	bench::StopTimer("TrackMotion_2_RecordingEnergy");

	if (recordWarps) {
		sceneLogger = new ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>(
				canonicalScene, liveScene, outputDirectory);
		sceneLogger->SaveScenesCompact();
		sceneLogger->StartSavingWarpState();
		if (hasFocusCoordinates) {
			sceneLogger->ClearHighlights();
		}
	}

	// endregion =======================================================================================================

}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::InitializeWarp2DSliceRecording(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* sourceLiveScene) {

	cv::Mat canonicalImg = rasterizer->DrawCanonicalSceneImageAroundPoint(canonicalScene) * 255.0f;
	cv::Mat canonicalImgOut;
	canonicalImg.convertTo(canonicalImgOut, CV_8UC1);
	cv::cvtColor(canonicalImgOut, canonicalImgOut, cv::COLOR_GRAY2BGR);
	cv::imwrite(ITMScene2DSliceLogger<TVoxelCanonical, TVoxelLive, TIndex>::iterationFramesFolderName
	            + "canonical.png", canonicalImgOut);
	cv::Mat liveImg = rasterizer->DrawLiveSceneImageAroundPoint(sourceLiveScene, 0) * 255.0f;
	cv::Mat liveImgOut;
	liveImg.convertTo(liveImgTemplate, CV_8UC1);
	cv::cvtColor(liveImgTemplate, liveImgOut, cv::COLOR_GRAY2BGR);
	cv::imwrite(ITMScene2DSliceLogger<TVoxelCanonical, TVoxelLive, TIndex>::iterationFramesFolderName + "live.png",
	            liveImgOut);
	//TODO: this is kinda backwards. Just build this in the constructor using constants from rasterizer for size. -Greg (GitHub: Algomorph)
	blank = cv::Mat::zeros(liveImg.rows, liveImg.cols, CV_8UC1);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void
ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveWarp2DSlice(int iteration) {
	if (hasFocusCoordinates && recordWarps) {
		cv::Mat warpImg = rasterizer->DrawWarpedSceneImageAroundPoint(canonicalScene) * 255.0f;
		cv::Mat warpImgChannel, warpImgOut, mask, liveImgChannel, markChannel;
		blank.copyTo(markChannel);
		rasterizer->MarkWarpedSceneImageAroundFocusPoint(canonicalScene, markChannel);
		liveImgChannel = cv::Mat::zeros(warpImg.rows, warpImg.cols, CV_8UC1);

		warpImg.convertTo(warpImgChannel, CV_8UC1);
		cv::threshold(warpImgChannel, mask, 1.0, 1.0, cv::THRESH_BINARY_INV);
		liveImgTemplate.copyTo(liveImgChannel, mask);

		cv::Mat channels[3] = {liveImgTemplate, warpImgChannel, markChannel};

		cv::merge(channels, 3, warpImgOut);
		std::stringstream numStringStream;
		numStringStream << std::setw(3) << std::setfill('0') << iteration;
		std::string image_name =
				ITMScene2DSliceLogger<TVoxelCanonical, TVoxelLive, TIndex>::iterationFramesFolderName + "warp" +
				numStringStream.str() + ".png";
		cv::imwrite(image_name, warpImgOut);
		cv::Mat liveImg = rasterizer->DrawLiveSceneImageAroundPoint(liveScene, iteration % 2) * 255.0f;
		cv::Mat liveImgOut;
		liveImg.convertTo(liveImgOut, CV_8UC1);
		cv::cvtColor(liveImgOut, liveImgOut, cv::COLOR_GRAY2BGR);
		cv::imwrite(
				ITMScene2DSliceLogger<TVoxelCanonical, TVoxelLive, TIndex>::liveIterationFramesFolderName + "live " +
				numStringStream.str() + ".png", liveImgOut);
	}
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::FinalizeRecording(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& liveScene) {

	if (recordWarps) {

		sceneLogger->StopSavingWarpState();
		Vector3i focusCoordinates = focusCoordinates;
		int focusSliceRadius = focusSliceRadius;
		if (hasFocusCoordinates) {
			Vector3i sliceMinPoint(focusCoordinates[0] - focusSliceRadius,
			                       focusCoordinates[1] - focusSliceRadius,
			                       focusCoordinates[2] - focusSliceRadius);
			Vector3i sliceMaxPoint(focusCoordinates[0] + focusSliceRadius,
			                       focusCoordinates[1] + focusSliceRadius,
			                       focusCoordinates[2] + focusSliceRadius);

			std::cout << "Making slice around voxel " << green << focusCoordinates << reset << " with l_0 radius of "
			          << focusSliceRadius << "...";
			std::string sliceId;
			sceneLogger->MakeSlice(sliceMinPoint, sliceMaxPoint, sliceId);
			std::cout << "Slice finished." << std::endl;
			sceneLogger->SwitchActiveScene(sliceId);
		}
		delete sceneLogger;
		sceneLogger = nullptr;
	}

	delete rasterizer;
	rasterizer = nullptr;
	energyStatisticsFile.close();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveWarps() {
	if (recordWarps) {
		this->sceneLogger->SaveCurrentWarpState();
	}
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::RecordStatistics(double totalDataEnergy,
                                                                                   double totalLevelSetEnergy,
                                                                                   double totalKillingEnergy,
                                                                                   double totalSmoothnessEnergy,
                                                                                   double totalEnergy) {
	energyStatisticsFile << totalDataEnergy << ", " << totalLevelSetEnergy << ", " << totalKillingEnergy << ", "
	                     << totalSmoothnessEnergy << ", " << totalEnergy << std::endl;

}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::IsRecordingWarp2DSlices() {
	return this->recordWarp2DSlices;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::IsRecordingWarps() {
	return this->recordWarps;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::LogHighlight(int hash, int locId,
                                                                               ITMHighlightIterationInfo info) {
	sceneLogger->LogHighlight(hash, locId, 0, info);
};


// endregion ===========================================================================================================
