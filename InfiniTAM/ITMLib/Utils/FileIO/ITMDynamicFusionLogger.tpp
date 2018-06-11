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

//stdlib
#include "iomanip"

//OpenCV
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

//local
#include "ITMDynamicFusionLogger.h"
#include "../Analytics/ITMBenchmarkUtils.h"
#include "../ITMPrintHelpers.h"


using namespace ITMLib;
namespace bench = ITMLib::Bench;

template<typename TVoxelCanonical, typename TVoxelLive, typename TVoxelIndex>
ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TVoxelIndex>::ITMDynamicFusionLogger() : focusSliceRadius(3) {}

// region ===================================== RECORDING ==============================================================


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::InitializeRecording(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& liveScene,
		std::string outputDirectory, bool hasFocusCoordinates, Vector3i focusCoordinates,
		bool saveLiveScene2DSlicesAsImages, bool saveCanonicalScene2DSlicesAsImages, bool recordWarps,
		bool recordScene1DSlicesWithUpdates, bool recordScene2DSlicesWithUpdates) {

	//TODO: make all visualizer/logger classes re-usable, i.e. just change the path & build them in the constructor (don't use pointers) -Greg (GitHub:Algomorph)

	this->canonicalScene = canonicalScene;
	this->liveScene = liveScene;

	this->hasFocusCoordinates = hasFocusCoordinates;
	this->focusCoordinates = focusCoordinates;
	this->outputDirectory = outputDirectory;

	// region ================================== 2D SLICES RECORDING ===================================================
	if (hasFocusCoordinates) {
		if (recordScene2DSlicesWithUpdates || saveCanonicalScene2DSlicesAsImages || saveLiveScene2DSlicesAsImages) {
			scene2DSliceVisualizer = new ITMScene2DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>(
					focusCoordinates, outputDirectory, 100, 16.0, PLANE_XY);
		}
		if (saveCanonicalScene2DSlicesAsImages) {
			scene2DSliceVisualizer->SaveLiveSceneSlicesAs2DImages_AllDirections(canonicalScene);
		}
		if (saveLiveScene2DSlicesAsImages) {
			scene2DSliceVisualizer->SaveLiveSceneSlicesAs2DImages_AllDirections(liveScene);
		}
		if (recordScene1DSlicesWithUpdates) {
			recordingScene1DSlicesWithUpdates = true;
			scene1DSliceVisualizer =
					new ITMScene1DSliceVisualizer(focusCoordinates, AXIS_X, 16,
					                              this->outputDirectory + "/scene_1D_slices");
			scene1DSliceVisualizer->Plot1DSceneSlice(canonicalScene, Vector4i(97, 181, 193, 255), 3.0);
			scene1DSliceVisualizer->Plot1DIndexedSceneSlice(liveScene, Vector4i(183, 115, 46, 255), 3.0,0);
		} else {
			recordingScene1DSlicesWithUpdates = false;
		}
		if (recordScene2DSlicesWithUpdates) {
			recordingScene2DSlicesWithUpdates = true;
			std::cout << yellow << "Recording 2D scene slices with warps & warped live scene progression as images in "
			          << scene2DSliceVisualizer->GetOutputDirectoryForWarps() << " and "
			          << scene2DSliceVisualizer->GetOutputDirectoryForWarpedLiveScenes() << " respectively..."
			          << reset << std::endl;
			InitializeWarp2DSliceRecording(canonicalScene, liveScene);
		} else {
			if(saveCanonicalScene2DSlicesAsImages || saveLiveScene2DSlicesAsImages){
				delete scene2DSliceVisualizer;
				scene2DSliceVisualizer = nullptr;
			}
			recordingScene2DSlicesWithUpdates = false;
		}

	}
	// endregion

	// region ========================= INITIALIZE WARP RECORDING ======================================================
	bench::StartTimer("TrackMotion_2_RecordingEnergy");

	const std::string energyStatFilePath = outputDirectory + "/energy.txt";
	energyStatisticsFile = std::ofstream(energyStatFilePath.c_str(), std::ios_base::out);
	energyStatisticsFile << "data" << "," << "level_set" << "," << "smoothness" << ","
	                     << "killing" << "," << "total" << std::endl;
	bench::StopTimer("TrackMotion_2_RecordingEnergy");

	this->recordingWarps = recordWarps;
	if (recordWarps) {
		scene3DLogger = new ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>(
				canonicalScene, liveScene, outputDirectory);
		scene3DLogger->SaveScenesCompact();
		scene3DLogger->StartSavingWarpState();
		if (hasFocusCoordinates) {
			scene3DLogger->ClearHighlights();
		}
	}

	// endregion =======================================================================================================

}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::InitializeWarp2DSliceRecording(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* sourceLiveScene) {

	cv::Mat canonicalImg = scene2DSliceVisualizer->DrawCanonicalSceneImageAroundPoint(canonicalScene) * 255.0f;
	cv::Mat canonicalImgOut;
	canonicalImg.convertTo(canonicalImgOut, CV_8UC1);
	cv::cvtColor(canonicalImgOut, canonicalImgOut, cv::COLOR_GRAY2BGR);
	scene2DSliceVisualizer->MakeOrClearOutputDirectories();
	cv::imwrite(scene2DSliceVisualizer->GetOutputDirectoryForWarps() + "/canonical.png", canonicalImgOut);
	cv::Mat liveImg = scene2DSliceVisualizer->DrawLiveSceneImageAroundPoint(sourceLiveScene, 0) * 255.0f;
	cv::Mat liveImgOut;
	liveImg.convertTo(liveImgTemplate, CV_8UC1);
	cv::cvtColor(liveImgTemplate, liveImgOut, cv::COLOR_GRAY2BGR);
	cv::imwrite(scene2DSliceVisualizer->GetOutputDirectoryForWarps() + "/live.png", liveImgOut);
	//TODO: this is kinda backwards. Just build this in the constructor using constants from rasterizer for size. -Greg (GitHub: Algomorph)
	blank = cv::Mat::zeros(liveImg.rows, liveImg.cols, CV_8UC1);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void
ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveWarpSlices(int iteration) {
	if (hasFocusCoordinates && recordingScene2DSlicesWithUpdates) {
		cv::Mat warpImg = scene2DSliceVisualizer->DrawWarpedSceneImageAroundPoint(canonicalScene) * 255.0f;
		cv::Mat warpImgChannel, warpImgOut, mask, liveImgChannel, markChannel;
		blank.copyTo(markChannel);
		scene2DSliceVisualizer->MarkWarpedSceneImageAroundFocusPoint(canonicalScene, markChannel);
		liveImgChannel = cv::Mat::zeros(warpImg.rows, warpImg.cols, CV_8UC1);

		warpImg.convertTo(warpImgChannel, CV_8UC1);
		cv::threshold(warpImgChannel, mask, 1.0, 1.0, cv::THRESH_BINARY_INV);
		liveImgTemplate.copyTo(liveImgChannel, mask);

		cv::Mat channels[3] = {liveImgTemplate, warpImgChannel, markChannel};

		cv::merge(channels, 3, warpImgOut);
		std::stringstream numStringStream;
		numStringStream << std::setw(3) << std::setfill('0') << iteration;
		std::string image_name =
				scene2DSliceVisualizer->GetOutputDirectoryForWarps() + "/warp" + numStringStream.str() + ".png";
		cv::imwrite(image_name, warpImgOut);
		cv::Mat liveImg = scene2DSliceVisualizer->DrawLiveSceneImageAroundPoint(liveScene, 1) * 255.0f;
		cv::Mat liveImgOut;
		liveImg.convertTo(liveImgOut, CV_8UC1);
		cv::cvtColor(liveImgOut, liveImgOut, cv::COLOR_GRAY2BGR);
		cv::imwrite(scene2DSliceVisualizer->GetOutputDirectoryForWarpedLiveScenes() + "/live " + numStringStream.str() +
		            ".png", liveImgOut);
	}
	if (hasFocusCoordinates && recordingScene1DSlicesWithUpdates) {
		scene1DSliceVisualizer->Plot1DIndexedSceneSlice(liveScene, Vector4i(0, 0, 0, 255), 1.0, 1);
		scene1DSliceVisualizer->Draw1DWarpUpdateVector(canonicalScene,Vector4i(255,0,0,255));
	}
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::FinalizeRecording(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>*& liveScene) {

	if (recordingWarps) {

		scene3DLogger->StopSavingWarpState();
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
			scene3DLogger->MakeSlice(sliceMinPoint, sliceMaxPoint, sliceId);
			std::cout << "Slice finished." << std::endl;
			scene3DLogger->SwitchActiveScene(sliceId);
		}
		delete scene3DLogger;
		scene3DLogger = nullptr;
	}
	if (recordingScene2DSlicesWithUpdates) {
		delete scene2DSliceVisualizer;
		scene2DSliceVisualizer = nullptr;
	}

	if (recordingScene1DSlicesWithUpdates) {
		delete scene1DSliceVisualizer;
		scene1DSliceVisualizer = nullptr;
	}

	energyStatisticsFile.close();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveWarps() {
	if (recordingWarps) {
		this->scene3DLogger->SaveCurrentWarpState();
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
	return this->recordingScene2DSlicesWithUpdates;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::IsRecordingWarps() {
	return this->recordingWarps;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::LogHighlight(int hash, int locId,
                                                                               ITMHighlightIterationInfo info) {
	scene3DLogger->LogHighlight(hash, locId, 0, info);
};


// endregion ===========================================================================================================
