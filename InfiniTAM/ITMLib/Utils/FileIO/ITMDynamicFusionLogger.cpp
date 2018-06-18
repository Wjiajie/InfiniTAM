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

//TODO: create/destroy windowed visualizers (i.e. plotter) when their corresponding setting values are turned on/off, and make them close their corresponding windows -Greg (GitHub:Algomorph)

// region ============================== DEFINE CONSTANTS ==============================================================
const std::string ITMDynamicFusionLogger::iterationFramesFolderName =
		"bucket_interest_region_2D_iteration_slices";

const std::string ITMDynamicFusionLogger::liveIterationFramesFolderName =
		"bucket_interest_region_live_slices";

const std::string ITMDynamicFusionLogger::canonicalSceneRasterizedFolderName =
		"canonical_rasterized";

const std::string ITMDynamicFusionLogger::liveSceneRasterizedFolderName =
		"live_rasterized";
// endregion ================================== END CONSTANT DEFINITIONS ===============================================
// region ======================================= SETTERS ==============================================================

void ITMDynamicFusionLogger::SetScenes(
		ITMScene<ITMVoxelCanonical, ITMVoxelIndex>* canonicalScene, ITMScene<ITMVoxelLive, ITMVoxelIndex>* liveScene) {
	this->canonicalScene = canonicalScene;
	this->liveScene = liveScene;
	this->scene3DLogger = new ITMSceneLogger<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>(
			canonicalScene, liveScene, outputDirectory);
}


void ITMDynamicFusionLogger::SetOutputDirectory(std::string outputDirectory) {
	this->scene3DLogger->SetPath(outputDirectory);
	this->outputDirectory = outputDirectory;
};


void ITMDynamicFusionLogger::SetFocusCoordinates(Vector3i focusCoordinates) {
	hasFocusCoordinates = true;
	this->focusCoordinates = focusCoordinates;
}


void ITMDynamicFusionLogger::SetPlaneFor2DSlices(Plane plane) {
	this->planeFor2DSlices = plane;
}


void ITMDynamicFusionLogger::TurnRecordingLiveSceneAs2DSlicesOn() {
	this->recordingLiveSceneAs2DSlices = true;
}


void ITMDynamicFusionLogger::TurnRecordingLiveSceneAs2DSlicesOff() {
	this->recordingLiveSceneAs2DSlices = false;
}


void ITMDynamicFusionLogger::ToggleRecordingLiveSceneAs2DSlices() {
	this->recordingLiveSceneAs2DSlices = !this->recordingLiveSceneAs2DSlices;
}


void ITMDynamicFusionLogger::TurnRecordingCanonicalSceneAs2DSlicesOn() {
	this->recordingCanonicalSceneAs2DSlices = true;
}


void ITMDynamicFusionLogger::TurnRecordingCanonicalSceneAs2DSlicesOff() {
	this->recordingCanonicalSceneAs2DSlices = false;
}


void ITMDynamicFusionLogger::TurnRecordingScene1DSlicesWithUpdatesOn() {
	this->recordingScene1DSlicesWithUpdates = true;
}


void ITMDynamicFusionLogger::TurnRecordingScene1DSlicesWithUpdatesOff() {
	this->recordingScene1DSlicesWithUpdates = false;
}


void ITMDynamicFusionLogger::TurnRecordingScene2DSlicesWithUpdatesOn() {
	this->recordingScene2DSlicesWithUpdates = true;
}


void ITMDynamicFusionLogger::TurnRecordingScene2DSlicesWithUpdatesOff() {
	this->recordingScene2DSlicesWithUpdates = false;
}


void ITMDynamicFusionLogger::ToggleRecordingScene2DSlicesWithUpdates() {
	this->recordingScene2DSlicesWithUpdates = !this->recordingScene2DSlicesWithUpdates;
}


void ITMDynamicFusionLogger::TurnRecording3DSceneAndWarpProgressionOn() {
	this->recording3DSceneAndWarpProgression = true;
}


void ITMDynamicFusionLogger::TurnRecording3DSceneAndWarpProgressionOff() {
	this->recording3DSceneAndWarpProgression = false;
}


void ITMDynamicFusionLogger::ToggleRecording3DSceneAndWarpProgression() {
	this->recording3DSceneAndWarpProgression = !this->recording3DSceneAndWarpProgression;
}


void ITMDynamicFusionLogger::TurnRecordingEnergiesToFilesOn() {
	this->recordingEnergiesToFile = true;
}


void ITMDynamicFusionLogger::TurnRecordingEnergiesToFilesOff() {
	this->recordingEnergiesToFile = false;
}


void ITMDynamicFusionLogger::TurnPlottingEnergiesOn() {
	this->plottingEnergies = true;
}


void ITMDynamicFusionLogger::TurnPlottingEnergiesOff() {
	this->plottingEnergies = false;
}

// endregion ===========================================================================================================
// region ========================================= GETTERS ============================================================


std::string ITMDynamicFusionLogger::GetOutputDirectory() const {
	return this->outputDirectory;
}

bool ITMDynamicFusionLogger::IsRecordingLiveSceneAs2DSlices() const {
	return this->recordingLiveSceneAs2DSlices;
}


bool ITMDynamicFusionLogger::IsRecordingCanonicalSceneAs2DSlices() const {
	return this->recordingCanonicalSceneAs2DSlices;
}


bool ITMDynamicFusionLogger::IsRecordingScene1DSlicesWithUpdates() const {
	return this->recordingScene1DSlicesWithUpdates;
}


bool ITMDynamicFusionLogger::IsRecordingScene2DSlicesWithUpdates() const {
	return this->recordingScene2DSlicesWithUpdates;
}


bool ITMDynamicFusionLogger::IsRecording3DSceneAndWarpProgression() const {
	return this->recording3DSceneAndWarpProgression;
}


bool ITMDynamicFusionLogger::IsRecordingEnergiesToFile() const {
	return this->recordingEnergiesToFile;
}


bool ITMDynamicFusionLogger::IsPlottingEnergies() const {
	return this->plottingEnergies;
}

// endregion ===========================================================================================================


ITMDynamicFusionLogger::ITMDynamicFusionLogger() :
		focusSliceRadius(3),
		scene1DSliceVisualizer(),
		scene2DSliceVisualizer() {}

// region ===================================== RECORDING ==============================================================


void ITMDynamicFusionLogger::InitializeFrameRecording() {

	//TODO: make all visualizer/logger classes re-usable, i.e. just change the path & build them in the constructor (don't use pointers) -Greg (GitHub:Algomorph)

	this->canonicalScene = canonicalScene;
	this->liveScene = liveScene;

	this->hasFocusCoordinates = hasFocusCoordinates;
	this->focusCoordinates = focusCoordinates;
	this->outputDirectory = outputDirectory;

	// region ================================== 1D/2D SLICE RECORDING =================================================
	if (hasFocusCoordinates) {
		scene2DSliceVisualizer.reset(new ITMScene2DSliceVisualizer<ITMVoxelCanonical,ITMVoxelLive,ITMVoxelIndex>(focusCoordinates, 100, 16.0, planeFor2DSlices));

		if (recordingCanonicalSceneAs2DSlices) {
			scene2DSliceVisualizer->SaveCanonicalSceneSlicesAs2DImages_AllDirections(
					canonicalScene, GetOutputDirectoryPrefixForCanonicalSceneAsSlices());
		}
		if (recordingLiveSceneAs2DSlices) {
			scene2DSliceVisualizer->SaveLiveSceneSlicesAs2DImages_AllDirections(
					liveScene, GetOutputDirectoryPrefixForLiveSceneAsSlices());
		}
		if (recordingScene1DSlicesWithUpdates) {
			this->scene1DSliceVisualizer.reset(new ITMScene1DSliceVisualizer(focusCoordinates, AXIS_X, 16));
			scene1DSliceVisualizer->Plot1DSceneSlice(canonicalScene, Vector4i(97, 181, 193, 255), 3.0);
			scene1DSliceVisualizer->Plot1DIndexedSceneSlice(liveScene, Vector4i(183, 115, 46, 255), 3.0, 0);
		}
		if (recordingScene2DSlicesWithUpdates) {
			std::cout << yellow << "Recording 2D scene slices with warps & warped live scene progression as images in "
			          << GetOutputDirectoryFor2DSceneSlicesWithWarps() << " and "
			          << GetOutputDirectoryFor2DLiveSceneSliceProgression() << " respectively..."
			          << reset << std::endl;
			InitializeWarp2DSliceRecording(canonicalScene, liveScene);
		}

	}
	// endregion
	if (plottingEnergies) {
		this->energyPlotter.reset(new ITMSceneTrackingEnergyPlotter());
	}

	// region ========================= INITIALIZE WARP RECORDING ======================================================
	bench::StartTimer("TrackMotion_2_RecordingEnergy");

	const std::string energyStatFilePath = outputDirectory + "/energy.txt";
	energyStatisticsFile = std::ofstream(energyStatFilePath.c_str(), std::ios_base::out);
	energyStatisticsFile << "data" << "," << "level_set" << "," << "smoothness" << ","
	                     << "killing" << "," << "total" << std::endl;
	bench::StopTimer("TrackMotion_2_RecordingEnergy");

	if (recording3DSceneAndWarpProgression) {
		scene3DLogger->SaveScenesCompact();
		scene3DLogger->StartSavingWarpState();
		if (hasFocusCoordinates) {
			scene3DLogger->ClearHighlights();
		}
	}

	// endregion =======================================================================================================

}


void ITMDynamicFusionLogger::InitializeWarp2DSliceRecording(
		ITMScene<ITMVoxelCanonical, ITMVoxelIndex>* canonicalScene,
		ITMScene<ITMVoxelLive, ITMVoxelIndex>* sourceLiveScene) {

	cv::Mat canonicalImg = scene2DSliceVisualizer->DrawCanonicalSceneImageAroundPoint(canonicalScene) * 255.0f;
	cv::Mat canonicalImgOut;
	canonicalImg.convertTo(canonicalImgOut, CV_8UC1);
	cv::cvtColor(canonicalImgOut, canonicalImgOut, cv::COLOR_GRAY2BGR);
	MakeOrClearOutputDirectoriesFor2DSceneSlices();
	cv::imwrite(GetOutputDirectoryFor2DSceneSlicesWithWarps() + "/canonical.png", canonicalImgOut);
	cv::Mat liveImg = scene2DSliceVisualizer->DrawLiveSceneImageAroundPoint(sourceLiveScene, 0) * 255.0f;
	cv::Mat liveImgOut;
	liveImg.convertTo(liveImgTemplate, CV_8UC1);
	cv::cvtColor(liveImgTemplate, liveImgOut, cv::COLOR_GRAY2BGR);
	cv::imwrite(GetOutputDirectoryFor2DSceneSlicesWithWarps() + "/live.png", liveImgOut);
	//TODO: this is kinda backwards. Just build this in the constructor using constants from rasterizer for size. -Greg (GitHub: Algomorph)
	blank = cv::Mat::zeros(liveImg.rows, liveImg.cols, CV_8UC1);
}


void
ITMDynamicFusionLogger::SaveWarpSlices(int iteration) {
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
				GetOutputDirectoryFor2DSceneSlicesWithWarps() + "/warp" + numStringStream.str() + ".png";
		cv::imwrite(image_name, warpImgOut);
		cv::Mat liveImg = scene2DSliceVisualizer->DrawLiveSceneImageAroundPoint(liveScene, 1) * 255.0f;
		cv::Mat liveImgOut;
		liveImg.convertTo(liveImgOut, CV_8UC1);
		cv::cvtColor(liveImgOut, liveImgOut, cv::COLOR_GRAY2BGR);
		cv::imwrite(GetOutputDirectoryFor2DLiveSceneSliceProgression() + "/live " + numStringStream.str() +
		            ".png", liveImgOut);
	}
	if (hasFocusCoordinates && recordingScene1DSlicesWithUpdates) {
		scene1DSliceVisualizer->Plot1DIndexedSceneSlice(liveScene, Vector4i(0, 0, 0, 255), 1.0, 1);
		scene1DSliceVisualizer->Draw1DWarpUpdateVector(canonicalScene, Vector4i(255, 0, 0, 255));
	}
}


void ITMDynamicFusionLogger::FinalizeFrameRecording() {
	if (recording3DSceneAndWarpProgression) {
		scene3DLogger->StopSavingWarpState();
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
	}
	if (plottingEnergies){
		energyPlotter->SaveScreenshot(this->outputDirectory + "/energy_plot.png");
		energyPlotter.reset();
	}
	scene1DSliceVisualizer.reset();
	scene2DSliceVisualizer.reset();
	energyStatisticsFile.close();
}


void ITMDynamicFusionLogger::SaveWarps() {
	if (recording3DSceneAndWarpProgression) {
		this->scene3DLogger->SaveCurrentWarpState();
	}
}


void ITMDynamicFusionLogger::RecordAndPlotEnergies(double totalDataEnergy,
                                                   double totalLevelSetEnergy,
                                                   double totalKillingEnergy,
                                                   double totalSmoothnessEnergy,
                                                   double totalEnergy) {
	if (this->recordingEnergiesToFile) {
		energyStatisticsFile << totalDataEnergy << ", " << totalLevelSetEnergy << ", " << totalKillingEnergy << ", "
		                     << totalSmoothnessEnergy << ", " << totalEnergy << std::endl;
	}
	if (this->plottingEnergies){
		this->energyPlotter->AddDataPoints(static_cast<float>(totalDataEnergy),
		                                   static_cast<float>(totalSmoothnessEnergy),
		                                   static_cast<float>(totalLevelSetEnergy),
		                                   static_cast<float>(totalKillingEnergy));
	}
}


bool ITMDynamicFusionLogger::IsRecordingWarp2DSlices() {
	return this->recordingScene2DSlicesWithUpdates;
}


bool ITMDynamicFusionLogger::IsRecordingWarps() {
	return this->recording3DSceneAndWarpProgression;
}


void ITMDynamicFusionLogger::LogHighlight(int hash, int locId,
                                          ITMHighlightIterationInfo info) {
	scene3DLogger->LogHighlight(hash, locId, 0, info);
}


ITMDynamicFusionLogger::~ITMDynamicFusionLogger() {
	delete this->scene3DLogger;
}

// endregion ===========================================================================================================
// region ================================ PATH GENERATION =============================================================



std::string ITMDynamicFusionLogger::GetOutputDirectoryFor2DSceneSlicesWithWarps() const {
	fs::path path(fs::path(this->outputDirectory) / (iterationFramesFolderName + "_" +
	                                                 PlaneToString(this->scene2DSliceVisualizer->GetPlane())));
	return path.string();
}


std::string ITMDynamicFusionLogger::GetOutputDirectoryFor2DLiveSceneSliceProgression() const {
	fs::path path(fs::path(this->outputDirectory) / (liveIterationFramesFolderName + "_" +
	                                                 PlaneToString(this->scene2DSliceVisualizer->GetPlane())));
	return path.string();
}


std::string
ITMDynamicFusionLogger::GetOutputDirectoryPrefixForLiveSceneAsSlices() const {
	fs::path path(fs::path(this->outputDirectory) / liveSceneRasterizedFolderName);
	return path.string();
}


std::string
ITMDynamicFusionLogger::GetOutputDirectoryPrefixForCanonicalSceneAsSlices() const {
	fs::path path(fs::path(this->outputDirectory) / canonicalSceneRasterizedFolderName);
	return path.string();
}

inline
static void ClearDirectory(const fs::path& path) {
	for (fs::directory_iterator end_dir_it, it(path); it != end_dir_it; ++it) {
		fs::remove_all(it->path());
	}
}


void ITMDynamicFusionLogger::MakeOrClearOutputDirectoriesFor2DSceneSlices() const {
	auto ClearIfExistsMakeIfDoesnt = [&](std::string pathString) {
		fs::path path = pathString;
		if (!fs::exists(path)) {
			fs::create_directories(path);
		} else {
			ClearDirectory(path);
		}
	};

	if (recordingLiveSceneAs2DSlices) {
		ClearIfExistsMakeIfDoesnt(GetOutputDirectoryPrefixForLiveSceneAsSlices());
	}
	if (recordingCanonicalSceneAs2DSlices) {
		ClearIfExistsMakeIfDoesnt(GetOutputDirectoryPrefixForCanonicalSceneAsSlices());
	}
	if (recordingScene2DSlicesWithUpdates) {
		ClearIfExistsMakeIfDoesnt(GetOutputDirectoryFor2DLiveSceneSliceProgression());
		ClearIfExistsMakeIfDoesnt(GetOutputDirectoryFor2DSceneSlicesWithWarps());
	}
}



// endregion ===========================================================================================================