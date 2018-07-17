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
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::iterationFramesFolderName =
		"bucket_interest_region_2D_iteration_slices";
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::liveIterationFramesFolderName =
		"bucket_interest_region_live_slices";
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::canonicalSceneRasterizedFolderName =
		"canonical_rasterized";
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::liveSceneRasterizedFolderName =
		"live_rasterized";
// endregion ================================== END CONSTANT DEFINITIONS ===============================================
// region ======================================= SETTERS ==============================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::SetScenes(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {
	this->canonicalScene = canonicalScene;
	this->liveScene = liveScene;
	this->scene3DLogger = new ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>(
			canonicalScene, liveScene, outputDirectory);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::SetOutputDirectory(std::string outputDirectory) {
	this->scene3DLogger->SetPath(outputDirectory);
	this->outputDirectory = outputDirectory;
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::SetFocusCoordinates(Vector3i focusCoordinates) {
	hasFocusCoordinates = true;
	this->focusCoordinates = focusCoordinates;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::SetPlaneFor2Dand3DSlices(Plane plane) {
	this->planeFor2Dand3DSlices = plane;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::Set3DSliceInPlaneRadius(
		unsigned int _3dSliceInPlaneRadius) {
	this->_3dSliceInPlaneRadius = _3dSliceInPlaneRadius;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::Set3DSliceOutOfPlaneRadius(
		unsigned int _3dSliceOutOfPlaneRadius) {
	this->_3dSliceOutOfPlaneRadius = _3dSliceOutOfPlaneRadius;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::SetShutdownRequestedFlagLocation(bool* flag) {
	this->shutdownRequestedFlag = flag;
}

// endregion ===========================================================================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::RequestAppShutdown() {
	if (shutdownRequestedFlag) {
		*shutdownRequestedFlag = true;
	}
}

// region ============================================== SWITCHES ======================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::TurnRecordingLiveSceneAs2DSlicesOn() {
	this->recordingLiveSceneAs2DSlices = true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::TurnRecordingLiveSceneAs2DSlicesOff() {
	this->recordingLiveSceneAs2DSlices = false;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::ToggleRecordingLiveSceneAs2DSlices() {
	this->recordingLiveSceneAs2DSlices = !this->recordingLiveSceneAs2DSlices;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::TurnRecordingCanonicalSceneAs2DSlicesOn() {
	this->recordingCanonicalSceneAs2DSlices = true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::TurnRecordingCanonicalSceneAs2DSlicesOff() {
	this->recordingCanonicalSceneAs2DSlices = false;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::TurnRecordingScene1DSlicesWithUpdatesOn() {
	this->recordingScene1DSlicesWithUpdates = true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::TurnRecordingScene1DSlicesWithUpdatesOff() {
	this->recordingScene1DSlicesWithUpdates = false;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::TurnRecordingScene2DSlicesWithUpdatesOn() {
	this->recordingScene2DSlicesWithUpdates = true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::TurnRecordingScene2DSlicesWithUpdatesOff() {
	this->recordingScene2DSlicesWithUpdates = false;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::TurnRecordingScene3DSlicesWithUpdatesOn() {
	this->recordingScene3DSlicesWithUpdates = true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::TurnRecordingScene3DSlicesWithUpdatesOff() {
	this->recordingScene3DSlicesWithUpdates = false;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::ToggleRecordingScene2DSlicesWithUpdates() {
	this->recordingScene2DSlicesWithUpdates = !this->recordingScene2DSlicesWithUpdates;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::TurnRecording3DSceneAndWarpProgressionOn() {
	this->recording3DSceneAndWarpProgression = true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::TurnRecording3DSceneAndWarpProgressionOff() {
	this->recording3DSceneAndWarpProgression = false;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::ToggleRecording3DSceneAndWarpProgression() {
	this->recording3DSceneAndWarpProgression = !this->recording3DSceneAndWarpProgression;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::TurnRecordingEnergiesToFilesOn() {
	this->recordingEnergiesToFile = true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::TurnRecordingEnergiesToFilesOff() {
	this->recordingEnergiesToFile = false;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::TurnPlottingEnergiesOn() {
	this->plottingEnergies = true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::TurnPlottingEnergiesOff() {
	this->plottingEnergies = false;
}

// endregion ===========================================================================================================
// region ========================================= GETTERS ============================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetOutputDirectory() const {
	return this->outputDirectory;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::IsRecordingLiveSceneAs2DSlices() const {
	return this->recordingLiveSceneAs2DSlices;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::IsRecordingCanonicalSceneAs2DSlices() const {
	return this->recordingCanonicalSceneAs2DSlices;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::IsRecordingScene1DSlicesWithUpdates() const {
	return this->recordingScene1DSlicesWithUpdates;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::IsRecordingScene2DSlicesWithUpdates() const {
	return this->recordingScene2DSlicesWithUpdates;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::IsRecordingScene3DSlicesWithUpdates() const {
	return this->recordingScene3DSlicesWithUpdates;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::IsRecording3DSceneAndWarpProgression() const {
	return this->recording3DSceneAndWarpProgression;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::IsRecordingEnergiesToFile() const {
	return this->recordingEnergiesToFile;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::IsPlottingEnergies() const {
	return this->plottingEnergies;
}

// endregion ===========================================================================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::ITMDynamicFusionLogger() :
		focusSliceRadius(3),
		scene1DSliceVisualizer(),
		scene2DSliceVisualizer() {}

// region ===================================== RECORDING ==============================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::InitializeFrameRecording() {

	//TODO: make all visualizer/logger classes re-usable, i.e. just change the path & build them in the constructor (don't use pointers) -Greg (GitHub:Algomorph)

	this->canonicalScene = canonicalScene;
	this->liveScene = liveScene;

	this->hasFocusCoordinates = hasFocusCoordinates;
	this->focusCoordinates = focusCoordinates;
	this->outputDirectory = outputDirectory;

	// region ================================== 1D/2D SLICE RECORDING =================================================
	if (hasFocusCoordinates) {

		if (recordingScene1DSlicesWithUpdates) {
			this->scene1DSliceVisualizer.reset(new ITMSceneSliceVisualizer1D(focusCoordinates, AXIS_X, 16));
			scene1DSliceVisualizer->Plot1DSceneSlice(canonicalScene, Vector4i(97, 181, 193, 255), 3.0);
			scene1DSliceVisualizer->Plot1DIndexedSceneSlice(liveScene, Vector4i(183, 115, 46, 255), 3.0, 0);
		}

		scene2DSliceVisualizer.reset(
				new ITMSceneSliceVisualizer2D<TVoxelCanonical, TVoxelLive, TIndex>(focusCoordinates, 100,
				                                                                              16.0,
				                                                                              planeFor2Dand3DSlices));

		if (recordingCanonicalSceneAs2DSlices) {
			scene2DSliceVisualizer->SaveCanonicalSceneSlicesAs2DImages_AllDirections(
					canonicalScene, GetOutputDirectoryPrefixForCanonicalSceneAsSlices());
		}
		if (recordingLiveSceneAs2DSlices) {
			scene2DSliceVisualizer->SaveLiveSceneSlicesAs2DImages_AllDirections(
					liveScene, GetOutputDirectoryPrefixForLiveSceneAsSlices());
		}
		if (recordingScene2DSlicesWithUpdates) {
			std::cout << yellow << "Recording 2D scene slices with warps & warped live scene progression as images in "
			          << GetOutputDirectoryFor2DSceneSlicesWithWarps() << " and "
			          << GetOutputDirectoryFor2DLiveSceneSliceProgression() << " respectively..."
			          << reset << std::endl;
			InitializeWarp2DSliceRecording(canonicalScene, liveScene);
		}

		if (recordingScene3DSlicesWithUpdates) {
			if(!scene3DSliceVisualizer){
				scene3DSliceVisualizer.reset(new ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>
					                             (canonicalScene, liveScene, focusCoordinates,
					                              planeFor2Dand3DSlices, _3dSliceInPlaneRadius,
					                              _3dSliceOutOfPlaneRadius));
			}else{
				scene3DSliceVisualizer->TriggerRebuildSlices();
			}
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

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::InitializeWarp2DSliceRecording(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
		ITMScene<TVoxelLive, TIndex>* sourceLiveScene) {

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

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveWarpSlices(int iteration) {
	if (hasFocusCoordinates) {
		if (recordingScene2DSlicesWithUpdates) {
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
		if (recordingScene1DSlicesWithUpdates) {
			scene1DSliceVisualizer->Plot1DIndexedSceneSlice(liveScene, Vector4i(0, 0, 0, 255), 1.0, 1);
			scene1DSliceVisualizer->Draw1DWarpUpdateVector(canonicalScene, Vector4i(255, 0, 0, 255));
		}
		if (recordingScene3DSlicesWithUpdates) {
			scene3DSliceVisualizer->TriggerDrawWarpUpdates();
			scene3DSliceVisualizer->TriggerUpdateLiveState();
		}
	}
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::FinalizeFrameRecording() {
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
	if (plottingEnergies) {
		energyPlotter->SaveScreenshot(this->outputDirectory + "/energy_plot.png");
		energyPlotter.reset();
	}
	if (recordingScene3DSlicesWithUpdates) {
		scene3DSliceVisualizer->TriggerBuildFusedCanonical();
	}
	scene1DSliceVisualizer.reset();
	scene2DSliceVisualizer.reset();
	energyStatisticsFile.close();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveWarps() {
	if (recording3DSceneAndWarpProgression) {
		this->scene3DLogger->SaveCurrentWarpState();
	}
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::RecordAndPlotEnergies(double totalDataEnergy,
                                                                                        double totalLevelSetEnergy,
                                                                                        double totalKillingEnergy,
                                                                                        double totalSmoothnessEnergy,
                                                                                        double totalEnergy) {
	if (this->recordingEnergiesToFile) {
		energyStatisticsFile << totalDataEnergy << ", " << totalLevelSetEnergy << ", " << totalKillingEnergy << ", "
		                     << totalSmoothnessEnergy << ", " << totalEnergy << std::endl;
	}
	if (this->plottingEnergies) {
		this->energyPlotter->AddDataPoints(static_cast<float>(totalDataEnergy),
		                                   static_cast<float>(totalSmoothnessEnergy),
		                                   static_cast<float>(totalLevelSetEnergy),
		                                   static_cast<float>(totalKillingEnergy));
	}
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::IsRecordingWarp2DSlices() {
	return this->recordingScene2DSlicesWithUpdates;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::IsRecordingWarps() {
	return this->recording3DSceneAndWarpProgression;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::LogHighlight(int hash, int locId,
                                                                               ITMHighlightIterationInfo info) {
	scene3DLogger->LogHighlight(hash, locId, 0, info);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::~ITMDynamicFusionLogger() {
	delete this->scene3DLogger;
}

// endregion ===========================================================================================================
// region ================================ PATH GENERATION =============================================================


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string
ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetOutputDirectoryFor2DSceneSlicesWithWarps() const {
	fs::path path(fs::path(this->outputDirectory) / (iterationFramesFolderName + "_" +
	                                                 PlaneToString(this->scene2DSliceVisualizer->GetPlane())));
	return path.string();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string
ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetOutputDirectoryFor2DLiveSceneSliceProgression() const {
	fs::path path(fs::path(this->outputDirectory) / (liveIterationFramesFolderName + "_" +
	                                                 PlaneToString(this->scene2DSliceVisualizer->GetPlane())));
	return path.string();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string
ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetOutputDirectoryPrefixForLiveSceneAsSlices() const {
	fs::path path(fs::path(this->outputDirectory) / liveSceneRasterizedFolderName);
	return path.string();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string
ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetOutputDirectoryPrefixForCanonicalSceneAsSlices() const {
	fs::path path(fs::path(this->outputDirectory) / canonicalSceneRasterizedFolderName);
	return path.string();
}

inline
static void ClearDirectory(const fs::path& path) {
	for (fs::directory_iterator end_dir_it, it(path); it != end_dir_it; ++it) {
		fs::remove_all(it->path());
	}
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::MakeOrClearOutputDirectoriesFor2DSceneSlices() const {
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