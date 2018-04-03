//  ================================================================
//  Created by Gregory Kramida on 12/20/17.
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
#include "ITMSceneLogger.h"

#include <regex>

//boost
#include <boost/filesystem.hpp>

//local
#include "../../Objects/Scene/ITMRepresentationAccess.h"
#include "ITMSceneLogger_InterestRegionInfo.tpp"
#include "ITMSceneLogger_SceneSlice.tpp"


namespace fs = boost::filesystem;

using namespace ITMLib;

// region ================================= CONSTANT DEFINITIONS =======================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::warpUpdatesFilename = "warp_updates";
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::binaryFileExtension = ".dat";
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::textFileExtension = ".txt";
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::canonicalName = "canonical";
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::liveName = "live";
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::highlightFilterInfoFilename = "highlight_filter_info.txt";
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::minRecurrenceHighlightFilterName = "min_recurrence_count_filter:";
//endregion
// region ================================= CONSTRUCTORS & DESTRUCTORS =================================================

/**
 * \brief Constructor: build a logger with the given path and scenes.
 * \tparam TVoxelCanonical
 * \tparam TVoxelLive
 * \tparam TIndex
 * \param path
 * \param canonicalScene
 * \param liveScene
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::ITMSceneLogger(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
		ITMScene<TVoxelLive, TIndex>* liveScene,
		std::string path) :
		fullCanonicalSceneLogger(new ITMWarpSceneLogger<TVoxelCanonical,TIndex>(canonicalScene)),
		activeWarpLogger(fullCanonicalSceneLogger),
		liveScene(liveScene),
		highlights("Hash ID", "Local voxel ix", "Frame", ""),
		mode(FULL_SCENE){
	if(canonicalScene == nullptr){
		throw std::runtime_error("Argument 'canonicalScene' cannot be set to nullptr here." __FILE__
		                         + std::to_string(__LINE__));
	}
	if (path != "") {
		SetPath(path);
	}
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::~ITMSceneLogger() {
	for(auto idAndSliceIterator = this->slices.begin(); idAndSliceIterator != this->slices.end(); ++idAndSliceIterator){
		delete idAndSliceIterator->second;
	}
	fullCanonicalSceneLogger->StopLoadingWarpState();
}
//endregion
// region ================================= GETTERS, SETTERS, PRINTERS =================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
int ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetVoxelCount() const {
	return activeWarpLogger->GetVoxelCount();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetScenesLoaded() const {
	return activeWarpLogger->Loaded();
}
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SetScenes(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {
	this->liveScene = liveScene;
	this->activeWarpLogger->scene = canonicalScene;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetPath() const {
	return this->path.string();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
unsigned int ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetGeneralIterationCursor() const {
	return this->activeWarpLogger->GetIterationCursor();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
unsigned int ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetInterestIterationCursor() const {
	return this->interestIterationCursor;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITM3DNestedMapOfArrays<ITMHighlightIterationInfo>
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetHighlights() const {
	return ITM3DNestedMapOfArrays<ITMHighlightIterationInfo>(highlights);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::PrintHighlights() {
	std::cout << "*** Highlights ***" << std::endl;
	std::cout << this->highlights << std::endl;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::CheckPath() {
	if (!liveScene || activeWarpLogger->Empty()) {
		std::cerr << "At least one of the two scenes, canonical/live, was not set to an actual scene. "
		          << __FILE__ << ":" << __LINE__ << std::endl;
	}
	if (!fs::is_directory(this->path)) {
		std::cout << "The directory '" << path << "' was not found.";
		return false;
	}
	return true;
}

/**
 * \brief Set the path to save everything to or load everything from. If a directory doesn't exist at that path, one
 * will be created.
 * \param path the new path.
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SetPath(std::string path) {
	if(this->path != ""){
		//clean up
		this->StopLoadingWarpState();
		slices.clear();
	}
	this->path = path;
	if (!fs::create_directories(this->path) && !fs::is_directory(this->path)) {
		DIEWITHEXCEPTION(std::string("Could not create the directory '") + path + "'. Exiting.["
				                 __FILE__
				                 ": " + std::to_string(__LINE__) + "]");
	}
	this->activeWarpLogger->scenePath = (this->path / canonicalName).string();
	this->activeWarpLogger->warpPath = (this->path / (warpUpdatesFilename + binaryFileExtension)).string();
	this->livePath = this->path / liveName;
	this->highlightsBinaryPath = this->path / ("highlights" + binaryFileExtension);
	this->highlightsTextPath = this->path / ("highlights" + textFileExtension);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const ITMScene<TVoxelCanonical, TIndex>* ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetActiveWarpScene() const{
	return this->activeWarpLogger->scene;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const ITMScene<TVoxelLive, TIndex>* ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetLiveScene() const{
	return this->liveScene;
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetIsActiveSceneASlice() const {
	return this->activeWarpLogger->isSlice;
}
//endregion
// region ================================= SCENE SAVING / LOADING =====================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveScenes() {
	if (!CheckPath()) {
		return false;
	}
	std::cout << "Saving scenes for current frame (this might take awhile)..." << std::endl;
	std::cout.flush();
	liveScene->SaveToDirectory(livePath.string());
	activeWarpLogger->Save();
	std::cout << "Scenes saved." << std::endl;
	return true;
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::LoadScenes() {
	if (!CheckPath()) {
		return false;
	}
	std::cout << "Loading scenes for current frame (this might take awhile)..." << std::endl;
	std::cout.flush();
	liveScene->LoadFromDirectory(livePath.c_str());
	activeWarpLogger->Load();
	std::cout << "Scenes loaded." << std::endl;
	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveScenesCompact() {
	if (!liveScene || activeWarpLogger->Empty()) {
		std::cerr << "At least one of the two scenes, canonical/live, was not set to an actual scene. "
		          << __FILE__ << ":" << __LINE__ << std::endl;
	}
	if (!fs::is_directory(this->path)) {
		std::cout << "The directory '" << path << "' was not found.";
		return false;
	}
	std::cout << "Saving scenes for current frame (this might take awhile)..." << std::endl;
	std::cout.flush();
	liveScene->SaveToDirectoryCompact_CPU(livePath.string());
	activeWarpLogger->SaveCompact();
	std::cout << "Scenes saved." << std::endl;
	return true;
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::LoadScenesCompact() {
	if (!CheckPath()) {
		return false;
	}
	std::cout << "Loading scenes for current frame (this might take awhile)..." << std::endl;
	std::cout.flush();
	ITMSceneReconstructionEngine<TVoxelLive, TIndex>* reconstructionEngineLive =
			ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxelLive, TIndex>(
					ITMLibSettings::DEVICE_CPU);
	reconstructionEngineLive->ResetScene(liveScene);
	liveScene->LoadFromDirectoryCompact_CPU(livePath.c_str());
	activeWarpLogger->LoadCompact();
	std::cout << "Scenes loaded." << std::endl;
	delete reconstructionEngineLive;
	return true;
}
//endregion
// region ================================= WARP STATE ONLINE SAVING / LOADING / BUFFERING =============================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::StartSavingWarpState(unsigned int frameIx) {
	return activeWarpLogger->StartLoadingWarpState(frameIx);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::StopSavingWarpState() {
	activeWarpLogger->StopSavingWarpState();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveCurrentWarpState() {
	return activeWarpLogger->SaveCurrentWarpState();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::StartLoadingWarpState() {
	return activeWarpLogger->StartLoadingWarpState();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::StartLoadingWarpState(unsigned int& frameIx) {
	return activeWarpLogger->StartLoadingWarpState(frameIx);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::StopLoadingWarpState() {
	activeWarpLogger->StopSavingWarpState();
}

/**
 * \brief Transfers the warp state from the warp file to the scene, imitating the .warp_t and .warp_t_update fields after
 * the current iteration.
 * \return True on success, false otherwise
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::LoadCurrentWarpState() {
	return activeWarpLogger->LoadCurrentWarpState();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::LoadPreviousWarpState() {
	return activeWarpLogger->LoadPreviousWarpState();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::IsLoadingWarpState() {
	return activeWarpLogger->IsLoadingWarpState();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::BufferCurrentWarpState(void* externalBuffer) {
	return activeWarpLogger->BufferCurrentWarpState(externalBuffer);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::BufferPreviousWarpState(void* externalBuffer) {
	return activeWarpLogger->BufferPreviousWarpState(externalBuffer);
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::BufferWarpStateAt(void* externalBuffer,
                                                                            unsigned int iterationIndex) {
	return activeWarpLogger->BufferWarpStateAt(externalBuffer, iterationIndex);
}
//endregion
// region ================================= HIGHLIGHTS =================================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::LogHighlight(
		int hashId, int voxelLocalIndex, int frameNumber, ITMHighlightIterationInfo info) {
	highlights.InsertOrdered(hashId, voxelLocalIndex, frameNumber, info);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveHighlights(std::string filePostfix) {
	if (!fs::is_directory(this->path)) {
		std::cout << "The directory '" << path << "' was not found.";
		return false;
	}
	highlightsBinaryPath = this->path / ("highlights" + filePostfix + binaryFileExtension);
	if (!this->highlights.SaveToFile(highlightsBinaryPath.c_str())) {
		std::cerr << "Could not save highlights to " << highlightsBinaryPath << std::endl;
		return false;
	} else {
		std::cout << "Saved highlights to" << highlightsBinaryPath << std::endl;
	}
	if (!this->highlights.SaveToTextFile(highlightsTextPath.c_str())) {
		std::cerr << "Could not save highlights to " << highlightsTextPath << std::endl;
		return false;
	} else {
		std::cout << "Saved highlights to" << highlightsTextPath << std::endl;
	}
	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::LoadHighlights(bool applyFilters, std::string filePostfix) {
	if (!fs::is_directory(this->path)) {
		std::cout << "The directory '" << path << "' was not found.";
		return false;
	}
	highlightsBinaryPath = this->path / ("highlights" + filePostfix + binaryFileExtension);
	if (!this->highlights.LoadFromFile(highlightsBinaryPath.c_str())) {
		std::cout << "Could not load highlights from " << highlightsBinaryPath << std::endl;
		return false;
	} else {
		std::cout << "Loaded highlights from " << highlightsBinaryPath << std::endl;
	}
	if (applyFilters) {
		fs::path wouldBeFilterInfoPath = path / fs::path(highlightFilterInfoFilename);
		if (fs::is_regular_file(wouldBeFilterInfoPath)) {
			std::ifstream highlightFilterInfoNote(wouldBeFilterInfoPath.c_str(), std::ios_base::in);
			std::string filterName;
			highlightFilterInfoNote >> filterName;
			if (filterName == minRecurrenceHighlightFilterName) {
				highlightFilterInfoNote >> minHighlightRecurrenceCount;
			}
			highlightFilterInfoNote.close();
		}
		FilterHighlights(minHighlightRecurrenceCount);
	}
	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::FilterHighlights(int anomalyFrameCountMinimum) {
	minHighlightRecurrenceCount = std::max(minHighlightRecurrenceCount, anomalyFrameCountMinimum);
	highlights = highlights.FilterBasedOnLevel0Lengths(anomalyFrameCountMinimum);
	std::ofstream highlightFilterInfoNote((path / fs::path(highlightFilterInfoFilename)).c_str(), std::ios_base::out);
	highlightFilterInfoNote << "min_reccurence_count_filter:" << " " << anomalyFrameCountMinimum << std::endl;
	highlightFilterInfoNote.close();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::ClearHighlights() {
	this->highlights.Clear();
}

//endregion