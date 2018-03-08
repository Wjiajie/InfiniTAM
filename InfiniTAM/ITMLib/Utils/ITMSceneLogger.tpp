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
#include "../Objects/Scene/ITMRepresentationAccess.h"
#include "ITMSceneLogger_InterestRegionInfo.tpp"

namespace fs = boost::filesystem;

using namespace ITMLib;

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::binaryFileExtension = ".dat";
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::textFileExtension = ".txt";
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::highlightFilterInfoFilename = "highlight_filter_info.txt";
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::minRecurrenceHighlightFilterName = "min_reccurence_count_filter:";

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
		std::string path,
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
		ITMScene<TVoxelLive, TIndex>* liveScene) :
		canonicalScene(canonicalScene),
		liveScene(liveScene),
		highlights("Hash ID", "Local voxel ix", "Frame", "") {
	if (path != "") {
		SetPath(path);
	}
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveScenes() {
	if (!CheckDirectory()) {
		return false;
	}
	std::cout << "Saving scenes for current frame (this might take awhile)..." << std::endl;
	std::cout.flush();
	liveScene->SaveToDirectory(livePath.string());
	canonicalScene->SaveToDirectory(canonicalPath.string());
	ITMSceneStatisticsCalculator<TVoxelCanonical, TIndex> statisticsCalculator;
	this->voxelCount = statisticsCalculator.ComputeAllocatedVoxelCount(canonicalScene);
	std::cout << "Scenes saved." << std::endl;
	return true;
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::LoadScenes() {
	if (!CheckDirectory()) {
		return false;
	}
	std::cout << "Loading scenes for current frame (this might take awhile)..." << std::endl;
	std::cout.flush();
	liveScene->LoadFromDirectory(livePath.c_str());
	canonicalScene->LoadFromDirectory(canonicalPath.c_str());
	ITMSceneStatisticsCalculator<TVoxelCanonical, TIndex> statisticsCalculator;
	this->voxelCount = statisticsCalculator.ComputeAllocatedVoxelCount(canonicalScene);
	std::cout << "Scenes loaded." << std::endl;
	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveScenesCompact() {
	if (!liveScene || !canonicalScene) {
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
	canonicalScene->SaveToDirectoryCompact_CPU(canonicalPath.string());
	ITMSceneStatisticsCalculator<TVoxelCanonical, TIndex> statisticsCalculator;
	this->voxelCount = statisticsCalculator.ComputeAllocatedVoxelCount(canonicalScene);
	std::cout << "Scenes saved." << std::endl;
	return true;
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::LoadScenesCompact() {
	if (!CheckDirectory()) {
		return false;
	}
	std::cout << "Loading scenes for current frame (this might take awhile)..." << std::endl;
	std::cout.flush();

	ITMSceneReconstructionEngine<TVoxelCanonical, TIndex>* reconstructionEngineCanonical =
			ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxelCanonical, TIndex>(
					ITMLibSettings::DEVICE_CPU);
	ITMSceneReconstructionEngine<TVoxelLive, TIndex>* reconstructionEngineLive =
			ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxelLive, TIndex>(
					ITMLibSettings::DEVICE_CPU);
	reconstructionEngineCanonical->ResetScene(canonicalScene);
	reconstructionEngineLive->ResetScene(liveScene);

	liveScene->LoadFromDirectoryCompact_CPU(livePath.c_str());
	canonicalScene->LoadFromDirectoryCompact_CPU(canonicalPath.c_str());
	ITMSceneStatisticsCalculator<TVoxelCanonical, TIndex> statisticsCalculator;
	this->voxelCount = statisticsCalculator.ComputeAllocatedVoxelCount(canonicalScene);
	std::cout << "Scenes loaded." << std::endl;
	delete reconstructionEngineCanonical;
	delete reconstructionEngineLive;
	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::StartSavingWarpState(unsigned int frameIx) {
	if (!fs::is_directory(path)) {
		std::cout << "The directory '" << path << "' was not found.";
		return false;
	}
	warpOFStream = std::ofstream(warpUpdatesPath.c_str(), std::ofstream::binary | std::ofstream::out);
	if (!warpOFStream)
		throw std::runtime_error("Could not open " + warpUpdatesPath.string() + " for writing ["  __FILE__  ": " +
		                         std::to_string(__LINE__) + "]");
	warpOFStream.write(reinterpret_cast<const char*>(&frameIx), sizeof(int));
	generalIterationCursor = 0;
	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::StopSavingWarpState() {
	warpOFStream.close();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveCurrentWarpState() {

	if (!warpOFStream) {
		std::cout << "Current warp-update OFStream cannot be saved to for whatever reason." << std::endl;
		return false;
	}
	warpOFStream.write(reinterpret_cast<const char* >(&this->generalIterationCursor), sizeof(unsigned int));
	const TVoxelCanonical* voxels = canonicalScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashBlocks = canonicalScene->index.GetEntries();
	int hashBlockCount = canonicalScene->index.noTotalEntries;

	for (int iHashBlock = 0; iHashBlock < hashBlockCount; iHashBlock++) {
		const ITMHashEntry& currentHashBlock = hashBlocks[iHashBlock];
		if (currentHashBlock.ptr < 0) continue;
		const TVoxelCanonical* localVoxelBlock = &(voxels[currentHashBlock.ptr * (SDF_BLOCK_SIZE3)]);
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int ixVoxelInHashBlock = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					const TVoxelCanonical& voxel = localVoxelBlock[ixVoxelInHashBlock];
					warpOFStream.write(reinterpret_cast<const char* >(&voxel.warp_t), sizeof(Vector3f));
					warpOFStream.write(reinterpret_cast<const char* >(&voxel.warp_t_update), sizeof(Vector3f));
				}
			}
		}
	}
	std::cout << "Written warp updates for iteration " << generalIterationCursor << " to disk." << std::endl;
	generalIterationCursor++;
	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::StartLoadingWarpState() {
	if (this->voxelCount == -1) {
		std::cerr << "Hashed voxel count has not been obtained. Have the scenes been loaded successfully?" << std::endl;
		return false;
	}
	if (!fs::is_directory(this->path)) {
		std::cerr << "The directory '" << path << "' was not found.";
		return false;
	}

	warpIFStream = std::ifstream(warpUpdatesPath.c_str(), std::ios::binary | std::ios::in);
	if (!warpIFStream) {
		std::cerr << "Could not open " + warpUpdatesPath.string() + " for reading. ["  __FILE__  ": " +
		             std::to_string(__LINE__) + "]";
		return false;
	}
	int frameIx;
	warpIFStream.read(reinterpret_cast<char*>(&frameIx), sizeof(int));
	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::StartLoadingWarpState(unsigned int& frameIx) {
	if (this->voxelCount == -1) {
		std::cerr << "Hashed voxel count has not been obtained. Have the scenes been loaded successfully?" << std::endl;
		return false;
	}
	if (!fs::is_directory(this->path)) {
		std::cerr << "The directory '" << path << "' was not found.";
		return false;
	}

	warpIFStream = std::ifstream(warpUpdatesPath.c_str(), std::ios::binary | std::ios::in);
	if (!warpIFStream) {
		std::cerr << "Could not open " + warpUpdatesPath.string() + " for reading. ["  __FILE__  ": " +
		             std::to_string(__LINE__) + "]";
		return false;
	}

	warpIFStream.read(reinterpret_cast<char*>(&frameIx), sizeof(unsigned int));

	generalIterationCursor = 0;
	return true;
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::StopLoadingWarpState() {
	warpIFStream.close();

}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::LoadCurrentWarpState() {
	if (!warpIFStream) {
		std::cout << "Attempted to read warp state with IFStream being in a bad state."
				" Was 'StartLoadingWarpState()' called?" << std::endl;
		return false;
	}
	if (!warpIFStream.read(reinterpret_cast<char*>(&generalIterationCursor), sizeof(unsigned int))) {
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}

	TVoxelCanonical* voxels = canonicalScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashBlocks = canonicalScene->index.GetEntries();
	int hashBlockCount = canonicalScene->index.noTotalEntries;

	int voxelCount = 0;
	float maxWarpUpdateLength = 0;
	for (int iHashBlock = 0; iHashBlock < hashBlockCount; iHashBlock++) {
		const ITMHashEntry& currentHashBlock = hashBlocks[iHashBlock];
		if (currentHashBlock.ptr < 0) continue;
		TVoxelCanonical* localVoxelBlock = &(voxels[currentHashBlock.ptr * (SDF_BLOCK_SIZE3)]);
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int ixVoxelInHashBlock = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxelCanonical& voxel = localVoxelBlock[ixVoxelInHashBlock];
					warpIFStream.read(reinterpret_cast<char*>(&voxel.warp_t), sizeof(Vector3f));
					warpIFStream.read(reinterpret_cast<char*>(&voxel.warp_t_update), sizeof(Vector3f));
					float warpUpdateLength = ORUtils::length(voxel.warp_t_update);
					if (warpUpdateLength > maxWarpUpdateLength) {
						maxWarpUpdateLength = warpUpdateLength;
					}
					voxelCount++;
				}
			}
		}
	}
	std::cout << "Iteration " << generalIterationCursor << ". Max warp update: " << maxWarpUpdateLength << std::endl;
	this->voxelCount = voxelCount;
	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::LoadPreviousWarpState() {
	if (generalIterationCursor < 1) {
		return false;
	}

	warpIFStream.seekg(-2 * (voxelCount * 2 * sizeof(Vector3f) + sizeof(unsigned int)), std::ios::cur);

	if (!warpIFStream.read(reinterpret_cast<char*>(&generalIterationCursor), sizeof(unsigned int))) {
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}

	TVoxelCanonical* voxels = canonicalScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashBlocks = canonicalScene->index.GetEntries();
	int hashBlockCount = canonicalScene->index.noTotalEntries;

	for (int iHashBlock = 0; iHashBlock < hashBlockCount; iHashBlock++) {
		const ITMHashEntry& currentHashBlock = hashBlocks[iHashBlock];
		if (currentHashBlock.ptr < 0) continue;
		TVoxelCanonical* localVoxelBlock = &(voxels[currentHashBlock.ptr * (SDF_BLOCK_SIZE3)]);
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int ixVoxelInHashBlock = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxelCanonical& voxel = localVoxelBlock[ixVoxelInHashBlock];
					warpIFStream.read(reinterpret_cast<char*>(&voxel.warp_t), sizeof(Vector3f));
					warpIFStream.read(reinterpret_cast<char*>(&voxel.warp_t_update), sizeof(Vector3f));
				}
			}
		}
	}
	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::IsLoadingWarpState() {
	return this->warpIFStream.is_open();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::BufferNextWarpState() {
	if (!warpIFStream) {
		std::cout << "Attempted to read warp state with IFStream being in a bad state."
				" Was 'StartLoadingWarpState()' called?" << std::endl;
		return false;
	}
	if (voxelCount == -1) {
		std::cout << "Attempted to read warp state without knowing voxel count apriori."
				" Were scenes loaded successfully?" << std::endl;
		return false;
	}
	//read in the number of the current update.
	if (!warpIFStream.read(reinterpret_cast<char*>(&generalIterationCursor), sizeof(unsigned int))) {
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}

	if (warpBuffer == NULL) {
		//allocate warp buffer
		warpBuffer = new Vector3f[voxelCount * 2];
	}
	warpIFStream.read(reinterpret_cast<char*>(warpBuffer), sizeof(Vector3f) * voxelCount * 2);

	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::BufferPreviousWarpState() {
	if (generalIterationCursor < 1) {
		return false;
	}

	warpIFStream.seekg(-2 * (voxelCount * 2 * sizeof(Vector3f) + sizeof(unsigned int)), std::ios::cur);
	//read in the number of the current update.
	if (!warpIFStream.read(reinterpret_cast<char*>(&generalIterationCursor), sizeof(unsigned int))) {
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}

	warpIFStream.read(reinterpret_cast<char*>(warpBuffer), sizeof(Vector3f) * voxelCount * 2);

	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::~ITMSceneLogger() {
	this->StopLoadingWarpState();
	delete[] warpBuffer;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::CopyWarpBuffer(float* warpDestination,
                                                                         float* warpUpdateDestination, int& iUpdate) {
	if (!warpBuffer) return false;
	memcpy(reinterpret_cast<void*>(warpDestination), reinterpret_cast<void*>(warpBuffer),
	       sizeof(Vector3f) * voxelCount);
	memcpy(reinterpret_cast<void*>(warpDestination), reinterpret_cast<void*>(warpBuffer + voxelCount),
	       sizeof(Vector3f) * voxelCount);
	iUpdate = this->generalIterationCursor;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::CopyWarpAt(int index, float voxelWarpDestination[3]) const {
	memcpy(reinterpret_cast<void*>(voxelWarpDestination), reinterpret_cast<void*>(warpBuffer + index * 2),
	       sizeof(Vector3f));
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::CopyWarpAt(int index, float* voxelWarpDestination,
                                                                     float* voxelUpdateDestination) const {
	memcpy(reinterpret_cast<void*>(voxelWarpDestination), reinterpret_cast<void*>(warpBuffer + index * 2),
	       sizeof(Vector3f));
	memcpy(reinterpret_cast<void*>(voxelUpdateDestination), reinterpret_cast<void*>(warpBuffer + index * 2 + 1),
	       sizeof(Vector3f));
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const float* ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpAt(int index) const {
	return reinterpret_cast<const float*>(warpBuffer + index * 2);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const float* ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::UpdateAt(int index) const {
	return reinterpret_cast<const float*>(warpBuffer + index * 2 + 1);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
int ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetVoxelCount() const {
	if (voxelCount == -1) return 0;
	return voxelCount;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::BufferCurrentWarpState(void* externalBuffer) {
	if (!warpIFStream) {
		std::cout << "Attempted to read warp state with IFStream being in a bad state."
				" Was 'StartLoadingWarpState()' called?" << std::endl;
		return false;
	}
	if (voxelCount == -1) {
		std::cout << "Attempted to read warp state without knowing voxel count apriori."
				" Were scenes loaded successfully?" << std::endl;
		return false;
	}
	//read in the number of the current update.
	if (warpIFStream.peek() == EOF) {
		std::cout << "At end of warp file." << std::endl;
		return false;
	}

	if (!warpIFStream.read(reinterpret_cast<char*>(&generalIterationCursor), sizeof(unsigned int))) {
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}
	warpIFStream.read(reinterpret_cast<char*>(externalBuffer), sizeof(Vector3f) * voxelCount * 2);
	std::cout << "Read warp state for iteration " << generalIterationCursor << std::endl;
	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::BufferPreviousWarpState(void* externalBuffer) {
	if (generalIterationCursor < 1) {
		return false;
	}
	if (warpIFStream.eof()) {
		warpIFStream.clear();
	}
	warpIFStream.seekg(-2 * (voxelCount * 2 * sizeof(Vector3f) + sizeof(unsigned int)), std::ios::cur);
	//read in the number of the current update.
	if (!warpIFStream.read(reinterpret_cast<char*>(&generalIterationCursor), sizeof(unsigned int))) {
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}

	warpIFStream.read(reinterpret_cast<char*>(externalBuffer), sizeof(Vector3f) * voxelCount * 2);
	std::cout << "Read warp state for iteration " << generalIterationCursor << std::endl;

	return !(warpIFStream.bad() || warpIFStream.fail());
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::BufferWarpStateAt(void* externalBuffer,
                                                                            unsigned int iterationIndex) {
	size_t headerSize = sizeof(int);
	warpIFStream.seekg(headerSize + iterationIndex * (voxelCount * 2 * sizeof(Vector3f) + sizeof(unsigned int)),
	                   std::ios::beg);
	if (warpIFStream.eof()) {
		warpIFStream.clear();
	}
	//read in the number of the current update.
	if (!warpIFStream.read(reinterpret_cast<char*>(&generalIterationCursor), sizeof(unsigned int))) {
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}

	generalIterationCursor = iterationIndex;
	warpIFStream.read(reinterpret_cast<char*>(externalBuffer), sizeof(Vector3f) * voxelCount * 2);
	std::cout << "Read warp state for iteration " << generalIterationCursor << std::endl;

	return !(warpIFStream.bad() || warpIFStream.fail());
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetScenesLoaded() const {
	return voxelCount != -1;
}

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
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SetScenes(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {
	this->liveScene = liveScene;
	this->canonicalScene = canonicalScene;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::PrintHighlights() {
	std::cout << "*** Highlights ***" << std::endl;
	std::cout << this->highlights << std::endl;
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
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::CheckDirectory() {
	if (!liveScene || !canonicalScene) {
		std::cerr << "At least one of the two scenes, canonical/live, was not set to an actual scene. "
		          << __FILE__ << ":" << __LINE__ << std::endl;
	}
	if (!fs::is_directory(this->path)) {
		std::cout << "The directory '" << path << "' was not found.";
		return false;
	}
	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITM3DNestedMapOfArrays<ITMHighlightIterationInfo>
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetHighlights() const {
	return ITM3DNestedMapOfArrays<ITMHighlightIterationInfo>(highlights);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetPath() const {
	return this->path.string();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
unsigned int ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetGeneralIterationCursor() const {
	return this->generalIterationCursor;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
unsigned int ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetInterestIterationCursor() const {
	return this->interestIterationCursor;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::ClearHighlights() {
	this->highlights.Clear();
}

/**
 * \brief Set the path to save everything to or load everything from. If a directory doesn't exist at that path, one
 * will be created.
 * \param path the new path.
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SetPath(std::string path) {
	this->path = path;
	if (!fs::create_directories(this->path) && !fs::is_directory(this->path)) {
		DIEWITHEXCEPTION(std::string("Could not create the directory '") + path + "'. Exiting.["
				                 __FILE__
				                 ": " + std::to_string(__LINE__) + "]");
	}
	this->canonicalPath = this->path / "canonical";
	this->livePath = this->path / "live";
	this->warpUpdatesPath = this->path / ("warp_updates" + binaryFileExtension);

	this->highlightsBinaryPath = this->path / ("highlights" + binaryFileExtension);
	this->highlightsTextPath = this->path / ("highlights" + textFileExtension);
}



