//  ================================================================
//  Created by Gregory Kramida on 3/28/18.
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
#include <regex>

//local
#include "ITMWarpSceneLogger.h"
#include "../Analytics/ITMSceneStatisticsCalculator.h"
#include "../ITMLibSettings.h"
#include "../../Engines/Reconstruction/ITMDynamicSceneReconstructionEngineFactory.h"


using namespace ITMLib;

// =====================================================================================================================
// =============================================== WARP SCENE WRAPPER INNER CLASS (CANONICAL SCENE & ITS SLICES)========
// =====================================================================================================================

// region ======================================== STATIC CONSTANTS ====================================================

template<typename TVoxel, typename TIndex>
const size_t ITMWarpSceneLogger<TVoxel, TIndex>::warpByteSize = sizeof(TVoxel::warp);

template<typename TVoxel, typename TIndex>
const size_t ITMWarpSceneLogger<TVoxel, TIndex>::warpFloatSize =
		ITMWarpSceneLogger<TVoxel, TIndex>::warpByteSize / sizeof(float);

//TODO: bytesize for gradient of live frame
template<typename TVoxel, typename TIndex>
const size_t ITMWarpSceneLogger<TVoxel, TIndex>::updateByteSize = 0;

template<typename TVoxel, typename TIndex>
const size_t ITMWarpSceneLogger<TVoxel, TIndex>::updateFloatSize =
		ITMWarpSceneLogger<TVoxel, TIndex>::updateByteSize / sizeof(float);

template<typename TVoxel, typename TIndex>
const size_t ITMWarpSceneLogger<TVoxel, TIndex>::warpAndUpdateByteSize =
		ITMWarpSceneLogger<TVoxel, TIndex>::warpByteSize + ITMWarpSceneLogger<TVoxel, TIndex>::updateByteSize;

template<typename TVoxel, typename TIndex>
const size_t ITMWarpSceneLogger<TVoxel, TIndex>::warpAndUpdateFloatSize =
		ITMWarpSceneLogger<TVoxel, TIndex>::warpAndUpdateByteSize / sizeof(float);;

template<typename TVoxel, typename TIndex>
const std::string ITMWarpSceneLogger<TVoxel, TIndex>::fullSceneSliceIdentifier = "full_scene";

template<typename TVoxel, typename TIndex>
const std::string ITMWarpSceneLogger<TVoxel, TIndex>::binaryFileExtension = ".dat";
template<typename TVoxel, typename TIndex>
const std::string ITMWarpSceneLogger<TVoxel, TIndex>::textFileExtension = ".txt";
// private
template<typename TVoxel, typename TIndex>
const std::string ITMWarpSceneLogger<TVoxel, TIndex>::highlightFilterInfoFilename = "highlight_filter_info.txt";

template<typename TVoxel, typename TIndex>
const std::string ITMWarpSceneLogger<TVoxel, TIndex>::warpUpdatesFilename = "warp_updates";

template<typename TVoxel, typename TIndex>
const std::string ITMWarpSceneLogger<TVoxel, TIndex>::minRecurrenceHighlightFilterName = "min_recurrence_count_filter:";

template<typename TVoxel, typename TIndex>
const std::string ITMWarpSceneLogger<TVoxel, TIndex>::canonicalName = "canonical";

template<typename TVoxel, typename TIndex>
const std::string ITMWarpSceneLogger<TVoxel, TIndex>::sliceFolderPrefix = "slice_";

template<typename TVoxel, typename TIndex>
const std::string ITMWarpSceneLogger<TVoxel, TIndex>::sliceScenePrefix = "scene_";

template<typename TVoxel, typename TIndex>
const std::string ITMWarpSceneLogger<TVoxel, TIndex>::continuousHighlightsPostfix = "continuous";

// endregion
// region ======================================== STATIC METHODS : PATH GENERATION ====================================

template<typename TVoxel, typename TIndex>
std::string ITMWarpSceneLogger<TVoxel, TIndex>::GenerateSliceStringIdentifier(
		const Vector3i& minPoint, const Vector3i& maxPoint) {
	return int_to_padded_string(minPoint.x, 3)
	       + "_" + int_to_padded_string(minPoint.y, 3)
	       + "_" + int_to_padded_string(minPoint.z, 3)
	       + "_" + int_to_padded_string(maxPoint.x, 3)
	       + "_" + int_to_padded_string(maxPoint.y, 3)
	       + "_" + int_to_padded_string(maxPoint.z, 3);
};


template<typename TVoxel, typename TIndex>
void ITMWarpSceneLogger<TVoxel, TIndex>::ExtractMinMaxFromSliceStringIdentifier(
		const std::string& stringContainingIdentifier,
		Vector3i& minPoint, Vector3i& maxPoint) {
	const int coordCount = 6;
	int coordinates[coordCount];
	std::regex numberGroupRegex("(-)?(?:0{1,2})?(\\d+)");
	std::sregex_iterator iter(stringContainingIdentifier.begin(),stringContainingIdentifier.end(), numberGroupRegex);
	std::sregex_iterator iterEnd; int iCoord;
	for (iCoord = 0; iter != iterEnd; iter++, iCoord++){
		std::smatch match = *iter;
		std::string numberStr = match[2].str();
		int coordinate = std::stoi(numberStr);
		if(match[1].matched){
			coordinate = -coordinate; // got minus, flip sign
		}
		coordinates[iCoord] = coordinate;
	}
	memcpy(minPoint.values,coordinates,sizeof(int)*3);
	memcpy(maxPoint.values,&coordinates[3],sizeof(int)*3);
}

template<typename TVoxel, typename TIndex>
boost::filesystem::path
ITMWarpSceneLogger<TVoxel, TIndex>::GenerateSliceFolderPath(const fs::path& fullScenePath, const Vector3i& minPoint,
                                                            const Vector3i& maxPoint) {
	return fullScenePath /
	       (sliceFolderPrefix
	        + GenerateSliceStringIdentifier(minPoint, maxPoint));
}

template<typename TVoxel, typename TIndex>
boost::filesystem::path
ITMWarpSceneLogger<TVoxel, TIndex>::GenerateSliceFolderPath(const fs::path& fullScenePath,
                                                            const std::string& sliceIdentifier) {
	return fullScenePath / (sliceFolderPrefix + sliceIdentifier);
};

template<typename TVoxel, typename TIndex>
std::string ITMWarpSceneLogger<TVoxel, TIndex>::GenerateSliceSceneFilename_UpToPostfix(const fs::path& fullScenePath,
                                                                                       const Vector3i& minPoint,
                                                                                       const Vector3i& maxPoint) {
	return (GenerateSliceFolderPath(fullScenePath, minPoint, maxPoint) / sliceScenePrefix).string();
}

template<typename TVoxel, typename TIndex>
std::string ITMWarpSceneLogger<TVoxel, TIndex>::GenerateSliceSceneFilename_UpToPostfix(const fs::path& fullScenePath,
                                                                                       const std::string& sliceIdentifier) {
	return (GenerateSliceFolderPath(fullScenePath, sliceIdentifier) / sliceScenePrefix).string();
}

template<typename TVoxel, typename TIndex>
std::string ITMWarpSceneLogger<TVoxel, TIndex>::GenerateSliceSceneFilename_Full(const fs::path& fullScenePath,
                                                                                const Vector3i& minPoint,
                                                                                const Vector3i& maxPoint) {
	return GenerateSliceSceneFilename_UpToPostfix(fullScenePath, minPoint, maxPoint)
	       + ITMScene<TVoxel, TIndex>::compactFilePostfixAndExtension;
}

template<typename TVoxel, typename TIndex>
std::string
ITMWarpSceneLogger<TVoxel, TIndex>::GenerateSliceSceneFilename_Full(const fs::path& fullScenePath,
                                                                    const std::string& sliceIdentifier) {
	return GenerateSliceSceneFilename_UpToPostfix(fullScenePath, sliceIdentifier)
	       + ITMScene<TVoxel, TIndex>::compactFilePostfixAndExtension;
}

template<typename TVoxel, typename TIndex>
std::string
ITMWarpSceneLogger<TVoxel, TIndex>::GenerateSliceWarpFilename(const fs::path& rootScenePath, const Vector3i& minPoint,
                                                              const Vector3i& maxPoint) {
	return (GenerateSliceFolderPath(rootScenePath, minPoint, maxPoint) /
	        (warpUpdatesFilename + binaryFileExtension)).string();
};

template<typename TVoxel, typename TIndex>
std::string ITMWarpSceneLogger<TVoxel, TIndex>::GenerateSliceWarpFilename(const fs::path& rootScenePath,
                                                                          const std::string& sliceIdentifier) {
	return (GenerateSliceFolderPath(rootScenePath, sliceIdentifier) /
	        (warpUpdatesFilename + binaryFileExtension)).string();
};

// endregion
// region ======================================== CONSTRUCTORS & DESTRUCTORS ==========================================
/**
 * \brief Build a full-scene version of the warp scene logger
 * \param scene an externally-managed scene
 * \param path path to root location of the files for the scene
 */
template<typename TVoxel, typename TIndex>
ITMWarpSceneLogger<TVoxel, TIndex>::ITMWarpSceneLogger(ITMScene<TVoxel, TIndex>* scene, boost::filesystem::path path):
		scene(scene),
		path(""),
		isSlice(false),
		minimum(0),
		maximum(0),
		highlights("Hash ID", "Local voxel ix", "Frame", ""),
		sliceIdentifier(fullSceneSliceIdentifier){

	if(scene == nullptr){
		DIEWITHEXCEPTION_REPORTLOCATION("Input scene cannot be null.");
	}
	SetPath(path);
}

/**
 * \brief Build a slice version of the warp scene logger. Manages its own scene data structure internally.
 * \param minPoint minimum coordinates (corner) of the slice
 * \param maxPoint maximum coordinates (corner) of the slice
 * \param fullScenePath to the root location of the files for the full scene encompassing the slice
 */
template<typename TVoxel, typename TIndex>
ITMWarpSceneLogger<TVoxel, TIndex>::ITMWarpSceneLogger(const Vector3i& minPoint, const Vector3i& maxPoint,
                                                       boost::filesystem::path fullScenePath):
		scene(nullptr),
		path(""),
		isSlice(true),
		minimum(minPoint),
		maximum(maxPoint),
		highlights("Hash ID", "Local voxel ix", "Frame", ""),
		sliceIdentifier(GenerateSliceStringIdentifier(minPoint,maxPoint)){

	ITMLibSettings* settings = new ITMLibSettings;
	MemoryDeviceType memoryType =
			settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
	this->scene = new ITMScene<TVoxel, TIndex>(&settings->sceneParams,
	                                           settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                           memoryType);
	delete settings;

	SetPath(fullScenePath);
}

template<typename TVoxel, typename TIndex>
ITMWarpSceneLogger<TVoxel, TIndex>::~ITMWarpSceneLogger() {
	if (isSlice) {
		delete scene;
	}
}

// endregion
// region ======================================== GETTERS =============================================================

template<typename TVoxel, typename TIndex>
unsigned int ITMWarpSceneLogger<TVoxel, TIndex>::GetIterationCursor() const {
	return iterationCursor;
}


template<typename TVoxel, typename TIndex>
bool ITMWarpSceneLogger<TVoxel, TIndex>::SetIterationCursor(unsigned int iterationIndex) {
	size_t headerSize = sizeof(int);
	warpIFStream.seekg(headerSize + iterationIndex * (voxelCount * warpAndUpdateByteSize + sizeof(iterationCursor)),
	                   std::ios::beg);
	if (warpIFStream.eof()) {
		warpIFStream.clear();
	}
	//assume that index matches the file
	iterationCursor = iterationIndex;
}

template<typename TVoxel, typename TIndex>
bool ITMWarpSceneLogger<TVoxel, TIndex>::Empty() const {
	return scene == nullptr;
}

template<typename TVoxel, typename TIndex>
bool ITMWarpSceneLogger<TVoxel, TIndex>::Loaded() const {
	return voxelCount != -1;
}

template<typename TVoxel, typename TIndex>
std::string ITMWarpSceneLogger<TVoxel, TIndex>::GetSliceIdentifier() const {
	if (!isSlice) {
		return fullSceneSliceIdentifier;
	}
	return ITMWarpSceneLogger<TVoxel, TIndex>::GenerateSliceStringIdentifier(this->minimum, this->maximum);
}

template<typename TVoxel, typename TIndex>
const ITMScene<TVoxel, TIndex>* ITMWarpSceneLogger<TVoxel, TIndex>::GetScene() const {
	return this->scene;
}
// endregion
// region ======================================== LOAD / SAVE SCENE ===================================================

template<typename TVoxel, typename TIndex>
void ITMWarpSceneLogger<TVoxel, TIndex>::Load() {
	scene->LoadFromDirectory(scenePath.c_str());
	ITMSceneStatisticsCalculator<TVoxel, TIndex> statisticsCalculator;
	voxelCount = statisticsCalculator.ComputeAllocatedVoxelCount(scene);
}

template<typename TVoxel, typename TIndex>
void ITMWarpSceneLogger<TVoxel, TIndex>::Save() {
	scene->SaveToDirectory(scenePath.c_str());
	ITMSceneStatisticsCalculator<TVoxel, TIndex> statisticsCalculator;
	this->voxelCount = statisticsCalculator.ComputeAllocatedVoxelCount(scene);
}

template<typename TVoxel, typename TIndex>
void ITMWarpSceneLogger<TVoxel, TIndex>::SaveCompact() {
	scene->SaveToDirectoryCompact_CPU(scenePath.c_str());
	ITMSceneStatisticsCalculator<TVoxel, TIndex> statisticsCalculator;
	this->voxelCount = statisticsCalculator.ComputeAllocatedVoxelCount(scene);
}

template<typename TVoxel, typename TIndex>
void ITMWarpSceneLogger<TVoxel, TIndex>::LoadCompact() {
	ITMSceneManipulationEngine_CPU<TVoxel,TIndex>::ResetScene(scene);
	scene->LoadFromDirectoryCompact_CPU(scenePath.c_str());
	ITMSceneStatisticsCalculator<TVoxel, TIndex> statisticsCalculator;
	this->voxelCount = statisticsCalculator.ComputeAllocatedVoxelCount(scene);
}
//endregion
// region ======================================== LOAD / SAVE HIGHLIGHTS ==============================================

template<typename TVoxel, typename TIndex>
bool ITMWarpSceneLogger<TVoxel, TIndex>::SaveHighlights(std::string filePostfix) {
	fs::path rootPath = fs::path(warpPath).parent_path();
	highlightsBinaryPath = rootPath / ("highlights" + filePostfix + binaryFileExtension);
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
	return false;
}


template<typename TVoxel, typename TIndex>
bool ITMWarpSceneLogger<TVoxel, TIndex>::LoadHighlights(bool applyFilters, std::string filePostfix) {
	fs::path rootPath = fs::path(warpPath).parent_path();
	highlightsBinaryPath = rootPath / ("highlights" + filePostfix + binaryFileExtension);
	if (!this->highlights.LoadFromFile(highlightsBinaryPath.c_str())) {
		std::cout << "Could not load highlights from " << highlightsBinaryPath << std::endl;
		return false;
	} else {
		std::cout << "Loaded highlights from " << highlightsBinaryPath << std::endl;
	}
	if (applyFilters) {
		fs::path wouldBeFilterInfoPath = rootPath / fs::path(highlightFilterInfoFilename);
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
	return false;
}

template<typename TVoxel, typename TIndex>
void ITMWarpSceneLogger<TVoxel, TIndex>::FilterHighlights(int anomalyFrameCountMinimum) {
	fs::path rootPath = fs::path(warpPath).parent_path();
	minHighlightRecurrenceCount = std::max(minHighlightRecurrenceCount, anomalyFrameCountMinimum);
	highlights = highlights.FilterBasedOnLevel0Lengths(anomalyFrameCountMinimum);
	std::ofstream highlightFilterInfoNote((rootPath / fs::path(highlightFilterInfoFilename)).c_str(),
	                                      std::ios_base::out);
	highlightFilterInfoNote << "min_reccurence_count_filter:" << " " << anomalyFrameCountMinimum << std::endl;
	highlightFilterInfoNote.close();
}

// region ======================================== LOAD / SAVE / BUFFER WARP ===========================================

template<typename TVoxel, typename TIndex>
bool
ITMWarpSceneLogger<TVoxel, TIndex>::StartSavingWarpState() {

	//TODO remove frameIdx recording and its dependencies
	std::string rootPath = fs::path(warpPath).parent_path().string();
	if (!fs::is_directory(rootPath)) {
		std::cout << "The directory '" << rootPath << "' was not found.";
		return false;
	}
	warpOFStream = std::ofstream(warpPath.c_str(), std::ofstream::binary | std::ofstream::out);
	if (!warpOFStream)
		throw std::runtime_error("Could not open " + warpPath.string() + " for writing ["  __FILE__  ": " +
		                         std::to_string(__LINE__) + "]");
	int frameIx = 0;
	warpOFStream.write(reinterpret_cast<const char*>(&frameIx), sizeof(frameIx));
	iterationCursor = 0;
	return true;
}

template<typename TVoxel, typename TIndex>
void ITMWarpSceneLogger<TVoxel, TIndex>::StopSavingWarpState() {
	warpOFStream.close();
}

template<typename TVoxel, typename TIndex>
bool ITMWarpSceneLogger<TVoxel, TIndex>::SaveCurrentWarpState() {

	if (!warpOFStream) {
		std::cerr << "Current warp-update OFStream cannot be saved to for whatever reason." << std::endl;
		return false;
	}
	warpOFStream.write(reinterpret_cast<const char* >(&this->iterationCursor), sizeof(iterationCursor));
	const TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashBlocks = scene->index.GetEntries();
	int hashBlockCount = scene->index.noTotalEntries;

	for (int iHashBlock = 0; iHashBlock < hashBlockCount; iHashBlock++) {
		const ITMHashEntry& currentHashBlock = hashBlocks[iHashBlock];
		if (currentHashBlock.ptr < 0) continue;
		const TVoxel* localVoxelBlock = &(voxels[currentHashBlock.ptr * (SDF_BLOCK_SIZE3)]);
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int ixVoxelInHashBlock = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					const TVoxel& voxel = localVoxelBlock[ixVoxelInHashBlock];
					warpOFStream.write(reinterpret_cast<const char* >(&voxel.warp), warpByteSize);
					//warpOFStream.write(reinterpret_cast<const char* >(&voxel.warp_t_update), updateByteSize);
				}
			}
		}
	}
	std::cout << "Written warp updates for iteration " << iterationCursor << " to disk." << std::endl;
	iterationCursor++;
	return true;
}

template<typename TVoxel, typename TIndex>
bool ITMWarpSceneLogger<TVoxel, TIndex>::StartLoadingWarpState() {
	if (this->voxelCount == -1) {
		std::cerr << "Hashed voxel count has not been obtained. Have the scenes been loaded successfully?" << std::endl;
		return false;
	}
	if (!fs::is_regular_file(this->warpPath)) {
		std::cerr << "The directory '" << warpPath << "' was not found.";
		return false;
	}

	warpIFStream = std::ifstream(warpPath.c_str(), std::ios::binary | std::ios::in);
	if (!warpIFStream) {
		std::cerr << "Could not open " + warpPath.string() + " for reading. ["  __FILE__  ": " +
		             std::to_string(__LINE__) + "]";
		return false;
	}
	int frameIx;
	warpIFStream.read(reinterpret_cast<char*>(&frameIx), sizeof(frameIx));
	return true;
}

template<typename TVoxel, typename TIndex>
bool
ITMWarpSceneLogger<TVoxel, TIndex>::StartLoadingWarpState(unsigned int& frameIx) {
	if (this->voxelCount == -1) {
		std::cerr << "Hashed voxel count has not been obtained. Have the scenes been loaded successfully?" << std::endl;
		return false;
	}
	if (!fs::is_regular_file(warpPath)) {
		std::cerr << "The file '" << warpPath << "' was not found.";
		return false;
	}

	warpIFStream = std::ifstream(warpPath.c_str(), std::ios::binary | std::ios::in);
	if (!warpIFStream) {
		std::cerr << "Could not open " + warpPath.string() + " for reading. ["  __FILE__  ": " +
		             std::to_string(__LINE__) + "]";
		return false;
	}

	warpIFStream.read(reinterpret_cast<char*>(&frameIx), sizeof(frameIx));

	iterationCursor = 0;
	return true;
}


template<typename TVoxel, typename TIndex>
void ITMWarpSceneLogger<TVoxel, TIndex>::StopLoadingWarpState() {
	warpIFStream.close();
}

/**
 * \brief Transfers the warp state from the warp file to the scene, imitating the .warp and .gradient fields after
 * the current iteration.
 * \return True on success, false on failure
 */
template<typename TVoxel, typename TIndex>
bool ITMWarpSceneLogger<TVoxel, TIndex>::LoadCurrentWarpState() {
	if (!warpIFStream) {
		std::cout << "Attempted to read warp state with IFStream being in a bad state."
		             " Was 'StartLoadingWarpState()' called?" << std::endl;
		return false;
	}
	if (!warpIFStream.read(reinterpret_cast<char*>(&iterationCursor), sizeof(iterationCursor))) {
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}

	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashBlocks = scene->index.GetEntries();
	int hashBlockCount = scene->index.noTotalEntries;

	int voxelCount = 0;
	float maxWarpUpdateLength = 0;
	for (int iHashBlock = 0; iHashBlock < hashBlockCount; iHashBlock++) {
		const ITMHashEntry& currentHashBlock = hashBlocks[iHashBlock];
		if (currentHashBlock.ptr < 0) continue;
		TVoxel* localVoxelBlock = &(voxels[currentHashBlock.ptr * (SDF_BLOCK_SIZE3)]);
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int ixVoxelInHashBlock = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxel& voxel = localVoxelBlock[ixVoxelInHashBlock];
					warpIFStream.read(reinterpret_cast<char*>(&voxel.warp), warpByteSize);
					//warpIFStream.read(reinterpret_cast<char*>(&voxel.warp_t_update), updateByteSize);
//					float warpUpdateLength = ORUtils::length(voxel.warp_t_update);
//					if (warpUpdateLength > maxWarpUpdateLength) {
//						maxWarpUpdateLength = warpUpdateLength;
//					}
					voxelCount++;
				}
			}
		}
	}
	std::cout << "Iteration " << iterationCursor << ". Max warp update: " << maxWarpUpdateLength << std::endl;
	this->voxelCount = voxelCount;
	return true;
}

template<typename TVoxel, typename TIndex>
bool ITMWarpSceneLogger<TVoxel, TIndex>::LoadPreviousWarpState() {
	if (iterationCursor < 1) {
		return false;
	}

	warpIFStream.seekg(-2 * (voxelCount * warpAndUpdateByteSize + sizeof(iterationCursor)), std::ios::cur);

	if (!warpIFStream.read(reinterpret_cast<char*>(&iterationCursor), sizeof(iterationCursor))) {
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}

	TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashBlocks = scene->index.GetEntries();
	int hashBlockCount = scene->index.noTotalEntries;

	for (int iHashBlock = 0; iHashBlock < hashBlockCount; iHashBlock++) {
		const ITMHashEntry& currentHashBlock = hashBlocks[iHashBlock];
		if (currentHashBlock.ptr < 0) continue;
		TVoxel* localVoxelBlock = &(voxels[currentHashBlock.ptr * (SDF_BLOCK_SIZE3)]);
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int ixVoxelInHashBlock = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxel& voxel = localVoxelBlock[ixVoxelInHashBlock];
					warpIFStream.read(reinterpret_cast<char*>(&voxel.warp), warpByteSize);
					//warpIFStream.read(reinterpret_cast<char*>(&voxel.warp_t_update), updateByteSize);
				}
			}
		}
	}
	return true;
}

/**
 * \brief Whether or not warp state is being loaded
 * \return True if warp state ifstream is open, false otherwise
 */
template<typename TVoxel, typename TIndex>
bool ITMWarpSceneLogger<TVoxel, TIndex>::IsLoadingWarpState() {
	return this->warpIFStream.is_open();
}

template<typename TVoxel, typename TIndex>
int ITMWarpSceneLogger<TVoxel, TIndex>::GetVoxelCount() const {
	if (voxelCount == -1) return 0;
	return voxelCount;
}

template<typename TVoxel, typename TIndex>
bool
ITMWarpSceneLogger<TVoxel, TIndex>::BufferCurrentWarpState(void* externalBuffer) {
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

	if (!warpIFStream.read(reinterpret_cast<char*>(&iterationCursor), sizeof(iterationCursor))) {
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}
	warpIFStream.read(reinterpret_cast<char*>(externalBuffer), warpAndUpdateByteSize * voxelCount);
	std::cout << "Read warp state for iteration " << iterationCursor << std::endl;
	return true;
}

template<typename TVoxel, typename TIndex>
bool
ITMWarpSceneLogger<TVoxel, TIndex>::BufferPreviousWarpState(void* externalBuffer) {
	if (iterationCursor < 1) {
		return false;
	}
	if (warpIFStream.eof()) {
		warpIFStream.clear();
	}
	warpIFStream.seekg(-2 * (voxelCount * warpAndUpdateByteSize + sizeof(iterationCursor)), std::ios::cur);
	//read in the number of the current update.
	if (!warpIFStream.read(reinterpret_cast<char*>(&iterationCursor), sizeof(iterationCursor))) {
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}

	warpIFStream.read(reinterpret_cast<char*>(externalBuffer), voxelCount * warpAndUpdateByteSize);
	std::cout << "Read warp state for iteration " << iterationCursor << std::endl;

	return !(warpIFStream.bad() || warpIFStream.fail());
}

template<typename TVoxel, typename TIndex>
bool ITMWarpSceneLogger<TVoxel, TIndex>::BufferWarpStateAt(void* externalBuffer,
                                                           unsigned int iterationIndex) {
	if (!SetIterationCursor(iterationIndex)) {
		return false;
	}
	unsigned int fileIterationCursor;
	if (!warpIFStream.read(reinterpret_cast<char*>(&fileIterationCursor), sizeof(fileIterationCursor))
	    || fileIterationCursor != iterationIndex) {
		std::cerr << "Read warp state attempt failed." << std::endl;
		return false;
	}
	warpIFStream.read(reinterpret_cast<char*>(externalBuffer), voxelCount * warpAndUpdateByteSize);
	std::cout << "Read warp state for iteration " << iterationCursor << std::endl;

	return !(warpIFStream.bad() || warpIFStream.fail());
}

template<typename TVoxel, typename TIndex>
void ITMWarpSceneLogger<TVoxel, TIndex>::SetPath(boost::filesystem::path fullScenePath) {
	if (fullScenePath != "") {
		StopLoadingWarpState();
	}
	if (isSlice) {
		this->path = GenerateSliceFolderPath(fullScenePath,this->minimum,this->maximum);
	} else {
		this->path = fullScenePath;
	}
	this->scenePath = this->path / sliceScenePrefix;
	this->warpPath = (this->path / (warpUpdatesFilename + binaryFileExtension)).string();
	this->highlightsBinaryPath = this->path / ("highlights" + binaryFileExtension);
	this->highlightsTextPath = this->path / ("highlights" + textFileExtension);
}
// endregion
