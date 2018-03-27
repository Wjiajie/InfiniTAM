//  ================================================================
//  Created by Gregory Kramida on 3/22/18.
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
#pragma once

//ITMLib
#include "ITMSceneLogger.h"
#include "../ITMLibSettings.h"
#include "../../Objects/Scene/ITMSceneManipulation.h"

using namespace ITMLib;

// =====================================================================================================================
// =============================================== WARP SCENE WRAPPER INNER CLASS (CANONICAL SCENE & ITS SLICES)========
// =====================================================================================================================
// region ======================================== CONSTRUCTORS & DESTRUCTORS ==========================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::WarpSceneWrapper(bool isSlice,
		ITMScene<TVoxelCanonical, TIndex>* scene, fs::path path) :
		isSlice(isSlice),
		scene(scene),
		path(path) {}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::~WarpSceneWrapper() {
	delete[] warpBuffer;
}

// endregion
// region ======================================== GETTERS =============================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
unsigned int ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::GetIterationCursor() const {
	return generalIterationCursor;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::Empty() const {
	return scene == nullptr;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::Loaded() const {
	return voxelCount != -1;
}
// endregion
// region ======================================== LOAD / SAVE SCENE ===================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::Load() {
	scene->LoadFromDirectory(path.c_str());
	ITMSceneStatisticsCalculator<TVoxelCanonical, TIndex> statisticsCalculator;
	voxelCount = statisticsCalculator.ComputeAllocatedVoxelCount(scene);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::Save() {
	scene->SaveToDirectory(path.string());
	ITMSceneStatisticsCalculator<TVoxelCanonical, TIndex> statisticsCalculator;
	this->voxelCount = statisticsCalculator.ComputeAllocatedVoxelCount(scene);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::SaveCompact() {
	scene->SaveToDirectoryCompact_CPU(path.string());
	ITMSceneStatisticsCalculator<TVoxelCanonical, TIndex> statisticsCalculator;
	this->voxelCount = statisticsCalculator.ComputeAllocatedVoxelCount(scene);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::LoadCompact() {
	ITMSceneReconstructionEngine<TVoxelCanonical, TIndex>* reconstructionEngine =
			ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxelCanonical, TIndex>(
					ITMLibSettings::DEVICE_CPU);
	reconstructionEngine->ResetScene(scene);
	delete reconstructionEngine;
	scene->LoadFromDirectoryCompact_CPU(path.c_str());
	ITMSceneStatisticsCalculator<TVoxelCanonical, TIndex> statisticsCalculator;
	this->voxelCount = statisticsCalculator.ComputeAllocatedVoxelCount(scene);

}
//endregion
// region ======================================== LOAD / SAVE / BUFFER WARP ===========================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::StartSavingWarpState(unsigned int frameIx) {
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
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::StopSavingWarpState() {
	warpOFStream.close();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::SaveCurrentWarpState() {

	if (!warpOFStream) {
		std::cout << "Current warp-update OFStream cannot be saved to for whatever reason." << std::endl;
		return false;
	}
	warpOFStream.write(reinterpret_cast<const char* >(&this->generalIterationCursor), sizeof(unsigned int));
	const TVoxelCanonical* voxels = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashBlocks = scene->index.GetEntries();
	int hashBlockCount = scene->index.noTotalEntries;

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
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::StartLoadingWarpState() {
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
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::StartLoadingWarpState(unsigned int& frameIx) {
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
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::StopLoadingWarpState() {
	warpIFStream.close();
}

/**
 * \brief Transfers the warp state from the warp file to the scene, imitating the .warp_t and .warp_t_update fields after
 * the current iteration.
 * \return True on success, false on failure
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::LoadCurrentWarpState() {
	if (!warpIFStream) {
		std::cout << "Attempted to read warp state with IFStream being in a bad state."
				" Was 'StartLoadingWarpState()' called?" << std::endl;
		return false;
	}
	if (!warpIFStream.read(reinterpret_cast<char*>(&generalIterationCursor), sizeof(unsigned int))) {
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}

	TVoxelCanonical* voxels = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashBlocks = scene->index.GetEntries();
	int hashBlockCount = scene->index.noTotalEntries;

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
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::LoadPreviousWarpState() {
	if (generalIterationCursor < 1) {
		return false;
	}

	warpIFStream.seekg(-2 * (voxelCount * 2 * sizeof(Vector3f) + sizeof(unsigned int)), std::ios::cur);

	if (!warpIFStream.read(reinterpret_cast<char*>(&generalIterationCursor), sizeof(unsigned int))) {
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}

	TVoxelCanonical* voxels = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashBlocks = scene->index.GetEntries();
	int hashBlockCount = scene->index.noTotalEntries;

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
/**
 * \brief Whether or not warp state is being loaded
 * \return True if warp state ifstream is open, false otherwise
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::IsLoadingWarpState() {
	return this->warpIFStream.is_open();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::BufferNextWarpState() {
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
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::BufferPreviousWarpState() {
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
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::CopyWarpBuffer(float* warpDestination,
                                                                         float* warpUpdateDestination, int& iUpdate) {
	if (!warpBuffer) return false;
	memcpy(reinterpret_cast<void*>(warpDestination), reinterpret_cast<void*>(warpBuffer),
	       sizeof(Vector3f) * voxelCount);
	memcpy(reinterpret_cast<void*>(warpDestination), reinterpret_cast<void*>(warpBuffer + voxelCount),
	       sizeof(Vector3f) * voxelCount);
	iUpdate = this->generalIterationCursor;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::CopyWarpAt(int index, float voxelWarpDestination[3]) const {
	memcpy(reinterpret_cast<void*>(voxelWarpDestination), reinterpret_cast<void*>(warpBuffer + index * 2),
	       sizeof(Vector3f));
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::CopyWarpAt(int index, float* voxelWarpDestination,
                                                                     float* voxelUpdateDestination) const {
	memcpy(reinterpret_cast<void*>(voxelWarpDestination), reinterpret_cast<void*>(warpBuffer + index * 2),
	       sizeof(Vector3f));
	memcpy(reinterpret_cast<void*>(voxelUpdateDestination), reinterpret_cast<void*>(warpBuffer + index * 2 + 1),
	       sizeof(Vector3f));
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const float* ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::WarpAt(int index) const {
	return reinterpret_cast<const float*>(warpBuffer + index * 2);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const float* ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::UpdateAt(int index) const {
	return reinterpret_cast<const float*>(warpBuffer + index * 2 + 1);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
int ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::GetVoxelCount() const {
	if (voxelCount == -1) return 0;
	return voxelCount;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::BufferCurrentWarpState(void* externalBuffer) {
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
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::BufferPreviousWarpState(void* externalBuffer) {
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
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpSceneWrapper::BufferWarpStateAt(void* externalBuffer,
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
//endregion
//======================================================================================================================
//================================================= SCENE LOGGER CLASS METHODS RELATING TO SLICES ======================
//======================================================================================================================
// region ======================================== SLICE PATH GENERATION ===============================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GenerateSliceStringIdentifier(
		const Vector3i& minPoint, const Vector3i& maxPoint) {
	return padded_to_string(minPoint.x, 3)
	       + "_" + padded_to_string(minPoint.y, 3)
	       + "_" + padded_to_string(minPoint.z, 3)
	       + "_" + padded_to_string(maxPoint.x, 3)
	       + "_" + padded_to_string(maxPoint.y, 3)
	       + "_" + padded_to_string(maxPoint.z, 3);
};


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
fs::path
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GenerateSliceFolderPath(const Vector3i& minPoint,
                                                                                     const Vector3i& maxPoint) {
	return this->path.parent_path() / ("slice_" + GenerateSliceStringIdentifier(minPoint, maxPoint));
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GenerateSliceWarpFilename(
		const Vector3i& minPoint,
		const Vector3i& maxPoint) {
	return (GenerateSliceFolderPath(minPoint, maxPoint) / (warpUpdatesFilename + binaryFileExtension)).string();
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GenerateSliceSceneFilename_UpToPostfix(
		const Vector3i& minPoint,
		const Vector3i& maxPoint) {

	return (GenerateSliceFolderPath(minPoint, maxPoint) / "scene_").string();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GenerateSliceSceneFilename_Full(
		const Vector3i& minPoint,
		const Vector3i& maxPoint) {
	return GenerateSliceSceneFilename_UpToPostfix(minPoint, maxPoint)
	       + ITMScene<TVoxelCanonical, TVoxelLive>::compactFilePostfixAndExtension;
}

//endregion
// region ======================================== SAVING SLICE AND SLICE WARP =========================================

//Assumes we have a scene loaded, as well as a warp file available.
/**
 * \brief Save a slice of the canonical scene with the specified extrema (overwrites if files exist already on disk)
 * \param extremum1 the first of the two points defining the slice bounds
 * \param extremum2 the second of the two points defining the slice bounds
 * \param frameIndex index of the frame to write to the saved warp file
 * \return true on success, false on failure
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveSlice(const Vector3i& extremum1,
                                                                                      const Vector3i& extremum2,
                                                                                      unsigned int frameIndex) {

	Vector3i minPoint, maxPoint;
	MinMaxFromExtrema(minPoint, maxPoint, extremum1, extremum2);
	ITMLibSettings* settings = new ITMLibSettings();
	ITMScene<TVoxelCanonical, TIndex>* slice = new ITMScene<TVoxelCanonical, TIndex>(canonicalScene.scene->sceneParams,
	                                                                                 settings->swappingMode ==
	                                                                                 ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                                                 settings->GetMemoryType());
	if (!CopySceneSlice_CPU(slice, canonicalScene.scene, minPoint, maxPoint)) {
		return false;
	}
	fs::path outputPath = GenerateSliceFolderPath(minPoint, maxPoint);
	if (fs::exists(outputPath)) {
		fs::remove(outputPath);//overwrite
	}
	fs::create_directories(outputPath);
	std::string sceneOutputPath = GenerateSliceSceneFilename_UpToPostfix(minPoint, maxPoint);
	slice->SaveToDirectoryCompact_CPU(sceneOutputPath);

	SaveWarpSlice(minPoint, maxPoint, frameIndex);
	delete settings;
	return true;
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveWarpSlice(const Vector3i& minPoint,
                                                                           const Vector3i& maxPoint,
                                                                           unsigned int frameIndex) {

	int totalHashEntryCount = canonicalScene.scene->index.noTotalEntries;
	TVoxelCanonical* voxels = canonicalScene.scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashTable = canonicalScene.scene->index.GetEntries();

	// to restore to same file position later
	unsigned int currentGeneralWarpCursor = canonicalScene.generalIterationCursor;

	bool wasLoadingWarpState = canonicalScene.warpIFStream.is_open();
	StopLoadingWarpState();

	StartLoadingWarpState();

	std::string sliceWarpOfstreamPath = GenerateSliceWarpFilename(minPoint, maxPoint);

	std::ofstream sliceWarpOfstream(sliceWarpOfstreamPath.c_str(), std::ofstream::binary | std::ofstream::out);
	unsigned int sliceIterationCursor = 0;
	if (!sliceWarpOfstream)
		throw std::runtime_error("Could not open " + sliceWarpOfstreamPath + " for writing. ["  __FILE__  ": " +
		                         std::to_string(__LINE__) + "]");
	canonicalScene.warpOFStream.write(reinterpret_cast<const char*>(&frameIndex), sizeof(int));

	while (LoadCurrentWarpState()) {
		canonicalScene.warpOFStream.write(reinterpret_cast<const char* >(&sliceIterationCursor), sizeof(unsigned int));
		for (int hash = 0; hash < totalHashEntryCount; hash++) {
			const ITMHashEntry& hashEntry = hashTable[hash];
			if (hashEntry.ptr < 0) continue;

			//position of the current entry in 3D space (in voxel units)
			Vector3i hashBlockPositionVoxels = hashEntry.pos.toInt() * SDF_BLOCK_SIZE;
			TVoxelCanonical* localVoxelBlock = &(voxels[hashEntry.ptr * (SDF_BLOCK_SIZE3)]);

			int zRangeStart, zRangeEnd, yRangeStart, yRangeEnd, xRangeStart, xRangeEnd;
			if (IsHashBlockFullyInRange(hashBlockPositionVoxels, minPoint, maxPoint)) {
				//we can safely copy the whole block
				xRangeStart = yRangeStart = zRangeStart = 0;
				xRangeEnd = yRangeEnd = zRangeEnd = SDF_BLOCK_SIZE;
			} else if (IsHashBlockPartiallyInRange(hashBlockPositionVoxels, minPoint, maxPoint)) {
				//only a portion of the block spans the slice range, figure out what it is
				ComputeCopyRanges(xRangeStart, xRangeEnd, yRangeStart, yRangeEnd, zRangeStart, zRangeEnd,
				                  hashBlockPositionVoxels, minPoint, maxPoint);
			} else {
				//no voxels in the block are within range, skip
				continue;
			}
			for (int z = zRangeStart; z < zRangeEnd; z++) {
				for (int y = yRangeStart; y < yRangeEnd; y++) {
					for (int x = xRangeStart; x < xRangeEnd; x++) {
						int ixVoxelInHashBlock = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
						const TVoxelCanonical& voxel = localVoxelBlock[ixVoxelInHashBlock];
						canonicalScene.warpOFStream.write(reinterpret_cast<const char* >(&voxel.warp_t), sizeof(Vector3f));
						canonicalScene.warpOFStream.write(reinterpret_cast<const char* >(&voxel.warp_t_update), sizeof(Vector3f));
					}
				}
			}
		}
		sliceIterationCursor++;
	}
	sliceWarpOfstream.close();
	StopLoadingWarpState();
	if (wasLoadingWarpState) {
		// ** restore previous warp-loading state **
		StartLoadingWarpState();
		size_t headerSize = sizeof(int);
		canonicalScene.warpIFStream.seekg(
				headerSize + currentGeneralWarpCursor * (canonicalScene.voxelCount * 2
				                                         * sizeof(Vector3f) + sizeof(unsigned int)),
				std::ios::beg);
		if (canonicalScene.warpIFStream.eof()) {
			canonicalScene.warpIFStream.clear();
		}
		canonicalScene.generalIterationCursor = currentGeneralWarpCursor;
	}
}
//endregion

/**
 * \brief Verify whether the slice specified by the bounds exists (by looking at names of folder & files within)
 * \param extremum1 the first of the two points defining the slice bounds
 * \param extremum2 the second of the two points defining the slice bounds
 * \return true if all files corresponding to the slice exist, false otherwise.
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::CanonicalSceneSliceExists(const Vector3i& extremum1,
                                                                                            const Vector3i& extremum2) {
	fs::path sliceFolderPath = GenerateSliceFolderPath(extremum1, extremum2);
	fs::path sliceScenePath = GenerateSliceSceneFilename_Full(extremum1, extremum2);
	fs::path sliceWarpPath = GenerateSliceWarpFilename(extremum1, extremum2);
	return fs::is_directory(sliceFolderPath) && fs::is_regular_file(sliceScenePath) &&
	       fs::is_regular_file(sliceWarpPath);
}
