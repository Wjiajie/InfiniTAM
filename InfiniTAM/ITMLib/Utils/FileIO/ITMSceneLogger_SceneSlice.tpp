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

//stdlib
#include <algorithm>

//ITMLib
#include "ITMSceneLogger.h"
#include "../ITMLibSettings.h"
#include "../../Objects/Scene/ITMSceneManipulation.h"

using namespace ITMLib;

//======================================================================================================================
//================================================= SCENE LOGGER CLASS CONSTANTS & METHODS RELATING TO SLICES ==========
//======================================================================================================================
// region ======================================== SLICE-RELATED CONSTANTS =============================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::sliceFolderPrefix = "slice_";

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::sliceScenePrefix = "scene_";

// endregion
// region ======================================== SLICE PATH GENERATION ===============================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
fs::path
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GenerateSliceFolderPath(const Vector3i& minPoint,
                                                                             const Vector3i& maxPoint) const {
	return this->path /
	       (sliceFolderPrefix
	        + ITMWarpSceneLogger<TVoxelCanonical, TIndex>::GenerateSliceStringIdentifier(minPoint, maxPoint));
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
fs::path
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GenerateSliceFolderPath(const std::string& sliceIdentifier) const {
	return this->path / (sliceFolderPrefix + sliceIdentifier);
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GenerateSliceSceneFilename_UpToPostfix(
		const Vector3i& minPoint, const Vector3i& maxPoint) const {
	return (GenerateSliceFolderPath(minPoint, maxPoint) / sliceScenePrefix).string();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GenerateSliceSceneFilename_Full(
		const Vector3i& minPoint, const Vector3i& maxPoint) const {
	return GenerateSliceSceneFilename_UpToPostfix(minPoint, maxPoint)
	       + ITMScene<TVoxelCanonical, TVoxelLive>::compactFilePostfixAndExtension;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GenerateSliceSceneFilename_Full(
		const std::string& sliceIdentifier) const {
	return (GenerateSliceFolderPath(sliceIdentifier) /
	        (sliceScenePrefix + ITMScene<TVoxelCanonical, TVoxelLive>::compactFilePostfixAndExtension)).string();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GenerateSliceWarpFilename(
		const Vector3i& minPoint, const Vector3i& maxPoint) const {
	return (GenerateSliceFolderPath(minPoint, maxPoint) / (warpUpdatesFilename + binaryFileExtension)).string();
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GenerateSliceWarpFilename(
		const std::string& sliceIdentifier) const {
	return (GenerateSliceFolderPath(sliceIdentifier) / (warpUpdatesFilename + binaryFileExtension)).string();
};

//endregion
// region ======================================== MAKING & SAVING SLICE AND SLICE WARP ================================

//Assumes we have a scene loaded, as well as a warp file available.
/**
 * \brief Make slice of the canonical scene with the specified extrema, store it locally,
 * and save it to disk, (overwrites if files exist already on disk)
 * \param destinationScene [out] the scene where to write the slice content - presumed blank, will be overwritten (not managed)
 * \param extremum1 [in] the first of the two points defining the slice bounds
 * \param extremum2 [in] the second of the two points defining the slice bounds
 * \param frameIndex [in] index of the frame to write to the saved warp file
 * \return true on success, false on failure
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::MakeSlice(const Vector3i& extremum1, const Vector3i& extremum2,
                                                                    unsigned int frameIndex) {

	Vector3i minPoint, maxPoint;
	MinMaxFromExtrema(minPoint, maxPoint, extremum1, extremum2);
	std::string sliceIdentifier =
			ITMWarpSceneLogger<TVoxelCanonical, TIndex>::GenerateSliceStringIdentifier(minPoint,maxPoint);
	if (slices.find(sliceIdentifier) != slices.end()) {
		return false;
	}
	std::string sliceScenePath = GenerateSliceSceneFilename_UpToPostfix(minPoint, maxPoint);
	std::string sliceWarpPath = GenerateSliceWarpFilename(minPoint, maxPoint);
	auto logger = new ITMWarpSceneLogger<TVoxelCanonical, TIndex> (nullptr, sliceScenePath, sliceWarpPath);
	if (!CopySceneSlice_CPU(logger->scene, fullCanonicalSceneLogger->scene, minPoint, maxPoint)) {
		return false;
	}
	fs::path outputPath = GenerateSliceFolderPath(minPoint, maxPoint);
	if (fs::exists(outputPath)) {
		fs::remove(outputPath);//overwrite
	}
	fs::create_directories(outputPath);

	SaveSliceWarp(minPoint, maxPoint, frameIndex, sliceWarpPath);

	logger->minimum = minPoint;
	logger->maximum = maxPoint;
	logger->SaveCompact();
	slices[sliceIdentifier] = logger;
	return true;
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveSliceWarp(const Vector3i& minPoint,
                                                                   const Vector3i& maxPoint,
                                                                   unsigned int frameIndex,
                                                                   std::string path) {

	int totalHashEntryCount = fullCanonicalSceneLogger->scene->index.noTotalEntries;
	TVoxelCanonical* voxels = fullCanonicalSceneLogger->scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashTable = fullCanonicalSceneLogger->scene->index.GetEntries();

	// to restore to same file position later
	unsigned int currentGeneralWarpCursor = fullCanonicalSceneLogger->iterationCursor;

	bool wasLoadingWarpState = fullCanonicalSceneLogger->warpIFStream.is_open();
	StopLoadingWarpState();

	StartLoadingWarpState();

	std::ofstream sliceWarpOfstream(path.c_str(), std::ofstream::binary | std::ofstream::out);
	unsigned int sliceIterationCursor = 0;
	if (!sliceWarpOfstream)
		throw std::runtime_error("Could not open '" + path + "' for writing. ["  __FILE__  ": " +
		                         std::to_string(__LINE__) + "]");
	fullCanonicalSceneLogger->warpOFStream.write(reinterpret_cast<const char*>(&frameIndex), sizeof(int));

	while (LoadCurrentWarpState()) {
		fullCanonicalSceneLogger->warpOFStream.write(reinterpret_cast<const char* >(&sliceIterationCursor),
		                                             sizeof(unsigned int));
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
						fullCanonicalSceneLogger->warpOFStream.write(reinterpret_cast<const char* >(&voxel.warp_t),
						                                             sizeof(Vector3f));
						fullCanonicalSceneLogger->warpOFStream.write(
								reinterpret_cast<const char* >(&voxel.warp_t_update), sizeof(Vector3f));
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
		fullCanonicalSceneLogger->warpIFStream.seekg(
				headerSize + currentGeneralWarpCursor * (fullCanonicalSceneLogger->voxelCount * 2
				                                         * sizeof(Vector3f) + sizeof(unsigned int)),
				std::ios::beg);
		if (fullCanonicalSceneLogger->warpIFStream.eof()) {
			fullCanonicalSceneLogger->warpIFStream.clear();
		}
		fullCanonicalSceneLogger->iterationCursor = currentGeneralWarpCursor;
	}
}
//endregion


/**
 * \brief Verifies whether the slice specified by identifier (by looking at names of folder & files within)
 * \param sliceIdentifier a string-form scene slice identifier, usually consistent of min & max point coordinates
 * separated by '_'
 * \return true if all files corresponding to the slice exist, false otherwise.
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SliceExistsOnDisk(const std::string& sliceIdentifier) const {
	fs::path sliceFolderPath = GenerateSliceFolderPath(sliceIdentifier);
	fs::path sliceScenePath = GenerateSliceSceneFilename_Full(sliceIdentifier);
	fs::path sliceWarpPath = GenerateSliceWarpFilename(sliceIdentifier);
	return fs::is_directory(sliceFolderPath) && fs::is_regular_file(sliceScenePath) &&
	       fs::is_regular_file(sliceWarpPath);
}

/**
 * \brief Verifies whether the slice specified by the bounds exists (by looking at names of folder & files within)
 * \param extremum1 the first of the two points defining the slice bounds
 * \param extremum2 the second of the two points defining the slice bounds
 * \return true if all files corresponding to the slice exist, false otherwise.
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SliceExistsOnDisk(const Vector3i& extremum1,
                                                                            const Vector3i& extremum2) const {
	fs::path sliceFolderPath = GenerateSliceFolderPath(extremum1, extremum2);
	fs::path sliceScenePath = GenerateSliceSceneFilename_Full(extremum1, extremum2);
	fs::path sliceWarpPath = GenerateSliceWarpFilename(extremum1, extremum2);
	return fs::is_directory(sliceFolderPath) && fs::is_regular_file(sliceScenePath) &&
	       fs::is_regular_file(sliceWarpPath);
}

/**
 * \brief Verifies whether the slice exists in memory within the current object data structure
 * \param sliceIdentifier identifier of the slice (string form of min & max points with the six coordinates separated by '_')
 * \return true if the slice exists, false otherwise
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SliceExistsInMemory(const std::string& sliceIdentifier) const {
	return this->slices.find(sliceIdentifier) != slices.end();
}

/**
 * \brief Loads the specified scene slice into destinationScene, and sets up internal structures for loading its warp
 * \param sliceIdentifier identifier of the slice (string form of min & max points with the six coordinates separated by '_')
 * \param destinationScene where to store the slice voxels
 * \return true on success, false on failure
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::LoadSlice(const std::string& sliceIdentifier,
                                                                    ITMScene<TVoxelCanonical, TIndex>* destinationScene) {
	if (!SliceExistsOnDisk(sliceIdentifier)) {
		return false;
	}
	fs::path sliceFolderPath = GenerateSliceFolderPath(sliceIdentifier);
	std::string sliceScenePath = GenerateSliceSceneFilename_Full(sliceIdentifier);
	std::string sliceWarpPath = GenerateSliceWarpFilename(sliceIdentifier);
	ITMWarpSceneLogger<TVoxelCanonical, TIndex>* logger =
			new ITMWarpSceneLogger<TVoxelCanonical, TIndex>(destinationScene, sliceScenePath, sliceWarpPath);
	logger->LoadCompact();
	slices[sliceIdentifier] = logger;
	return true;
}

/**
 * \brief Switch to a different scene slice or to the full scene
 * \param sliceIdentifier either identifier of the slice (string form of min & max points with the six coordinates separated by '_'), or the full scene identifier, \sa ITMWarpSceneLogger::fullSceneSliceIdentifier
 * \return true on success, false on failure
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SwitchActiveScene(std::string sliceIdentifier) {
	bool isLoadingWarps = activeWarpLogger->IsLoadingWarpState();
	int iterationCursor = static_cast<int>(activeWarpLogger->GetIterationCursor());

	if (sliceIdentifier == ITMWarpSceneLogger<TVoxelCanonical, TIndex>::fullSceneSliceIdentifier) {
		mode = FULL_SCENE;
		activeWarpLogger->StopLoadingWarpState();
		activeWarpLogger = fullCanonicalSceneLogger;
	} else {
		if (SliceExistsInMemory(sliceIdentifier)) {
			mode = SLICE;
			activeWarpLogger->StopLoadingWarpState();
			activeWarpLogger = slices[sliceIdentifier];
		} else {
			return false;
		}
	}
	if (isLoadingWarps) {
		activeWarpLogger->StartLoadingWarpState();
		//set cursor to the *previous* iteration
		activeWarpLogger->SetIterationCursor(static_cast<unsigned int>(std::max(--iterationCursor, 0)));
	}
	return true;
}