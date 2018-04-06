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
#include <regex>

//boost
#include <boost/range/iterator_range_core.hpp>

//ITMLib
#include "ITMSceneLogger.h"
#include "../ITMLibSettings.h"
#include "../../Objects/Scene/ITMSceneManipulation.h"

using namespace ITMLib;

//======================================================================================================================
//================================================= SCENE LOGGER CLASS CONSTANTS & METHODS RELATING TO SLICES ==========
//======================================================================================================================
// region ======================================== SLICE-RELATED CONSTANTS =============================================

// endregion
// region ======================================== SLICE PATH GENERATION ===============================================

//endregion
// region ======================================== MAKING & SAVING SLICE AND SLICE WARP ================================

//Assumes we have a scene loaded, as well as a warp file available.
/**
 * \brief Make slice of the canonical scene with the specified extrema, store it locally,
 * and save it to disk, (overwrites if files exist already on disk)
 * \param extremum1 [in] the first of the two points defining the slice bounds
 * \param extremum2 [in] the second of the two points defining the slice bounds
 * \param frameIndex [in] index of the frame to write to the saved warp file
 * \param identifier [out] string slice identifier to later retrive / switch to the slice
 * \return true on success, false on failure
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::MakeSlice(const Vector3i& extremum1, const Vector3i& extremum2,
                                                                    unsigned int frameIndex, std::string& identifier) {

	Vector3i minPoint, maxPoint;
	MinMaxFromExtrema(minPoint, maxPoint, extremum1, extremum2);
	identifier = ITMWarpSceneLogger<TVoxelCanonical, TIndex>::GenerateSliceStringIdentifier(minPoint,maxPoint);

	if (slices.find(identifier) != slices.end()) {
		return false;
	}
	auto logger = new ITMWarpSceneLogger<TVoxelCanonical, TIndex>(minPoint, maxPoint, this->path);
	if (!CopySceneSlice_CPU(logger->scene, fullCanonicalSceneLogger->scene, minPoint, maxPoint)) {
		return false;
	}
	fs::path outputPath = logger->path;
	if (fs::exists(outputPath)) {
		fs::remove_all(outputPath);//overwrite
	}
	fs::create_directories(outputPath);

	SaveSliceWarp(minPoint, maxPoint, frameIndex, logger->warpPath);

	logger->minimum = minPoint;
	logger->maximum = maxPoint;
	logger->SaveCompact();
	logger->sliceLoaded = true;
	slices[identifier] = logger;
	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::MakeSlice(const Vector3i& extremum1, const Vector3i& extremum2,
                                                                    unsigned int frameIndex){
	std::string identifier;
	return MakeSlice(extremum1,extremum2,frameIndex,identifier);
};


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveSliceWarp(const Vector3i& minPoint,
                                                                   const Vector3i& maxPoint,
                                                                   unsigned int frameIndex,
                                                                   const boost::filesystem::path& path) {

	int totalHashEntryCount = fullCanonicalSceneLogger->scene->index.noTotalEntries;
	TVoxelCanonical* voxels = fullCanonicalSceneLogger->scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashTable = fullCanonicalSceneLogger->scene->index.GetEntries();

	std::ofstream sliceWarpOfstream(path.c_str(), std::ofstream::binary | std::ofstream::out);
	if (!sliceWarpOfstream)
		throw std::runtime_error("Could not open '" + path.string() + "' for writing. ["  __FILE__  ": " +
		                         std::to_string(__LINE__) + "]");
	sliceWarpOfstream.write(reinterpret_cast<const char*>(&frameIndex), sizeof(frameIndex));


	bool wasCanonicalLoadingWarp = fullCanonicalSceneLogger->IsLoadingWarpState();
	unsigned int originalfullSceneIterationCursor = fullCanonicalSceneLogger->GetIterationCursor();
	fullCanonicalSceneLogger->StopLoadingWarpState();
	fullCanonicalSceneLogger->StartLoadingWarpState();
	int zRangeStart, zRangeEnd, yRangeStart, yRangeEnd, xRangeStart, xRangeEnd;

	while (fullCanonicalSceneLogger->LoadCurrentWarpState()) {
		unsigned int sliceIterationCursor = fullCanonicalSceneLogger->GetIterationCursor();
		sliceWarpOfstream.write(reinterpret_cast<const char* >(&sliceIterationCursor), sizeof(sliceIterationCursor));
		for (int hash = 0; hash < totalHashEntryCount; hash++) {
			const ITMHashEntry& hashEntry = hashTable[hash];
			if (hashEntry.ptr < 0) continue;

			//position of the current entry in 3D space (in voxel units)
			Vector3i hashBlockPositionVoxels = hashEntry.pos.toInt() * SDF_BLOCK_SIZE;
			const TVoxelCanonical* localVoxelBlock = &(voxels[hashEntry.ptr * (SDF_BLOCK_SIZE3)]);

			//if no voxels in the block are within range, skip
			if (IsHashBlockFullyInRange(hashBlockPositionVoxels, minPoint, maxPoint) ||
			    IsHashBlockPartiallyInRange(hashBlockPositionVoxels, minPoint, maxPoint)) {
				for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
					for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
						for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
							int ixVoxelInHashBlock = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
							const TVoxelCanonical& voxel = localVoxelBlock[ixVoxelInHashBlock];
							sliceWarpOfstream.write(reinterpret_cast<const char* >(&voxel.warp_t),
							                        sizeof(Vector3f));
							sliceWarpOfstream.write(
									reinterpret_cast<const char* >(&voxel.warp_t_update), sizeof(Vector3f));
						}
					}
				}
			}
		}
	}
	sliceWarpOfstream.close();
	fullCanonicalSceneLogger->StopLoadingWarpState();
	if(wasCanonicalLoadingWarp){
		fullCanonicalSceneLogger->SetIterationCursor(originalfullSceneIterationCursor);
	}
}
//endregion

// region =============================== OTHER SLICE-RELATED FUNCTIONS ================================================

/**
 * \brief Verifies whether the slice specified by identifier (by looking at names of folder & files within)
 * \param sliceIdentifier a string-form scene slice identifier, usually consistent of min & max point coordinates
 * separated by '_'
 * \return true if all files corresponding to the slice exist, false otherwise.
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SliceExistsOnDisk(const std::string& sliceIdentifier) const {
	fs::path sliceFolderPath =
			ITMWarpSceneLogger<TVoxelCanonical,TIndex>::GenerateSliceFolderPath(path,sliceIdentifier);
	fs::path sliceScenePath =
			ITMWarpSceneLogger<TVoxelCanonical,TIndex>::GenerateSliceSceneFilename_Full(path, sliceIdentifier);
	fs::path sliceWarpPath =
			ITMWarpSceneLogger<TVoxelCanonical,TIndex>::GenerateSliceWarpFilename(path, sliceIdentifier);
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
	fs::path sliceFolderPath =
			ITMWarpSceneLogger<TVoxelCanonical,TIndex>::GenerateSliceFolderPath(path, extremum1, extremum2);
	fs::path sliceScenePath =
			ITMWarpSceneLogger<TVoxelCanonical,TIndex>::GenerateSliceSceneFilename_Full(path, extremum1, extremum2);
	fs::path sliceWarpPath =
			ITMWarpSceneLogger<TVoxelCanonical,TIndex>::GenerateSliceWarpFilename(path, extremum1, extremum2);
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
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::LoadSlice(const std::string& sliceIdentifier) {
	if (!SliceExistsOnDisk(sliceIdentifier)) {
		return false;
	}
	Vector3i minPoint, maxPoint;
	ITMWarpSceneLogger<TVoxelCanonical,TIndex>::ExtractMinMaxFromSliceStringIdentifier(sliceIdentifier,minPoint,maxPoint);
	ITMWarpSceneLogger<TVoxelCanonical, TIndex>* logger =
			new ITMWarpSceneLogger<TVoxelCanonical, TIndex>(minPoint, maxPoint, path);
	logger->LoadCompact();
	logger->sliceLoaded = true;
	slices[sliceIdentifier] = logger;
	return true;
}

/**
 * \brief Switch to a different scene slice or to the full scene (if the requested slice or the full scene is available)
 * \param sliceIdentifier either identifier of the slice (string form of min & max points with the six coordinates separated by '_'), or the full scene identifier, \sa ITMWarpSceneLogger::fullSceneSliceIdentifier
 * \return true on success, false on failure
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SwitchActiveScene(std::string sliceIdentifier) {
	bool isLoadingWarps = activeWarpLogger->IsLoadingWarpState();
	int iterationCursor = static_cast<int>(activeWarpLogger->GetIterationCursor());

	if (sliceIdentifier == ITMWarpSceneLogger<TVoxelCanonical, TIndex>::fullSceneSliceIdentifier) {
		if(fullCanonicalSceneLogger != nullptr){
			mode = FULL_SCENE;
			activeWarpLogger->StopLoadingWarpState();
			activeWarpLogger = fullCanonicalSceneLogger;
		}else{
			return false;
		}
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

/**
 * \brief Loads all slices available in the logger's root path (if any).
 * \details If the path is invalid or not properly set, silently returns an empty vector of strings.
 * \return A vector of slice string identifiers for all slices that were discovered.
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::vector<std::string> ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::LoadAllSlices() {
	std::vector<std::string> identifiers;
	if(!CheckPath()){
		return identifiers;
	}
	const int coordCount = 6;
	std::regex sliceDirectoryRegex("slice(?:_(?:[-]|\\d)\\d{2}){" + std::to_string(coordCount) + "}");

	for (auto& entry : boost::make_iterator_range(fs::directory_iterator(this->path))){
		std::string directoryName = entry.path().filename().string();
		if(fs::is_directory(entry.path()) && std::regex_match(directoryName, sliceDirectoryRegex)){
			Vector3i minPoint, maxPoint;
			ITMWarpSceneLogger<TVoxelCanonical,TIndex>::ExtractMinMaxFromSliceStringIdentifier(directoryName,minPoint,maxPoint);
			std::string sliceIdentifier =
					ITMWarpSceneLogger<TVoxelCanonical,TIndex>::GenerateSliceStringIdentifier(minPoint, maxPoint);
			if(LoadSlice(sliceIdentifier)){
				identifiers.push_back(sliceIdentifier);
			}
		}
	}

}
// endregion