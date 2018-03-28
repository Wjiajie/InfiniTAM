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
 * \param destinationSlice [out] the scene where to write the changes - presumed blank, will be overwritten (not managed)
 * \param extremum1 the first of the two points defining the slice bounds
 * \param extremum2 the second of the two points defining the slice bounds
 * \param frameIndex index of the frame to write to the saved warp file
 * \return the resultant sliceLogger on the heap (not managed) on success, nullptr on failure
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMWarpSceneLogger<TVoxelCanonical, TIndex>*
	ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::Slice(ITMScene<TVoxelCanonical, TIndex>* destinationSlice,
                                                                const Vector3i& extremum1, const Vector3i& extremum2,
                                                                unsigned int frameIndex) {

	Vector3i minPoint, maxPoint;
	MinMaxFromExtrema(minPoint, maxPoint, extremum1, extremum2);

	if (!CopySceneSlice_CPU(destinationSlice, canonicalScene.scene, minPoint, maxPoint)) {
		return nullptr;
	}
	fs::path outputPath = GenerateSliceFolderPath(minPoint, maxPoint);
	if (fs::exists(outputPath)) {
		fs::remove(outputPath);//overwrite
	}
	fs::create_directories(outputPath);
	std::string sceneOutputPath = GenerateSliceSceneFilename_UpToPostfix(minPoint, maxPoint);
	destinationSlice->SaveToDirectoryCompact_CPU(sceneOutputPath);
	std::string sliceWarpPath = GenerateSliceWarpFilename(minPoint, maxPoint);
	SaveSliceWarp(minPoint, maxPoint, frameIndex, sliceWarpPath);
	return new ITMWarpSceneLogger<TVoxelCanonical,TIndex>(true, destinationSlice, sceneOutputPath, sliceWarpPath);
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveSliceWarp(const Vector3i& minPoint,
                                                                   const Vector3i& maxPoint,
                                                                   unsigned int frameIndex,
                                                                   std::string path) {

	int totalHashEntryCount = canonicalScene.scene->index.noTotalEntries;
	TVoxelCanonical* voxels = canonicalScene.scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashTable = canonicalScene.scene->index.GetEntries();

	// to restore to same file position later
	unsigned int currentGeneralWarpCursor = canonicalScene.generalIterationCursor;

	bool wasLoadingWarpState = canonicalScene.warpIFStream.is_open();
	StopLoadingWarpState();

	StartLoadingWarpState();

	std::ofstream sliceWarpOfstream(path.c_str(), std::ofstream::binary | std::ofstream::out);
	unsigned int sliceIterationCursor = 0;
	if (!sliceWarpOfstream)
		throw std::runtime_error("Could not open '" + path + "' for writing. ["  __FILE__  ": " +
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
