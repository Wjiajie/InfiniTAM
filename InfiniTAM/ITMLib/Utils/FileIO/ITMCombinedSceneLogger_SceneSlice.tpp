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
#include "ITMCombinedSceneLogger.h"
#include "../ITMLibSettings.h"
#include "../../Objects/Scene/ITMSceneManipulation.h"

using namespace ITMLib;

// region ======================================== FILE PATH GENERATION ================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string ITMCombinedSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GenerateSliceStringIdentifier(
		const Vector3i& minPoint, const Vector3i& maxPoint) {
	return padded_to_string(minPoint.x, 3)
	       + "_" + padded_to_string(minPoint.y, 3)
	       + "_" + padded_to_string(minPoint.z, 3)
	       + "_" + padded_to_string(maxPoint.x, 3)
	       + "_" + padded_to_string(maxPoint.y, 3)
	       + "_" + padded_to_string(maxPoint.z, 3);
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string ITMCombinedSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GenerateSliceWarpFilename(
		const Vector3i& minPoint,
		const Vector3i& maxPoint) {
	return (this->path / (warpUpdatesFilename + "_" + GenerateSliceStringIdentifier(minPoint, maxPoint) +
	                      binaryFileExtension)).string();
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string ITMCombinedSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GenerateSliceSceneFilename_UpToPostfix(
		const Vector3i& minPoint,
		const Vector3i& maxPoint) {

	return this->canonicalPath.string()
	       + "_" + GenerateSliceStringIdentifier(minPoint, maxPoint);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::string ITMCombinedSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GenerateSliceSceneFilename_Full(
		const Vector3i& minPoint,
		const Vector3i& maxPoint) {
	return GenerateSliceSceneFilename_UpToPostfix(minPoint, maxPoint)
	       + ITMScene<TVoxelCanonical,TVoxelLive>::compactFilePostfixAndExtension;
}

//endregion
// region ======================================== SAVING SLICE AND SLICE WARP =========================================

//Assumes we have a scene loaded, as well as a warp file available.
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMCombinedSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveCanonicalSceneSlice(const Vector3i& extremum1,
                                                                                  const Vector3i& extremum2,
                                                                                  unsigned int frameIndex) {

	Vector3i minPoint, maxPoint;
	MinMaxFromExtrema(minPoint, maxPoint, extremum1, extremum2);
	ITMLibSettings* settings = new ITMLibSettings();
	ITMScene<TVoxelCanonical, TIndex>* slice = new ITMScene<TVoxelCanonical, TIndex>(canonicalScene->sceneParams,
	                                                                                 settings->swappingMode ==
	                                                                                 ITMLibSettings::SWAPPINGMODE_ENABLED,
	                                                                                 settings->GetMemoryType());
	if (!CopySceneSlice_CPU(slice, canonicalScene, minPoint, maxPoint)) {
		return false;
	}
	std::string sceneOutputPath = GenerateSliceSceneFilename_UpToPostfix(minPoint, maxPoint);
	slice->SaveToDirectoryCompact_CPU(sceneOutputPath);

	SaveWarpSlice(minPoint, maxPoint, frameIndex);
	delete settings;
	return false;
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void
ITMCombinedSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveWarpSlice(const Vector3i& minPoint, const Vector3i& maxPoint,
                                                                   unsigned int frameIndex) {

	int totalHashEntryCount = canonicalScene->index.noTotalEntries;
	TVoxelCanonical* voxels = canonicalScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashTable = canonicalScene->index.GetEntries();

	// to restore to same file position later
	unsigned int currentGeneralWarpCursor = this->generalIterationCursor;

	bool wasLoadingWarpState = this->warpIFStream.is_open();
	StopLoadingWarpState();

	StartLoadingWarpState();

	std::string sliceWarpOfstreamPath = GenerateSliceWarpFilename(minPoint, maxPoint);

	std::ofstream sliceWarpOfstream(sliceWarpOfstreamPath.c_str(), std::ofstream::binary | std::ofstream::out);
	unsigned int sliceIterationCursor = 0;
	if (!sliceWarpOfstream)
		throw std::runtime_error("Could not open " + sliceWarpOfstreamPath + " for writing. ["  __FILE__  ": " +
		                         std::to_string(__LINE__) + "]");
	warpOFStream.write(reinterpret_cast<const char*>(&frameIndex), sizeof(int));

	while (LoadCurrentWarpState()) {
		warpOFStream.write(reinterpret_cast<const char* >(&sliceIterationCursor), sizeof(unsigned int));
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
						warpOFStream.write(reinterpret_cast<const char* >(&voxel.warp_t), sizeof(Vector3f));
						warpOFStream.write(reinterpret_cast<const char* >(&voxel.warp_t_update), sizeof(Vector3f));
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
		warpIFStream.seekg(
				headerSize + currentGeneralWarpCursor * (voxelCount * 2 * sizeof(Vector3f) + sizeof(unsigned int)),
				std::ios::beg);
		if (warpIFStream.eof()) {
			warpIFStream.clear();
		}
		this->generalIterationCursor = currentGeneralWarpCursor;
	}
}
//endregion