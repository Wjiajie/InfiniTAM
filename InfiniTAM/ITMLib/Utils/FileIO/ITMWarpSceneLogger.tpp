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
#include "ITMWarpSceneLogger.h"
#include "../ITMSceneStatisticsCalculator.h"
#include "../ITMLibSettings.h"
#include "../../Engines/Reconstruction/ITMSceneReconstructionEngineFactory.h"


using namespace ITMLib;

// =====================================================================================================================
// =============================================== WARP SCENE WRAPPER INNER CLASS (CANONICAL SCENE & ITS SLICES)========
// =====================================================================================================================

// region ======================================== STATIC CONSTANTS ====================================================

template<typename TVoxel, typename TIndex>
const size_t ITMWarpSceneLogger<TVoxel, TIndex>::warpByteSize = sizeof(TVoxel::warp_t);

template<typename TVoxel, typename TIndex>
const size_t ITMWarpSceneLogger<TVoxel, TIndex>::updateByteSize = sizeof(TVoxel::warp_t_update);

template<typename TVoxel, typename TIndex>
const size_t ITMWarpSceneLogger<TVoxel, TIndex>::warpAndUpdateByteSize =
		ITMWarpSceneLogger<TVoxel, TIndex>::warpByteSize + ITMWarpSceneLogger<TVoxel, TIndex>::updateByteSize;

// endregion
// region ======================================== STATIC METHODS ======================================================

template<typename TVoxel, typename TIndex>
std::string ITMWarpSceneLogger<TVoxel, TIndex>::GenerateSliceStringIdentifier(
		const Vector3i& minPoint, const Vector3i& maxPoint) {
	return padded_to_string(minPoint.x, 3)
	       + "_" + padded_to_string(minPoint.y, 3)
	       + "_" + padded_to_string(minPoint.z, 3)
	       + "_" + padded_to_string(maxPoint.x, 3)
	       + "_" + padded_to_string(maxPoint.y, 3)
	       + "_" + padded_to_string(maxPoint.z, 3);
};

// endregion
// region ======================================== CONSTRUCTORS & DESTRUCTORS ==========================================

template<typename TVoxel, typename TIndex>
ITMWarpSceneLogger<TVoxel, TIndex>::ITMWarpSceneLogger(bool isSlice, ITMScene<TVoxel, TIndex>* scene, std::string scenePath, std::string warpPath):
		scene(scene),
		scenePath(scenePath),
		warpPath(warpPath),
		isSlice(isSlice),
		minimum(0),
		maximum(0){}

template<typename TVoxel, typename TIndex>
ITMWarpSceneLogger<TVoxel, TIndex>::~ITMWarpSceneLogger() {
}

// endregion
// region ======================================== GETTERS =============================================================

template<typename TVoxel, typename TIndex>
unsigned int ITMWarpSceneLogger<TVoxel, TIndex>::GetIterationCursor() const {
	return iterationCursor;
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
	if(!isSlice){
		return "full_scene";
	}
	return ITMWarpSceneLogger<TVoxel, TIndex>::GenerateSliceStringIdentifier(this->minimum,this->maximum);
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
	ITMSceneReconstructionEngine<TVoxel, TIndex>* reconstructionEngine =
			ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxel, TIndex>(
					ITMLibSettings::DEVICE_CPU);
	reconstructionEngine->ResetScene(scene);
	delete reconstructionEngine;
	scene->LoadFromDirectoryCompact_CPU(scenePath.c_str());
	ITMSceneStatisticsCalculator<TVoxel, TIndex> statisticsCalculator;
	this->voxelCount = statisticsCalculator.ComputeAllocatedVoxelCount(scene);

}
//endregion
// region ======================================== LOAD / SAVE / BUFFER WARP ===========================================

template<typename TVoxel, typename TIndex>
bool
ITMWarpSceneLogger<TVoxel, TIndex>::StartSavingWarpState(unsigned int frameIx) {
	if (!fs::is_directory(scenePath)) {
		std::cout << "The directory '" << scenePath << "' was not found.";
		return false;
	}
	warpOFStream = std::ofstream(warpPath.c_str(), std::ofstream::binary | std::ofstream::out);
	if (!warpOFStream)
		throw std::runtime_error("Could not open " + warpPath + " for writing ["  __FILE__  ": " +
		                         std::to_string(__LINE__) + "]");
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
		std::cout << "Current warp-update OFStream cannot be saved to for whatever reason." << std::endl;
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
					warpOFStream.write(reinterpret_cast<const char* >(&voxel.warp_t), warpByteSize);
					warpOFStream.write(reinterpret_cast<const char* >(&voxel.warp_t_update), updateByteSize);
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
	if (!fs::is_directory(this->scenePath)) {
		std::cerr << "The directory '" << scenePath << "' was not found.";
		return false;
	}

	warpIFStream = std::ifstream(warpPath.c_str(), std::ios::binary | std::ios::in);
	if (!warpIFStream) {
		std::cerr << "Could not open " + warpPath + " for reading. ["  __FILE__  ": " +
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
	if (!fs::is_directory(this->scenePath)) {
		std::cerr << "The directory '" << scenePath << "' was not found.";
		return false;
	}

	warpIFStream = std::ifstream(warpPath.c_str(), std::ios::binary | std::ios::in);
	if (!warpIFStream) {
		std::cerr << "Could not open " + warpPath + " for reading. ["  __FILE__  ": " +
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
 * \brief Transfers the warp state from the warp file to the scene, imitating the .warp_t and .warp_t_update fields after
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
					warpIFStream.read(reinterpret_cast<char*>(&voxel.warp_t), warpByteSize);
					warpIFStream.read(reinterpret_cast<char*>(&voxel.warp_t_update), updateByteSize);
					float warpUpdateLength = ORUtils::length(voxel.warp_t_update);
					if (warpUpdateLength > maxWarpUpdateLength) {
						maxWarpUpdateLength = warpUpdateLength;
					}
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
					warpIFStream.read(reinterpret_cast<char*>(&voxel.warp_t), warpByteSize);
					warpIFStream.read(reinterpret_cast<char*>(&voxel.warp_t_update), updateByteSize);
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
	warpIFStream.read(reinterpret_cast<char*>(externalBuffer), warpAndUpdateByteSize * voxelCount );
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
	size_t headerSize = sizeof(int);
	warpIFStream.seekg(headerSize + iterationIndex * (voxelCount * warpAndUpdateByteSize + sizeof(iterationCursor)),
	                   std::ios::beg);
	if (warpIFStream.eof()) {
		warpIFStream.clear();
	}
	//read in the number of the current update.
	if (!warpIFStream.read(reinterpret_cast<char*>(&iterationCursor), sizeof(iterationCursor))) {
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}

	iterationCursor = iterationIndex;
	warpIFStream.read(reinterpret_cast<char*>(externalBuffer), voxelCount * warpAndUpdateByteSize);
	std::cout << "Read warp state for iteration " << iterationCursor << std::endl;

	return !(warpIFStream.bad() || warpIFStream.fail());
}

// endregion