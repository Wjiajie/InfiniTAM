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

//boost
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

using namespace ITMLib;


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::ITMSceneLogger(
		std::string path,
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
		ITMScene<TVoxelLive, TIndex>* liveScene) :
		path(path),
		warpUpdatesPath(this->path / "warp_updates.dat"),
		canonicalPath(this->path / "canonical"),
		livePath(this->path / "live"),
		canonicalScene(canonicalScene),
		liveScene(liveScene) {
	if (!fs::create_directories(this->path) && !fs::is_directory(this->path)){
		DIEWITHEXCEPTION(std::string("Could not create the directory '") + path + "'. Exiting.");
	}
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveScenes() {
	if (!fs::is_directory(this->path)) {
		std::cout << "The directory '" << path << "' was not found.";
		return false;
	}
	std::cout <<"Saving scenes for current frame (this might take awhile)..." << std::endl;
	std::cout.flush();
	liveScene->SaveToDirectory(livePath.string());
	canonicalScene->SaveToDirectory(canonicalPath.string());
	std::cout <<"Scenes saved." << std::endl;
	return true;
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::LoadScenes() {
	if (!fs::is_directory(this->path)) {
		std::cout << "The directory '" << path << "' was not found.";
		return false;
	}
	std::cout <<"Loading scenes for current frame (this might take awhile)..." << std::endl;
	std::cout.flush();
	liveScene->LoadFromDirectory(livePath.c_str());
	canonicalScene->LoadFromDirectory(canonicalPath.c_str());
	ITMSceneStatisticsCalculator<TVoxelCanonical,TIndex> statisticsCalculator;
	this->voxelCount = statisticsCalculator.ComputeHashedVoxelCount(canonicalScene);
	std::cout <<"Scenes loaded." << std::endl;
	return true;
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::StartSavingWarpState() {
	if (!fs::is_directory(path)) {
		std::cout << "The directory '" << path << "' was not found.";
		return false;
	}
	currentWarpOFStream = std::ofstream(warpUpdatesPath.c_str(),std::ofstream::binary | std::ofstream::out);
	if (!currentWarpOFStream) throw std::runtime_error("Could not open " + warpUpdatesPath.string() + " for writing");

	iUpdate = 0;
	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::StopSavingWarpState() {
	currentWarpOFStream.close();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveCurrentWarpState() {

	if(!currentWarpOFStream){
		std::cout << "Current warp-update OFStream cannot be saved to for whatever reason." << std::endl;
		return false;
	}
	currentWarpOFStream.write(reinterpret_cast<const char* >(&this->iUpdate), sizeof(unsigned int));
	const TVoxelCanonical* voxels = canonicalScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashBlocks = canonicalScene->index.GetEntries();
	int hashBlockCount = canonicalScene->index.noTotalEntries;

	for (int iHashBlock = 0; iHashBlock < hashBlockCount; iHashBlock++) {
		const ITMHashEntry& currentHashBlock = hashBlocks[iHashBlock];
		if(currentHashBlock.ptr < 0) continue;
		const TVoxelCanonical* localVoxelBlock = &(voxels[currentHashBlock.ptr * (SDF_BLOCK_SIZE3)]);
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int ixVoxelInHashBlock = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					const TVoxelCanonical& voxel = localVoxelBlock[ixVoxelInHashBlock];
					currentWarpOFStream.write(reinterpret_cast<const char* >(&voxel.warp_t), sizeof(Vector3f));
					currentWarpOFStream.write(reinterpret_cast<const char* >(&voxel.warp_t_update), sizeof(Vector3f));
				}
			}
		}
	}
	std::cout << "Written warp updates for iteration " << iUpdate << " to disk." << std::endl;
	iUpdate++;
	return true;
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::StartLoadingWarpState() {
	if (this->voxelCount == -1){
		std::cout << "Hashed voxel count has not been obtained. Have the scenes been loaded successfully?" <<std::endl;
		return false;
	}
	if (!fs::is_directory(this->path)) {
		std::cout << "The directory '" << path << "' was not found.";
		return false;
	}

	currentWarpIFStream = std::ifstream(warpUpdatesPath.c_str(), std::ios::binary | std::ios::in);
	if (!currentWarpIFStream) throw std::runtime_error("Could not open " + warpUpdatesPath.string() + " for reading");
	iUpdate = 0;
	return true;
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::StopLoadingWarpState() {
	currentWarpIFStream.close();

}



template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::LoadNextWarpState() {
	if(!currentWarpIFStream){
		std::cout << "Attempted to read warp state with IFStream being in a bad state."
				" Was 'StartLoadingWarpState()' called?" << std::endl;
		return  false;
	}
	if(!currentWarpIFStream.read(reinterpret_cast<char*>(&iUpdate),sizeof(unsigned int))){
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
		if(currentHashBlock.ptr < 0) continue;
		TVoxelCanonical* localVoxelBlock = &(voxels[currentHashBlock.ptr * (SDF_BLOCK_SIZE3)]);
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int ixVoxelInHashBlock = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxelCanonical& voxel = localVoxelBlock[ixVoxelInHashBlock];
					currentWarpIFStream.read(reinterpret_cast<char*>(&voxel.warp_t),sizeof(Vector3f));
					currentWarpIFStream.read(reinterpret_cast<char*>(&voxel.warp_t_update),sizeof(Vector3f));
					float warpUpdateLength = ORUtils::length(voxel.warp_t_update);
					if(warpUpdateLength > maxWarpUpdateLength){
						maxWarpUpdateLength = warpUpdateLength;
					}
					voxelCount++;
				}
			}
		}
	}
	std::cout << "Iteration " << iUpdate << ". Max warp update: " << maxWarpUpdateLength << std::endl;
	this->voxelCount = voxelCount;
	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::LoadPreviousWarpState() {
	if(iUpdate < 1){
		return false;
	}

	currentWarpIFStream.seekg( -(voxelCount*2*sizeof(Vector3f) + sizeof(unsigned int)), std::ios::cur);

	if(!currentWarpIFStream.read(reinterpret_cast<char*>(&iUpdate),sizeof(unsigned int))){
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}

	TVoxelCanonical* voxels = canonicalScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashBlocks = canonicalScene->index.GetEntries();
	int hashBlockCount = canonicalScene->index.noTotalEntries;

	for (int iHashBlock = 0; iHashBlock < hashBlockCount; iHashBlock++) {
		const ITMHashEntry& currentHashBlock = hashBlocks[iHashBlock];
		if(currentHashBlock.ptr < 0) continue;
		TVoxelCanonical* localVoxelBlock = &(voxels[currentHashBlock.ptr * (SDF_BLOCK_SIZE3)]);
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int ixVoxelInHashBlock = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxelCanonical& voxel = localVoxelBlock[ixVoxelInHashBlock];
					currentWarpIFStream.read(reinterpret_cast<char*>(&voxel.warp_t),sizeof(Vector3f));
					currentWarpIFStream.read(reinterpret_cast<char*>(&voxel.warp_t_update),sizeof(Vector3f));
				}
			}
		}
	}

	return true;

}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::IsLoadingWarpState() {
	return this->currentWarpIFStream.is_open();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::BufferNextWarpState() {
	if(!currentWarpIFStream){
		std::cout << "Attempted to read warp state with IFStream being in a bad state."
				" Was 'StartLoadingWarpState()' called?" << std::endl;
		return  false;
	}
	if(voxelCount == -1){
		std::cout << "Attempted to read warp state without knowing voxel count apriori."
				" Were scenes loaded successfully?" << std::endl;
		return  false;
	}
	//read in the number of the current update.
	if(!currentWarpIFStream.read(reinterpret_cast<char*>(&iUpdate),sizeof(unsigned int))){
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}

	if(warpBuffer == NULL){
		//allocate warp buffer
		warpBuffer = new Vector3f[voxelCount*2];
	}
	currentWarpIFStream.read(reinterpret_cast<char*>(warpBuffer),sizeof(Vector3f)*voxelCount*2);

	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::BufferPreviousWarpState() {
	if(iUpdate < 1){
		return false;
	}

	currentWarpIFStream.seekg( -(voxelCount*2*sizeof(Vector3f) + sizeof(unsigned int)), std::ios::cur);
	//read in the number of the current update.
	if(!currentWarpIFStream.read(reinterpret_cast<char*>(&iUpdate),sizeof(unsigned int))){
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}

	currentWarpIFStream.read(reinterpret_cast<char*>(warpBuffer),sizeof(Vector3f)*voxelCount*2);

	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::~ITMSceneLogger() {
	delete[] warpBuffer;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::CopyWarpBuffer(float* warpDestination,
                                                                         float* warpUpdateDestination, int& iUpdate) {
	if(!warpBuffer) return false;
	memcpy(reinterpret_cast<void*>(warpDestination),reinterpret_cast<void*>(warpBuffer), sizeof(Vector3f)*voxelCount);
	memcpy(reinterpret_cast<void*>(warpDestination),reinterpret_cast<void*>(warpBuffer + voxelCount), sizeof(Vector3f)*voxelCount);
	iUpdate = this->iUpdate;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::CopyWarpAt(int index, float voxelWarpDestination[3]) const {
	memcpy(reinterpret_cast<void*>(voxelWarpDestination),reinterpret_cast<void*>(warpBuffer + index*2), sizeof(Vector3f));
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::CopyWarpAt(int index, float* voxelWarpDestination,
                                                                     float* voxelUpdateDestination) const {
	memcpy(reinterpret_cast<void*>(voxelWarpDestination),reinterpret_cast<void*>(warpBuffer + index*2), sizeof(Vector3f));
	memcpy(reinterpret_cast<void*>(voxelUpdateDestination),reinterpret_cast<void*>(warpBuffer + index*2 + 1), sizeof(Vector3f));
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const float* ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::WarpAt(int index) const {
	return reinterpret_cast<const float*>(warpBuffer + index*2);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const float* ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::UpdateAt(int index) const {
	return reinterpret_cast<const float*>(warpBuffer + index*2 + 1);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
int ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetVoxelCount() const {
	if(voxelCount == -1) return 0;
	return voxelCount;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::BufferNextWarpState(void* externalBuffer) {
	if(!currentWarpIFStream){
		std::cout << "Attempted to read warp state with IFStream being in a bad state."
				" Was 'StartLoadingWarpState()' called?" << std::endl;
		return  false;
	}
	if(voxelCount == -1){
		std::cout << "Attempted to read warp state without knowing voxel count apriori."
				" Were scenes loaded successfully?" << std::endl;
		return  false;
	}
	//read in the number of the current update.
	if(!currentWarpIFStream.read(reinterpret_cast<char*>(&iUpdate),sizeof(unsigned int))){
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}

	if(warpBuffer == NULL){
		//allocate warp buffer
		warpBuffer = new Vector3f[voxelCount*2];
	}
	currentWarpIFStream.read(reinterpret_cast<char*>(externalBuffer),sizeof(Vector3f)*voxelCount*2);

	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::BufferPreviousWarpState(void* externalBuffer) {
	if(iUpdate < 1){
		return false;
	}

	currentWarpIFStream.seekg( -(voxelCount*2*sizeof(Vector3f) + sizeof(unsigned int)), std::ios::cur);
	//read in the number of the current update.
	if(!currentWarpIFStream.read(reinterpret_cast<char*>(&iUpdate),sizeof(unsigned int))){
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}

	currentWarpIFStream.read(reinterpret_cast<char*>(warpBuffer),sizeof(Vector3f)*voxelCount*2);

	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetScenesLoaded() const {
	return voxelCount != -1;
}






