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

namespace fs = boost::filesystem;

using namespace ITMLib;

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::binaryFileExtension = ".dat";
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::textFileExtension = ".txt";

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::ITMSceneLogger(
		std::string path,
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
		ITMScene<TVoxelLive, TIndex>* liveScene) :
		path(path),
		canonicalPath(this->path / "canonical"),
		livePath(this->path / "live"),
		canonicalScene(canonicalScene),
		liveScene(liveScene),
		warpUpdatesPath(this->path / ("warp_updates" + binaryFileExtension)),
		highlights("Hash ID", "Local voxel ix", "Frame", "Iteration"),
		highlightsBinaryPath(this->path / ("highlights" + binaryFileExtension)),
        highlightsTextPath(this->path / ("highlights" + textFileExtension)){
	if (!fs::create_directories(this->path) && !fs::is_directory(this->path)){
		DIEWITHEXCEPTION(std::string("Could not create the directory '") + path + "'. Exiting.");
	}
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveScenes() {
	if (!CheckDirectory()) {
		return false;
	}
	std::cout <<"Saving scenes for current frame (this might take awhile)..." << std::endl;
	std::cout.flush();
	liveScene->SaveToDirectory(livePath.string());
	canonicalScene->SaveToDirectory(canonicalPath.string());
	ITMSceneStatisticsCalculator<TVoxelCanonical,TIndex> statisticsCalculator;
	this->voxelCount = statisticsCalculator.ComputeHashedVoxelCount(canonicalScene);
	std::cout <<"Scenes saved." << std::endl;
	return true;
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::LoadScenes() {
	if (!CheckDirectory()) {
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
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveScenesCompact() {
	if(!liveScene || !canonicalScene){
		std::cerr << "At least one of the two scenes, canonical/live, was not set to an actual scene. "
		          << __FILE__ << ":" << __LINE__ << std::endl;
	}
	if (!fs::is_directory(this->path)) {
		std::cout << "The directory '" << path << "' was not found.";
		return false;
	}
	std::cout <<"Saving scenes for current frame (this might take awhile)..." << std::endl;
	std::cout.flush();
	liveScene->SaveToDirectoryCompact_CPU(livePath.string());
	canonicalScene->SaveToDirectoryCompact_CPU(canonicalPath.string());
	ITMSceneStatisticsCalculator<TVoxelCanonical,TIndex> statisticsCalculator;
	this->voxelCount = statisticsCalculator.ComputeHashedVoxelCount(canonicalScene);
	std::cout <<"Scenes saved." << std::endl;
	return true;
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::LoadScenesCompact() {
	if (!CheckDirectory()) {
		return false;
	}
	std::cout <<"Loading scenes for current frame (this might take awhile)..." << std::endl;
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
	ITMSceneStatisticsCalculator<TVoxelCanonical,TIndex> statisticsCalculator;
	this->voxelCount = statisticsCalculator.ComputeHashedVoxelCount(canonicalScene);
	std::cout <<"Scenes loaded." << std::endl;
	delete reconstructionEngineCanonical;
	delete reconstructionEngineLive;
	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::StartSavingWarpState() {
	if (!fs::is_directory(path)) {
		std::cout << "The directory '" << path << "' was not found.";
		return false;
	}
	warpOFStream = std::ofstream(warpUpdatesPath.c_str(),std::ofstream::binary | std::ofstream::out);
	if (!warpOFStream) throw std::runtime_error("Could not open " + warpUpdatesPath.string() + " for writing");

	iUpdate = 0;
	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::StopSavingWarpState() {
	warpOFStream.close();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveCurrentWarpState() {

	if(!warpOFStream){
		std::cout << "Current warp-update OFStream cannot be saved to for whatever reason." << std::endl;
		return false;
	}
	warpOFStream.write(reinterpret_cast<const char* >(&this->iUpdate), sizeof(unsigned int));
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
					warpOFStream.write(reinterpret_cast<const char* >(&voxel.warp_t), sizeof(Vector3f));
					warpOFStream.write(reinterpret_cast<const char* >(&voxel.warp_t_update), sizeof(Vector3f));
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

	warpIFStream = std::ifstream(warpUpdatesPath.c_str(), std::ios::binary | std::ios::in);
	if (!warpIFStream) throw std::runtime_error("Could not open " + warpUpdatesPath.string() + " for reading");
	iUpdate = 0;
	return true;
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::StopLoadingWarpState() {
	warpIFStream.close();

}



template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::LoadNextWarpState() {
	if(!warpIFStream){
		std::cout << "Attempted to read warp state with IFStream being in a bad state."
				" Was 'StartLoadingWarpState()' called?" << std::endl;
		return  false;
	}
	if(!warpIFStream.read(reinterpret_cast<char*>(&iUpdate),sizeof(unsigned int))){
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
					warpIFStream.read(reinterpret_cast<char*>(&voxel.warp_t),sizeof(Vector3f));
					warpIFStream.read(reinterpret_cast<char*>(&voxel.warp_t_update),sizeof(Vector3f));
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

	warpIFStream.seekg( -2*(voxelCount*2*sizeof(Vector3f) + sizeof(unsigned int)), std::ios::cur);

	if(!warpIFStream.read(reinterpret_cast<char*>(&iUpdate),sizeof(unsigned int))){
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
					warpIFStream.read(reinterpret_cast<char*>(&voxel.warp_t),sizeof(Vector3f));
					warpIFStream.read(reinterpret_cast<char*>(&voxel.warp_t_update),sizeof(Vector3f));
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
	if(!warpIFStream){
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
	if(!warpIFStream.read(reinterpret_cast<char*>(&iUpdate),sizeof(unsigned int))){
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}

	if(warpBuffer == NULL){
		//allocate warp buffer
		warpBuffer = new Vector3f[voxelCount*2];
	}
	warpIFStream.read(reinterpret_cast<char*>(warpBuffer),sizeof(Vector3f)*voxelCount*2);

	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::BufferPreviousWarpState() {
	if(iUpdate < 1){
		return false;
	}

	warpIFStream.seekg( -2*(voxelCount*2*sizeof(Vector3f) + sizeof(unsigned int)), std::ios::cur);
	//read in the number of the current update.
	if(!warpIFStream.read(reinterpret_cast<char*>(&iUpdate),sizeof(unsigned int))){
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}

	warpIFStream.read(reinterpret_cast<char*>(warpBuffer),sizeof(Vector3f)*voxelCount*2);

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
	if(!warpIFStream){
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
	if(!warpIFStream.read(reinterpret_cast<char*>(&iUpdate),sizeof(unsigned int))){
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}
	warpIFStream.read(reinterpret_cast<char*>(externalBuffer),sizeof(Vector3f)*voxelCount*2);
	std::cout << "Read warp state for iteration " << iUpdate << std::endl;
	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::BufferPreviousWarpState(void* externalBuffer) {
	if(iUpdate < 1){
		return false;
	}

	warpIFStream.seekg( -2*(voxelCount*2*sizeof(Vector3f) + sizeof(unsigned int)), std::ios::cur);
	//read in the number of the current update.
	if(!warpIFStream.read(reinterpret_cast<char*>(&iUpdate),sizeof(unsigned int))){
		std::cout << "Read warp state attempt failed." << std::endl;
		return false;
	}

	warpIFStream.read(reinterpret_cast<char*>(externalBuffer),sizeof(Vector3f)*voxelCount*2);
	std::cout << "Read warp state for iteration " << iUpdate << std::endl;

	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetScenesLoaded() const {
	return voxelCount != -1;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::LogHighlight(
		int hashId, int voxelLocalIndex, int frameNumber, int iterationNumber) {
	highlights.InsertOrdered(hashId,voxelLocalIndex,frameNumber,iterationNumber);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveHighlights() {
	if (!fs::is_directory(this->path)) {
		std::cout << "The directory '" << path << "' was not found.";
		return false;
	}
	if(!this->highlights.SaveToFile(highlightsBinaryPath.c_str())){
		std::cerr << "Could not save highlights to " << highlightsBinaryPath << std::endl;
		return false;
	}else{
		std::cout << "Saved highlights to" << highlightsBinaryPath << std::endl;
	}
	if(!this->highlights.SaveToTextFile(highlightsTextPath.c_str())){
		std::cerr << "Could not save highlights to " << highlightsTextPath << std::endl;
		return false;
	}else{
		std::cout << "Saved highlights to" << highlightsTextPath << std::endl;
	}
	return true;
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::LoadHighlights() {
	if (!fs::is_directory(this->path)) {
		std::cout << "The directory '" << path << "' was not found.";
		return false;
	}
	if(!this->highlights.LoadFromFile(highlightsBinaryPath.c_str())){
		std::cout << "Could not load highlights from " << highlightsBinaryPath << std::endl;
		return false;
	} else {
		std::cout << "Loaded highlights from " << highlightsBinaryPath << std::endl;
		return true;
	}
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SetScenes(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {
	this->liveScene = liveScene;
	this->canonicalScene = canonicalScene;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::PrintHighlights() {
	std::cout <<"*** Highlights ***" << std::endl;
	std::cout << this->highlights << std::endl;
}



/**
 * \brief Set up the interest regions whose warps to save into individual files based on
 * (a) highlights (which have to be loaded / defined)
 * (b) existing canonical scene (which have to be loaded)
 *
 * \tparam TVoxelCanonical type of voxel in canonical scene
 * \tparam TVoxelLive type of voxel in live scene
 * \tparam TIndex type of voxel index structure
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SetUpInterestRegionsForSaving() {
	if(this->voxelCount == -1){
		DIEWITHEXCEPTION("Attempted to set up interest regions before loading the scenes. Aborting.");
	}
	interestRegionInfos.clear();
	interestRegionInfoByHashId.clear();

	const ITMHashEntry* hashBlocks = canonicalScene->index.GetEntries();
	int hashBlockCount = canonicalScene->index.noTotalEntries;

	//traverse hash blocks where anomalies/errors/oscillations occur
	for(int centerHashId : highlights.GetOuterLevelKeys()){
		const ITMHashEntry& currentHashBlock = hashBlocks[centerHashId];
		if(currentHashBlock.ptr < 0) {
			throw std::runtime_error("Got hash Id " + std::to_string(centerHashId)
			                         + " in the highlights that doesn't correspond to a populated block in the scene");
		}
		std::vector<int> regionHashIds;
		Vector3s centerBlockPos = currentHashBlock.pos;
		//traverse neighborhood of the interest hash block in a predefined order
		for(Vector3s offset : InterestRegionInfo::blockTraversalOrder){
			int iHashBlock = hashIndex(centerBlockPos+offset);
			if(iHashBlock >= 0 && iHashBlock < hashBlockCount && hashBlocks[iHashBlock].ptr >= 0){
				regionHashIds.push_back(iHashBlock);
			}
		}
		std::shared_ptr<InterestRegionInfo> info(new InterestRegionInfo(regionHashIds, centerHashId, *this));
		//instert the same region into map by the hash blocks it contains
		for(int regionHashBlockId : regionHashIds){
			interestRegionInfoByHashId[regionHashBlockId] = info;
		}
		interestRegionInfos.push_back(info);
	}

}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveAllInterestRegionWarps() {
	for(std::shared_ptr<InterestRegionInfo> info: interestRegionInfos){
		info->SaveCurrentWarpState();
	}
	std::cout << "Saved all interest region warps." << std::endl;
}

/**
 * \brief Set up all interest regions for loading based on the files in current active directory for the loader.
 * \details The files should follow the interest region naming convention
 * \tparam TVoxelCanonical
 * \tparam TVoxelLive
 * \tparam TIndex
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SetUpInterestRegionsForLoading() {
	if(this->voxelCount == -1){
		DIEWITHEXCEPTION("Attempted to set up interest regions before loading the scenes. Aborting.");
	}
	interestRegionInfos.clear();
	interestRegionInfoByHashId.clear();

	std::regex regionFileRegex(InterestRegionInfo::prefix + "\\d+" + binaryFileExtension);
	std::smatch match;
	std::vector<fs::path> regionPaths;
	for(fs::directory_iterator itr{path};itr != fs::directory_iterator{}; itr++){
		std::string filename = fs::basename(itr->path());
		if(fs::is_regular_file(itr->path()) && std::regex_match(filename, match, regionFileRegex)){
			std::shared_ptr<InterestRegionInfo> info(new InterestRegionInfo(itr->path(),*this));
			for(const int iHashBlockId : info->GetHashBlockIds()){
				interestRegionInfoByHashId[iHashBlockId] = info;
			}
			interestRegionInfos.push_back(info);
		}
	}

}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::FilterHighlights(int anomalyFrameCountMinimum) {
	highlights = highlights.FilterBasedOnLevel0Lengths(anomalyFrameCountMinimum);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::CheckDirectory() {
	if(!liveScene || !canonicalScene){
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
const Vector3s ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo::blockTraversalOrder[] = 
		{Vector3s(-1,-1,-1), Vector3s(-1,-1, 0), Vector3s(-1,-1, 1),
		 Vector3s(-1, 0,-1), Vector3s(-1, 0, 0), Vector3s(-1, 0, 1),
		 Vector3s(-1, 1,-1), Vector3s(-1, 1, 0), Vector3s(-1, 1, 1),
		 
		 Vector3s( 0,-1,-1), Vector3s( 0,-1, 0), Vector3s( 0,-1, 1),
		 Vector3s( 0, 0,-1), Vector3s( 0, 0, 0), Vector3s( 0, 0, 1),
		 Vector3s( 0, 1,-1), Vector3s( 0, 1, 0), Vector3s( 0, 1, 1),

		 Vector3s( 1,-1,-1), Vector3s( 1,-1, 0), Vector3s( 1,-1, 1),
		 Vector3s( 1, 0,-1), Vector3s( 1, 0, 0), Vector3s( 1, 0, 1),
		 Vector3s( 1, 1,-1), Vector3s( 1, 1, 0), Vector3s( 1, 1, 1)};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo::prefix = "region_";
/**
 * \brief Make an interest region info for saving warps in a specific hash block neighborhood to disk
 * \tparam TVoxelCanonical type of canonical/reference scene voxels
 * \tparam TVoxelLive type of live/target scene voxels
 * \tparam TIndex indexing structure
 * \param hashBlockIds all hash block ids in the region
 * \param centerHashBlockId the central hash block (where the highlight/anomaly occurs)
 * \param parent parent logger
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo::InterestRegionInfo(
		std::vector<int>& hashBlockIds, int centerHashBlockId,
		ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>& parent):
		centerHashBlockId(centerHashBlockId),
		hashBlockIds(hashBlockIds),
		path(parent.path / fs::path(prefix + std::to_string(centerHashBlockId) + binaryFileExtension)),
		ifStream(),
		ofStream(std::ofstream(path.c_str(),std::ios::binary | std::ios::out)),
		parent(parent),
		voxelCount(static_cast<int>(hashBlockIds.size()) * SDF_BLOCK_SIZE3){
	if (!ofStream) throw std::runtime_error("Could not open " + path.string() + " for writing");
	//write region header
	ofStream.write(reinterpret_cast<const char* >(&centerHashBlockId), sizeof(unsigned int));
	size_t hashBlockCount = hashBlockIds.size();
	ofStream.write(reinterpret_cast<const char* >(&hashBlockCount), sizeof(size_t));
	for(int hashBlockId : hashBlockIds){
		ofStream.write(reinterpret_cast<const char* >(&hashBlockId), sizeof(int));
	}
	isSaving = true;
	isLoading = false;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo::~InterestRegionInfo() {
	ofStream.close();
	ifStream.close();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo::SaveCurrentWarpState() {
	if(isLoading){
		throw std::runtime_error("Attempting to save region made for loading (not allowed). Use alternative constructor.");
	}

	ofStream.write(reinterpret_cast<const char* >(&iUpdate), sizeof(unsigned int));
	const TVoxelCanonical* voxels = parent.canonicalScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashBlocks = parent.canonicalScene->index.GetEntries();
	for(int iHashBlock : hashBlockIds){
		const ITMHashEntry& currentHashBlock = hashBlocks[iHashBlock];
		if(currentHashBlock.ptr < 0) continue;
		const TVoxelCanonical* localVoxelBlock = &(voxels[currentHashBlock.ptr * (SDF_BLOCK_SIZE3)]);
		//TODO: make static inline, reuse above in write warp updates -Greg (GitHub: Algomorph)
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int ixVoxelInHashBlock = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					const TVoxelCanonical& voxel = localVoxelBlock[ixVoxelInHashBlock];
					ofStream.write(reinterpret_cast<const char* >(&voxel.warp_t), sizeof(Vector3f));
					ofStream.write(reinterpret_cast<const char* >(&voxel.warp_t_update), sizeof(Vector3f));
				}
			}
		}
	}

	iUpdate++;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo::BufferNextWarpState(void* externalBuffer) {
	if(isSaving){
		throw std::runtime_error("Attempting to load while saving (not allowed).");
	}

	//read in the number of the current update.
	if(!ifStream.read(reinterpret_cast<char*>(&iUpdate),sizeof(unsigned int))){
		throw std::runtime_error("Read region warp state attempt failed for region " + std::to_string(centerHashBlockId));
	}

	ifStream.read(reinterpret_cast<char*>(externalBuffer),sizeof(Vector3f)*voxelCount*2);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo::BufferPreviousWarpState(void* externalBuffer) {
	ifStream.seekg( -2*(voxelCount*2*sizeof(Vector3f) + sizeof(unsigned int)), std::ios::cur);

	//read in the number of the current update.
	if(!ifStream.read(reinterpret_cast<char*>(&iUpdate),sizeof(unsigned int))){
		throw std::runtime_error("Read region warp state attempt failed for region " + std::to_string(centerHashBlockId));
	}

	ifStream.read(reinterpret_cast<char*>(externalBuffer),sizeof(Vector3f)*voxelCount*2);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo::InterestRegionInfo(
		fs::path path, ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>& parent):
	path(path),
	ifStream(std::ifstream(path.c_str(), std::ios::binary | std::ios::in)),
	parent(parent){

	if (!ifStream) throw std::runtime_error("Could not open " + path.string() + " for reading.");
	hashBlockIds.clear();
	//read region header
	ifStream.read(reinterpret_cast<char*>(&centerHashBlockId), sizeof(unsigned int));
	size_t hashBlockCount;
	ifStream.read(reinterpret_cast<char* >(&hashBlockCount), sizeof(size_t));
	for(int iHashBlockId = 0 ; iHashBlockId < hashBlockCount; iHashBlockId++){
		int hashBlockId;
		ifStream.read(reinterpret_cast<char*>(&hashBlockId), sizeof(int));
		hashBlockIds.push_back(hashBlockId);
	}
	isLoading = true;
	isSaving = false;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::vector<int>& ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo::GetHashBlockIds() const{
	return this->hashBlockIds;
}

