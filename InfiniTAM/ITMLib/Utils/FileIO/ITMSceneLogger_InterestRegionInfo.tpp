//  ================================================================
//  Created by Gregory Kramida on 3/7/18.
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
#include "ITMSceneLogger.h"


// *********************************************************************************************************************
// ********************************* MEMBERS OF THE INNER InterestRegionInfo CLASS *************************************
// *********************************************************************************************************************
// region ======================================== CONSTANTS ===========================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::string ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo::prefix = "region_";

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const Vector3s ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo::blockTraversalOrder[] =
		{Vector3s(-1, -1, -1), Vector3s(-1, -1, 0), Vector3s(-1, -1, 1),
		 Vector3s(-1, 0, -1), Vector3s(-1, 0, 0), Vector3s(-1, 0, 1),
		 Vector3s(-1, 1, -1), Vector3s(-1, 1, 0), Vector3s(-1, 1, 1),

		 Vector3s(0, -1, -1), Vector3s(0, -1, 0), Vector3s(0, -1, 1),
		 Vector3s(0, 0, -1), Vector3s(0, 0, 0), Vector3s(0, 0, 1),
		 Vector3s(0, 1, -1), Vector3s(0, 1, 0), Vector3s(0, 1, 1),

		 Vector3s(1, -1, -1), Vector3s(1, -1, 0), Vector3s(1, -1, 1),
		 Vector3s(1, 0, -1), Vector3s(1, 0, 0), Vector3s(1, 0, 1),
		 Vector3s(1, 1, -1), Vector3s(1, 1, 0), Vector3s(1, 1, 1)};
//endregion
// region ======================================== CONSTRUCTORS & DESTRUCTORS ==========================================

/**
 * \brief General-purpose destructor
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo::~InterestRegionInfo() {
	ofStream.close();
	ifStream.close();
}

/**
 * \brief Constructor used to make an interest region info for saving warps in a specific hash block neighborhood to disk
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
		ofStream(),
		parent(parent),
		voxelCount(static_cast<int>(hashBlockIds.size()) * SDF_BLOCK_SIZE3) {
	this->RewriteHeader();
	isSaving = true;
	isLoading = false;
}

/**
 * \brief Constructor used for reading/loading interest region info from disk
 * \param path - path to file
 * \param parent - parent loader
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo::InterestRegionInfo(
		fs::path path, ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>& parent):
		path(path),
		ifStream(std::ifstream(path.c_str(), std::ios::binary | std::ios::in)),
		parent(parent) {

	voxelCount = 0;
	if (!ifStream)
		throw std::runtime_error(
				"Could not open " + path.string() + " for reading. ["  __FILE__  ": " + std::to_string(__LINE__) + "]");
	hashBlockIds.clear();
	//read region header
	ifStream.read(reinterpret_cast<char*>(&centerHashBlockId), sizeof(unsigned int));
	size_t hashBlockCount;
	ifStream.read(reinterpret_cast<char* >(&hashBlockCount), sizeof(size_t));
	for (int iHashBlockId = 0; iHashBlockId < hashBlockCount; iHashBlockId++) {
		int hashBlockId;
		ifStream.read(reinterpret_cast<char*>(&hashBlockId), sizeof(int));
		hashBlockIds.push_back(hashBlockId);
		voxelCount += SDF_BLOCK_SIZE3;
	}
	isLoading = true;
	isSaving = false;
}
//endregion
// region ======================================== GENERAL INFORMATION GETTERS =========================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::vector<int>& ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo::GetHashes() const {
	return this->hashBlockIds;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
unsigned int ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo::GetIterationCursor() const {
	return this->iterationCursor;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
size_t ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo::GetIterationWarpBytesize() const {
	return this->voxelCount * 2 * sizeof(Vector3f);
}

//endregion
// region ======================================== READING WARP STATE IN THE INTEREST REGION ===========================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo::BufferCurrentWarpState(void* externalBuffer) {
	if (isSaving) {
		throw std::runtime_error(
				"Attempting to load while saving (not allowed).["  __FILE__  ": " + std::to_string(__LINE__) + "]");
	}
	if (ifStream.peek() == EOF) {
		std::cout << "At end of warp file." << std::endl;
		return false;
	}
	//read in the number of the current update.
	int iIteration;
	if (!ifStream.read(reinterpret_cast<char*>(&iIteration), sizeof(unsigned int))) { return false; }
	ifStream.read(reinterpret_cast<char*>(externalBuffer), sizeof(Vector3f) * voxelCount * 2);
	this->iterationCursor++;
	return !(ifStream.fail() || ifStream.bad());;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo::SeekPrevious() {
	if (isSaving) {
		throw std::runtime_error(
				"Attempting to load while saving (not allowed).["  __FILE__  ": " + std::to_string(__LINE__) + "]");
	}
	if (iterationCursor == 0) { return false; }
	if (ifStream.eof()) {
		ifStream.clear();
	}
	ifStream.seekg(-1 * (voxelCount * 2 * sizeof(Vector3f) + sizeof(unsigned int)), std::ios::cur);
	this->iterationCursor--;
	return !(ifStream.fail() || ifStream.bad());
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo::SeekAt(unsigned int cursor) {
	if (isSaving) {
		throw std::runtime_error(
				"Attempting to load while saving (not allowed).["  __FILE__  ": " + std::to_string(__LINE__) + "]");
	}
	if (ifStream.eof()) {
		ifStream.clear();
	}
	size_t headerSize = sizeof(unsigned int) + sizeof(size_t) + hashBlockIds.size()*sizeof(int);
	ifStream.seekg(headerSize + cursor*(voxelCount*2*sizeof(Vector3f) + sizeof(unsigned int)), std::ios::beg);
	this->iterationCursor = cursor;
	return !(ifStream.fail() || ifStream.bad());
}
//endregion
// region ======================================== WRITING OF THE INTEREST REGION TO DISK ==============================

/**
 * \brief (Re)writes the interest region header (only!) to disk. Will erase any warp data in the file, if it is there.
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo::RewriteHeader() {
	ofStream.close();
	ofStream = std::ofstream(path.c_str(), std::ios::binary | std::ios::out);
	if (!ofStream)
		throw std::runtime_error("Could not open " + path.string()
		                         + " for writing. ["  __FILE__  ": " + std::to_string(__LINE__) + "]");
	//write region header
	ofStream.write(reinterpret_cast<const char* >(&centerHashBlockId), sizeof(unsigned int));
	size_t hashBlockCount = hashBlockIds.size();
	ofStream.write(reinterpret_cast<const char* >(&hashBlockCount), sizeof(size_t));
	for (int hashBlockId : hashBlockIds) {
		ofStream.write(reinterpret_cast<const char* >(&hashBlockId), sizeof(int));
	}
}

/**
 * \brief Saves the current warp state for the interest region, advances the local iteration cursor
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo::SaveCurrentWarpState() {
	if (isLoading) {
		throw std::runtime_error(
				"Attempting to save region made for loading (not allowed). Use alternative constructor.["  __FILE__  ": " +
				std::to_string(__LINE__) + "]");
	}

	ofStream.write(reinterpret_cast<const char* >(&iterationCursor), sizeof(unsigned int));
	const TVoxelCanonical* voxels = parent.activeWarpLogger->scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashBlocks = parent.activeWarpLogger->scene->index.GetEntries();
	for (int iHashBlock : hashBlockIds) {
		const ITMHashEntry& currentHashBlock = hashBlocks[iHashBlock];
		if (currentHashBlock.ptr < 0) continue;
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
	iterationCursor++;
}
//endregion
// *********************************************************************************************************************
// ********** MEMBERS OF THE SCENE LOGGER CLASS DIRECTLY PERTAINING TO INTEREST REGION MANAGEMENT **********************
// *********************************************************************************************************************
// region ======================================== SAVING SETUP & SAVING WARPS =========================================

/**
 * \brief Set
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SetUpInterestRegionsForSaving() {
	ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SetUpInterestRegionsForSaving(this->highlights);
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
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SetUpInterestRegionsForSaving(
		const ITM3DNestedMapOfArrays<ITMHighlightIterationInfo>& highlights) {
	if (this->activeWarpLogger->Empty()) {
		DIEWITHEXCEPTION("Attempted to set up interest regions before loading the scenes. Aborting. ["
				                 __FILE__
				                 ": " + std::to_string(__LINE__) + "]");
	}
	interestRegionInfos.clear();
	interestRegionInfoByHashId.clear();

	const ITMHashEntry* hashBlocks = activeWarpLogger->scene->index.GetEntries();
	int hashBlockCount = activeWarpLogger->scene->index.noTotalEntries;

	//traverse hash blocks where anomalies/errors/oscillations occur
	for (int centerHashId : highlights.GetOuterLevelKeys()) {
		const ITMHashEntry& currentHashBlock = hashBlocks[centerHashId];
		if (currentHashBlock.ptr < 0) {
			throw std::runtime_error("Got hash Id " + std::to_string(centerHashId) +
			                         " in the highlights that doesn't correspond to a populated block in the scene. ["
					                         __FILE__  ": " + std::to_string(__LINE__) + "]");
		}
		std::vector<int> regionHashIds;
		Vector3s centerBlockPos = currentHashBlock.pos;

		//traverse neighborhood of the interest hash block in a predefined order
		std::shared_ptr<InterestRegionInfo> overlappingRegion;
		std::set<int> nonOverlappingHashes;
		overlappingRegion.reset();
		for (Vector3s offset : InterestRegionInfo::blockTraversalOrder) {
			int hash = FindHashBlock(hashBlocks, centerBlockPos + offset);
			if (hash >= 0 && hash < hashBlockCount && hashBlocks[hash].ptr >= 0) {
				regionHashIds.push_back(hash);
				//hash is in another interest region, a merge is necessary to ensure there is no overlap between regions
				if (interestRegionInfoByHashId.find(hash) != interestRegionInfoByHashId.end()) {
					overlappingRegion = interestRegionInfoByHashId[hash];
					overlappingRegion->hashBlockIds.push_back(hash);
					overlappingRegion->voxelCount += SDF_BLOCK_SIZE3;
				} else {
					nonOverlappingHashes.insert(hash);
				}
			}
		}
		if (overlappingRegion) {
			//overlapping with 'some other region(s)', insert the remaining hashes into one of them
			for (int hash : nonOverlappingHashes) {
				overlappingRegion->hashBlockIds.push_back(hash);
				overlappingRegion->voxelCount += SDF_BLOCK_SIZE3;
			}
			overlappingRegion->RewriteHeader();
		} else {
			std::shared_ptr<InterestRegionInfo> info(new InterestRegionInfo(regionHashIds, centerHashId, *this));
			//instert the same region into map by the hash blocks it contains
			for (int regionHashBlockId : regionHashIds) {
				interestRegionInfoByHashId[regionHashBlockId] = info;
			}
			interestRegionInfos.push_back(info);
		}
	}
	interestRegionsHaveBeenSetUp = true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SaveAllInterestRegionWarps() {
	for (std::shared_ptr<InterestRegionInfo> info: interestRegionInfos) {
		info->SaveCurrentWarpState();
	}
	if (interestRegionInfos.size() > 0) {
		interestIterationCursor = interestRegionInfos[0]->GetIterationCursor();
	}
	std::cout << "Saved all interest region warps." << std::endl;
}
//endregion
// region ======================================== LOADING SETUP =======================================================
/**
 * \brief Set up all interest regions for loading based on the files in current active directory for the loader.
 * \details The files should follow the interest region naming convention
 * \tparam TVoxelCanonical
 * \tparam TVoxelLive
 * \tparam TIndex
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::SetUpInterestRegionsForLoading() {
	if (activeWarpLogger->Empty()) {
		DIEWITHEXCEPTION("Attempted to set up interest regions before loading the scenes. Aborting.");
	}
	interestRegionInfos.clear();
	interestRegionInfoByHashId.clear();

	std::regex regionFileRegex(InterestRegionInfo::prefix + "\\d+" + binaryFileExtension);
	std::smatch match;
	std::vector<fs::path> regionPaths;
	for (fs::directory_iterator itr{path}; itr != fs::directory_iterator{}; itr++) {
		std::string filename = itr->path().filename().string();
		if (fs::is_regular_file(itr->path()) && std::regex_match(filename, match, regionFileRegex)) {
			std::shared_ptr<InterestRegionInfo> info(new InterestRegionInfo(itr->path(), *this));
			for (const int iHashBlockId : info->GetHashes()) {
				interestRegionInfoByHashId[iHashBlockId] = info;
			}
			interestRegionInfos.push_back(info);
		}
	}

	interestRegionsHaveBeenSetUp = true;
}
//endregion
//region ======================================== INFORMATION GETTERS / SETTERS =======================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
int ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetTotalInterestVoxelCount() {
	int total = 0;
	for (std::shared_ptr<InterestRegionInfo> info: interestRegionInfos) {
		total += info->voxelCount;
	}
	return total;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetInterestRegionsSetUp() const {
	return this->interestRegionsHaveBeenSetUp;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
typename ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::Mode
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetMode() const {
	return mode;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const std::map<int, std::shared_ptr<typename ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::InterestRegionInfo>>&
ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetInterestRegionsByHash() {
	return this->interestRegionInfoByHashId;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::vector<int> ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::GetInterestRegionHashes() const {
	std::vector<int> hashes;
	for (std::shared_ptr<InterestRegionInfo> info: interestRegionInfos) {
		for (int hash : info->GetHashes()) {
			hashes.push_back(hash);
		}
	}
	return hashes;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::IsHashInInterestRegion(int hashId) {
	return (this->interestRegionInfoByHashId.find(hashId) == this->interestRegionInfoByHashId.end());
}
//endregion
// region ======================================== INTEREST REGION WARP TRAVERSAL ======================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::BufferInterestWarpStateAtIteration(void* externalBuffer,
                                                                                             unsigned int iterationIndex) {
	for(std::shared_ptr<InterestRegionInfo> info: interestRegionInfos){
		if(!info->SeekAt(iterationIndex)){
			return false;
		}
	}
	interestIterationCursor = iterationIndex;
	return BufferCurrentInterestWarpState(externalBuffer);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::BufferCurrentInterestWarpState(void* externalBuffer) {
	char* cursor = reinterpret_cast<char*>(externalBuffer);
	for (std::shared_ptr<InterestRegionInfo> info: interestRegionInfos) {
		if (!info->BufferCurrentWarpState(cursor)) {
			return false;
		}
		cursor += info->GetIterationWarpBytesize();
		interestIterationCursor = info->GetIterationCursor();
	}
	return true;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>::BufferPreviousInterestWarpState(void* externalBuffer) {
	char* cursor = reinterpret_cast<char*>(externalBuffer);
	if (interestIterationCursor == 0) { return false; } //at beginning of stream
	if (interestIterationCursor == 1) {
		for (std::shared_ptr<InterestRegionInfo> info: interestRegionInfos) {
			if (!info->SeekPrevious()) { return false; }
			cursor += info->GetIterationWarpBytesize();
			interestIterationCursor = info->GetIterationCursor();
		}
	} else {
		for (std::shared_ptr<InterestRegionInfo> info: interestRegionInfos) {
			if (!info->SeekPrevious() || !info->SeekPrevious() || !info->BufferCurrentWarpState(cursor)) {
				return false;
			}
			cursor += info->GetIterationWarpBytesize();
			interestIterationCursor = info->GetIterationCursor();
		}
	}
	return true;
}
//endregion


