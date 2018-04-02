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
#pragma once

//stdlib
#include <set>


//local
#include "../../Objects/Scene/ITMScene.h"
#include "../ITM3DNestedMapOfArrays.h"
#include "../ITMNeighborVoxelIterationInfo.h"
#include "ITMWarpSceneLogger.h"

//boost
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace ITMLib {


/**
 * \brief Wraps the functionality of saving canonical/live scenes or scene slices for dynamic fusion along
 * with warp changes during optimization between frames.
 * \tparam TVoxelCanonical Type of canonical ("initial"/"source"/"reference") scene voxels
 * \tparam TVoxelLive Type of live ("streaming"/"target") scene voxels
 * \tparam TIndex Type of index used for the voxel scenes
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMSceneLogger {


public:
// region === STATIC CONSTANTS ===

	static const std::string warpUpdatesFilename;
	static const std::string binaryFileExtension;
	static const std::string textFileExtension;
	static const std::string liveName;
	static const std::string canonicalName;
	static const std::string sliceFolderPrefix;
	static const std::string sliceScenePrefix;
// endregion
// region === PUBLIC ENUMS ===

	enum Mode {
		FULL_SCENE,
		SLICE
	};
//endregion
// region === PUBLIC INNER CLASSES ===

	/**
	 * \brief cube-shaped interest region with fixed edge length consistent of hash blocks within the scene
	 */
	class InterestRegionInfo {
		friend class ITMSceneLogger;
	public:
		static const Vector3s blockTraversalOrder[];
		static const std::string prefix;

		InterestRegionInfo(std::vector<int>& hashBlockIds, int centerHashBlockId,
		                   ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>& parent);
		InterestRegionInfo(fs::path path, ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>& parent);


		void SaveCurrentWarpState();

		bool BufferCurrentWarpState(void* externalBuffer);
		bool SeekPrevious();
		bool SeekAt(unsigned int cursor);
		unsigned int GetIterationCursor() const;
		size_t GetIterationWarpBytesize() const;

		const std::vector<int>& GetHashes() const;


		virtual ~InterestRegionInfo();

	private:
		void RewriteHeader();
		// ** member variables **
		bool isLoading = false;
		bool isSaving = false;
		int centerHashBlockId;
		std::vector<int> hashBlockIds;
		fs::path path;
		std::ofstream ofStream;
		std::ifstream ifStream;
		ITMSceneLogger& parent;
		unsigned int iterationCursor = 0;
		int voxelCount;

	};

//endregion
// region === CONSTRUCTORS / DESTRUCTORS ===
	ITMSceneLogger(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	               ITMScene<TVoxelLive, TIndex>* liveScene,
	               std::string path = "");

	ITMSceneLogger() = delete;//disable default constructor generation
	virtual ~ITMSceneLogger();

// endregion
// region === MEMBER FUNCTIONS ===

	//*** setters / preparation
	void SetScenes(ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene);
	void SetPath(std::string path);

	//*** getters
	std::string GetPath() const;
	int GetVoxelCount() const;
	bool GetScenesLoaded() const;
	bool GetInterestRegionsSetUp() const;
	Mode GetMode() const;
	unsigned int GetGeneralIterationCursor() const;
	unsigned int GetInterestIterationCursor() const;
	const std::map<int, std::shared_ptr<InterestRegionInfo>>& GetInterestRegionsByHash();
	ITM3DNestedMapOfArrays<ITMHighlightIterationInfo> GetHighlights() const;
	std::vector<int> GetInterestRegionHashes() const;
	const ITMScene<TVoxelCanonical,TIndex>* GetActiveScene() const;
	const ITMScene<TVoxelLive,TIndex>* GetLiveScene() const;

	//*** scene loading/saving
	bool SaveScenes();
	bool LoadScenes();
	bool SaveScenesCompact();
	bool LoadScenesCompact();

	//*** saving of highlight meta-information & interest regions
	void LogHighlight(int hashId, int voxelLocalIndex, int frameNumber, ITMHighlightIterationInfo info);
	bool SaveHighlights(std::string filePostfix = "");
	void ClearHighlights();
	void PrintHighlights();
	bool LoadHighlights(bool applyFilters = true, std::string filePostfix = "");
	void FilterHighlights(int anomalyFrameCountMinimum);
	void SetUpInterestRegionsForSaving();
	void SetUpInterestRegionsForSaving(const ITM3DNestedMapOfArrays<ITMHighlightIterationInfo>& highlights);
	void SaveAllInterestRegionWarps();
	void SetUpInterestRegionsForLoading();
	bool BufferInterestWarpStateAtIteration(void* externalBuffer, unsigned int iterationIndex);
	bool BufferCurrentInterestWarpState(void* externalBuffer);
	bool BufferPreviousInterestWarpState(void* externalBuffer);
	bool IsHashInInterestRegion(int hashId);
	int GetTotalInterestVoxelCount();

	//** canonical warp-state saving/loading
	bool StartSavingWarpState(unsigned int frameIx);
	void StopSavingWarpState();
	bool StartLoadingWarpState();
	bool StartLoadingWarpState(unsigned int& frameIx);
	void StopLoadingWarpState();

	bool SaveCurrentWarpState();
	bool LoadCurrentWarpState();
	bool BufferCurrentWarpState(void* externalBuffer);
	bool BufferPreviousWarpState(void* externalBuffer);
	bool BufferWarpStateAt(void* externalBuffer, unsigned int iterationIndex);
	bool LoadPreviousWarpState();

	bool IsLoadingWarpState();

	//*** slice generation & usage***
	fs::path GenerateSliceFolderPath(const Vector3i& minPoint, const Vector3i& maxPoint) const;
	fs::path GenerateSliceFolderPath(const std::string& sliceIdentifier) const;
	std::string GenerateSliceSceneFilename_UpToPostfix(const Vector3i& minPoint, const Vector3i& maxPoint) const;
	std::string GenerateSliceSceneFilename_Full(const Vector3i& minPoint, const Vector3i& maxPoint) const;
	std::string GenerateSliceSceneFilename_Full(const std::string& sliceIdentifier) const;
	std::string GenerateSliceWarpFilename(const Vector3i& minPoint, const Vector3i& maxPoint) const;
	std::string GenerateSliceWarpFilename(const std::string& sliceIdentifier) const;
	bool MakeSlice(const Vector3i& extremum1, const Vector3i& extremum2,
		               unsigned int frameIndex);
	bool SliceExistsInMemory(const std::string& sliceIdentifier) const;
	bool SliceExistsOnDisk(const Vector3i& extremum1,
	                       const Vector3i& extremum2) const;

	bool SliceExistsOnDisk(const std::string& sliceIdentifier) const;
	bool LoadSlice(const std::string& sliceIdentifier,
	               ITMScene<TVoxelCanonical, TIndex>* destinationScene);
	bool SwitchActiveScene(
			std::string sliceIdentifier = ITMWarpSceneLogger<TVoxelCanonical, TIndex>::fullSceneSliceIdentifier);

//endregion
private:
// region === CONSTANTS ===

	static const std::string highlightFilterInfoFilename;
	static const std::string minRecurrenceHighlightFilterName;
// endregion
// region === MEMBER FUNCTIONS ===
	void SaveSliceWarp(const Vector3i& minPoint, const Vector3i& maxPoint,
	                   unsigned int frameIndex, std::string path);
	bool CheckPath();

// endregion
// region === MEMBER VARIABLES ===

// *** root folder
	fs::path path;

// *** subpaths
	fs::path livePath;
	fs::path highlightsBinaryPath;
	fs::path highlightsTextPath;

// *** scene structures ***
	ITMWarpSceneLogger<TVoxelCanonical, TIndex>* fullCanonicalSceneLogger;
	ITMWarpSceneLogger<TVoxelCanonical, TIndex>* activeWarpLogger;
	ITMScene<TVoxelLive, TIndex>* liveScene;

// *** scene meta-information + reading/writing
	// map of hash blocks to voxels, voxels to frame numbers, frame numbers to iteration numbers
	ITM3DNestedMapOfArrays<ITMHighlightIterationInfo> highlights;
	std::map<int, std::shared_ptr<InterestRegionInfo>> interestRegionInfoByHashId;
	std::vector<std::shared_ptr<InterestRegionInfo>> interestRegionInfos;
	std::map<std::string, ITMWarpSceneLogger<TVoxelCanonical, TIndex>*> slices;

// *** data manipulation information ***
	int minHighlightRecurrenceCount = 0;

// *** state ***
	Mode mode;
	bool interestRegionsHaveBeenSetUp = false;

	//TODO: the way these update numbers are tracked are less than ideal (see comment below) -Greg (GitHub: Algomorph)
	// There is no way to ensure proper iteration number, since it is not kept track of in the scene.
	// It would be ideal to extend the scene class and log that number there, since it reflects the state of the scene.
	unsigned int interestIterationCursor = 0;


// endregion
};


}//namespace ITMLib


