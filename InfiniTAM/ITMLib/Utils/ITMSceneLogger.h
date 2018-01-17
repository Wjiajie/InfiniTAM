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
#include <unordered_set>
#include <unordered_map>


//local
#include "../Objects/Scene/ITMScene.h"
#include "ITMIntArrayMap3D.h"


//TODO: eventually replace boost::filesystem with stdlib filesystem when that is no longer experimental -Greg (GitHub: Algomorph)
//TODO: add HAVE_BOOST guards -Greg (GitHub: Algomorph)

//boost
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;


namespace ITMLib{

/**
 * \brief Wraps the functionality of saving canonical/live scenes for dynamic fusion along with warp changes during optimization between frames.
 * \tparam TVoxelCanonical Type of canonical ("initial"/"source"/"reference") scene voxels
 * \tparam TVoxelLive Type of live ("streaming"/"target") scene voxels
 * \tparam TIndex Type of index used for the voxel scenes
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMSceneLogger {

public:
	//*** constructors/destructors
	ITMSceneLogger(std::string path, ITMScene<TVoxelCanonical, TIndex>* canonicalScene=NULL, ITMScene<TVoxelLive, TIndex>* liveScene=NULL);
	ITMSceneLogger() = delete;//disable default constructor generation
	virtual ~ITMSceneLogger();

	//*** setters / preparation

	void SetScenes(ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene);

	//*** scene loading/saving
	bool SaveScenes();
	bool LoadScenes();

	//*** information getters
	int GetVoxelCount() const;
	bool GetScenesLoaded() const;

	//*** saving of meta-information
	void LogHighlight(int hashId, int voxelLocalIndex, int frameNumber, int iterationNumber);
	bool SaveHighlights();
	void PrintHighlights();
	bool LoadHighlights();


	//** warp-state saving/loading
	bool StartSavingWarpState();
	bool SaveCurrentWarpState();
	void StopSavingWarpState();
	bool StartLoadingWarpState();
	bool LoadNextWarpState();
	bool BufferNextWarpState();
	bool BufferPreviousWarpState();
	bool BufferNextWarpState(void* externalBuffer);
	bool BufferPreviousWarpState(void* externalBuffer);
	bool LoadPreviousWarpState();
	void StopLoadingWarpState();
	bool IsLoadingWarpState();
	bool CopyWarpBuffer(float* warpDestination,
		                    float* warpUpdateDestination, int& iUpdate);
	bool CopyWarpAt(int index, float voxelWarpDestination[3]) const;
	bool CopyWarpAt(int index, float voxelWarpDestination[3], float voxelUpdateDestination[3]) const;
	const float* WarpAt(int index) const;
	const float* UpdateAt(int index) const;

private:

// *** root folder
	fs::path path;
// *** canonical/live scene saving/loading
	fs::path canonicalPath;
	fs::path livePath;
	ITMScene<TVoxelCanonical, TIndex>* canonicalScene;
	ITMScene<TVoxelLive, TIndex>* liveScene;
// *** scene meta-information + reading/writing
	int voxelCount = -1;
// map of hash blocks to voxels, voxels to frame numbers, frame numbers to iteration numbers
	ITMIntArrayMap3D highlights;
	fs::path highlightsPath;


// *** optimization warp-updates reading/writing
	fs::path warpUpdatesPath;
	std::ofstream currentWarpOFStream;
	std::ifstream currentWarpIFStream;
	unsigned int iUpdate = 0;
	Vector3f* warpBuffer = NULL;

};


}//namespace ITMLib


