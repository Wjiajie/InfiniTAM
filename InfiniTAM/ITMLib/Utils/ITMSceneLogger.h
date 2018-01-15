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

//local
#include "../Objects/Scene/ITMScene.h"

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
	ITMSceneLogger(std::string path, ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene);
	ITMSceneLogger() = delete;//disable default constructor generation
	virtual ~ITMSceneLogger();
	bool SaveScenes();

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
	bool LoadScenes();
	int GetVoxelCount() const;
	bool GetScenesLoaded() const;

private:

	fs::path path;
	fs::path canonicalPath;
	fs::path livePath;
	fs::path warpUpdatesPath;
	ITMScene<TVoxelCanonical, TIndex>* canonicalScene;
	ITMScene<TVoxelLive, TIndex>* liveScene;
	std::ofstream currentWarpOFStream;
	std::ifstream currentWarpIFStream;
	unsigned int iUpdate = 0;
	int voxelCount = -1;
	Vector3f* warpBuffer = NULL;


};


}//namespace ITMLib


