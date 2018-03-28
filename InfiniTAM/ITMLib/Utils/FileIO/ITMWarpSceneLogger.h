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
#pragma once


#include "../../Objects/Scene/ITMScene.h"
//boost
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;


namespace ITMLib{

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMSceneLogger;

template<typename TVoxel, typename TIndex>
class ITMWarpSceneLogger{
	template<typename TVoxelCanonical, typename TVoxelLive, typename TIndexLogger>
	friend class ITMSceneLogger;
public:

	~ITMWarpSceneLogger();

	//*** getters ***
	unsigned int GetIterationCursor() const;
	int GetVoxelCount() const;
	bool Empty() const;
	bool Loaded() const;
	void Load();

	//*** scene saving / loading ***
	void Save();
	void SaveCompact();
	void LoadCompact();

	//*** warp loading / saving / buffering ***
	bool StartSavingWarpState(unsigned int frameIx);
	void StopSavingWarpState();
	bool SaveCurrentWarpState();
	bool LoadPreviousWarpState();
	bool BufferWarpStateAt(void* externalBuffer, unsigned int iterationIndex);
	bool BufferPreviousWarpState(void* externalBuffer);
	bool BufferCurrentWarpState(void* externalBuffer);
	const float* UpdateAt(int index) const;
	const float* WarpAt(int index) const;
	bool CopyWarpAt(int index, float* voxelWarpDestination, float* voxelUpdateDestination) const;
	bool CopyWarpAt(int index, float* voxelWarpDestination) const;
	bool CopyWarpBuffer(float* warpDestination, float* warpUpdateDestination, int& iUpdate);
	bool BufferPreviousWarpState();
	bool BufferNextWarpState();
	bool LoadCurrentWarpState();
	bool StartLoadingWarpState(unsigned int& frameIx);
	bool StartLoadingWarpState();
	void StopLoadingWarpState();
	bool IsLoadingWarpState();

private:
	explicit ITMWarpSceneLogger(bool isSlice,
	                            ITMScene <TVoxel, TIndex>* scene = nullptr,
	                            std::string scenePath = "", std::string warpPath = "");

	std::string scenePath;
	std::string warpPath;
	ITMScene<TVoxel, TIndex>* scene;

	unsigned int generalIterationCursor = 0;
	int voxelCount = -1;
	Vector3f* warpBuffer = nullptr;

	// *** optimization warp-updates reading/writing

	std::ofstream warpOFStream;
	std::ifstream warpIFStream;

	// *** slice parameters (optional)
	bool isSlice = false;
	Vector3i minimum;
	Vector3i maximum;

};
}//namespace ITMLib

