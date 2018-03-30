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
	// region ================================ STATIC CONSTANTS ========================================================

	static const size_t warpByteSize;
	static const size_t updateByteSize;
	static const size_t warpAndUpdateByteSize;
	static const std::string fullSceneSliceIdentifier;
	//endregion
	// region ================================ STATIC FUNCTIONS ========================================================

	static std::string GenerateSliceStringIdentifier(const Vector3i& minPoint, const Vector3i& maxPoint);

	// endregion
	// region ================================ CONSTRUCTORS & DESTRUCTORS ==============================================

	explicit ITMWarpSceneLogger(bool isSlice,
	                            ITMScene <TVoxel, TIndex>* scene = nullptr,
	                            std::string scenePath = "", std::string warpPath = "");
	~ITMWarpSceneLogger();

	// endregion
	// region ================================ MEMBER FUNCTIONS ========================================================

	//*** getters / setters ***
	unsigned int GetIterationCursor() const;
	bool SetIterationCursor(unsigned int iterationIndex);
	int GetVoxelCount() const;
	bool Empty() const;
	bool Loaded() const;
	void Load();
	std::string GetSliceIdentifier() const;
	const ITMScene<TVoxel,TIndex>* GetScene() const;

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
	bool LoadCurrentWarpState();
	bool StartLoadingWarpState(unsigned int& frameIx);
	bool StartLoadingWarpState();
	void StopLoadingWarpState();
	bool IsLoadingWarpState();
	// endregion
private:
	// region ================================ MEMBER VARIABLES ========================================================

	std::string scenePath;
	std::string warpPath;
	ITMScene<TVoxel, TIndex>* scene;

	unsigned int iterationCursor = 0;
	int voxelCount = -1;

	// *** optimization warp-updates reading/writing

	std::ofstream warpOFStream;
	std::ifstream warpIFStream;

	// *** slice parameters (optional)
	bool isSlice = false;
	Vector3i minimum;
	Vector3i maximum;
	//endregion
};
}//namespace ITMLib

