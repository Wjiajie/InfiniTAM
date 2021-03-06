//  ================================================================
//  Created by Gregory Kramida on 7/10/18.
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

#include "../../Objects/Scene/VoxelBlockHash.h"
#include "../../Objects/Scene/PlainVoxelArray.h"
#include "../../Objects/Scene/ITMVoxelVolume.h"

namespace ITMLib{

static const std::string compactFilePostfixAndExtension = "compact.dat";

template<typename TVoxel, typename TIndex>
class ITMSceneFileIOEngine;

template<typename TVoxel>
class ITMSceneFileIOEngine<TVoxel,VoxelBlockHash>{
public:
	static void SaveToDirectoryCompact(const ITMVoxelVolume<TVoxel,VoxelBlockHash>* scene, const std::string& outputDirectory);
	static void LoadFromDirectoryCompact(ITMVoxelVolume<TVoxel,VoxelBlockHash>* scene, const std::string& outputDirectory);
};


template<typename TVoxel>
class ITMSceneFileIOEngine<TVoxel,PlainVoxelArray>{
public:
	static void SaveToDirectoryCompact(const ITMVoxelVolume<TVoxel,PlainVoxelArray>* scene, const std::string& outputDirectory);
	static void LoadFromDirectoryCompact(ITMVoxelVolume<TVoxel,PlainVoxelArray>* scene, const std::string& outputDirectory);
};



}//namespace ITMLib



