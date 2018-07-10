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

#include "../../Objects/Scene/ITMVoxelBlockHash.h"
#include "../../Objects/Scene/ITMPlainVoxelArray.h"
#include "../../Objects/Scene/ITMScene.h"

namespace ITMLib{

static const std::string compactFilePostfixAndExtension;

template<typename TVoxel, typename TIndex>
class ITMSceneFileIOEngine;

template<typename TVoxel>
class ITMSceneFileIOEngine<TVoxel,ITMVoxelBlockHash>{
public:
	static void SaveToDirectoryCompact(ITMScene<TVoxel,ITMVoxelBlockHash>* scene, const std::string& outputDirectory);
	static void LoadFromDirectoryCompact(ITMScene<TVoxel,ITMVoxelBlockHash>* scene, const std::string& outputDirectory);
};


template<typename TVoxel>
class ITMSceneFileIOEngine<TVoxel,ITMPlainVoxelArray>{
public:
	static void SaveToDirectoryCompact(ITMScene<TVoxel,ITMPlainVoxelArray>* scene, const std::string& outputDirectory);
	static void LoadFromDirectoryCompact(ITMScene<TVoxel,ITMPlainVoxelArray>* scene, const std::string& outputDirectory);
};



}//namespace ITMLib



