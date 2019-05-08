//  ================================================================
//  Created by Gregory Kramida on 5/8/19.
//  Copyright (c) 2019 Gregory Kramida
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

#include "ITMScene.h"

namespace ITMLib {

/**
 * \brief Generate a new scene (Caution: does not reset / initialize the voxel storage itself)
 * \param _sceneParams scene parameters (\see ITMSceneParams definition)
 * \param _useSwapping whether or not to use the swapping mechanism (TODO: better explanation of the swapping)
 * \param _memoryType DRAM to use -- GPU or CPU
 * \param size (optional) size of the scene -- affects only bounded index types, such as ITMPlainVoxelArray
 * \param offset (optional) offset of the scene -- affects only bounded index types, such as ITMPlainVoxelArray
 */
template<typename TVoxel, typename TIndex>
ITMScene<TVoxel,TIndex>::ITMScene(const ITMSceneParams* _sceneParams, bool _useSwapping, MemoryDeviceType _memoryType,
               Vector3i size, Vector3i offset)
	: sceneParams(_sceneParams),
		index(_memoryType, size, offset),
	  localVBA(_memoryType, index.getNumAllocatedVoxelBlocks(), index.getVoxelBlockSize())
{
	if (_useSwapping) globalCache = new ITMGlobalCache<TVoxel>();
	else globalCache = nullptr;
}

}  // namespace ITMLib