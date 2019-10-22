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

#include "ITMVoxelVolume.h"
#include "../../Engines/SceneFileIO/ITMSceneFileIOEngine.h"

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
ITMVoxelVolume<TVoxel,TIndex>::ITMVoxelVolume(const ITMSceneParams* _sceneParams, bool _useSwapping, MemoryDeviceType _memoryType,
                                              Vector3i size, Vector3i offset)
	: sceneParams(_sceneParams),
		index(_memoryType, size, offset),
	  localVBA(_memoryType, index.getNumAllocatedVoxelBlocks(), index.getVoxelBlockSize())
{
	if (_useSwapping) globalCache = new ITMGlobalCache<TVoxel>();
	else globalCache = nullptr;
}

template<class TVoxel, class TIndex>
ITMVoxelVolume<TVoxel, TIndex>::ITMVoxelVolume(const ITMVoxelVolume& other, MemoryDeviceType _memoryType)
	: sceneParams(other.sceneParams),
	index(other.index,_memoryType),
	localVBA(other.localVBA, _memoryType){
	if(other.globalCache != nullptr){
		// TODO: not sure if global cache needs to be shared or copied between copied scenes
		globalCache = new ITMGlobalCache<TVoxel>();
	}else{
		globalCache = nullptr;
	}
}

template<class TVoxel, class TIndex>
void ITMVoxelVolume<TVoxel, TIndex>::SetFrom(const ITMVoxelVolume& other) {
	index.SetFrom(other.index);
	localVBA.SetFrom(other.localVBA);
	if(other.globalCache != nullptr){
		// TODO: not sure if global cache needs to be shared or copied between copied scenes
		globalCache = new ITMGlobalCache<TVoxel>();
	}else{
		globalCache = nullptr;
	}
}

template<class TVoxel, class TIndex>
void ITMVoxelVolume<TVoxel, TIndex>::SaveToDirectory(const std::string& outputDirectory) const {
	ITMSceneFileIOEngine<TVoxel,TIndex>::SaveToDirectoryCompact(this, outputDirectory);
}

template<class TVoxel, class TIndex>
void ITMVoxelVolume<TVoxel, TIndex>::LoadFromDirectory(const std::string& outputDirectory) {
	ITMSceneFileIOEngine<TVoxel,TIndex>::LoadFromDirectoryCompact(this, outputDirectory);
}

}  // namespace ITMLib