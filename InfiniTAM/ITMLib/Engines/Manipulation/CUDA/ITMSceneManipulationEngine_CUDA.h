//  ================================================================
//  Created by Gregory Kramida on 7/24/18.
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

#include "../../../Objects/Scene/ITMPlainVoxelArray.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"


namespace ITMLib {
template<typename TVoxel, typename TIndex>
class ITMSceneManipulationEngine_CUDA {
};


template<typename TVoxel>
class ITMSceneManipulationEngine_CUDA<TVoxel, ITMPlainVoxelArray> {
public:
	static void ResetScene(ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* scene);
	static bool SetVoxel(ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* scene, Vector3i at, TVoxel voxel);
	static TVoxel ReadVoxel(ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* scene, Vector3i at);
	static TVoxel
	ReadVoxel(ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* scene, Vector3i at, ITMPlainVoxelArray::IndexCache& cache);
	static void OffsetWarps(ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* scene, Vector3f offset);
	/**
	 * \brief Copies the slice (box-like window) specified by points extremum1 and extremum2 from the source scene into a
	 * destination scene. Clears the destination scene before copying.
	 * \tparam TVoxel type of voxel
	 * \tparam TIndex type of voxel index
	 * \param destination destination voxel grid (can be uninitialized)
	 * \param source source voxel grid
	 * \param bounds minimum point in the desired slice (inclusive), i.e. minimum x, y, and z coordinates
	 * \param maxPoint maximum point in the desired slice (inclusive), i.e. maximum x, y, and z coordinates
	 * \return true on success (destination scene contains the slice), false on failure (there are no allocated hash blocks
	 */
	static bool CopySceneSlice(ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* destination,
	                           ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* source,
	                           Vector6i bounds);
};


template<typename TVoxel>
class ITMSceneManipulationEngine_CUDA<TVoxel, ITMVoxelBlockHash> {
public:
	/**
	 * \brief Clear out scene and reset the index
	 * \param scene
	 */
	static void ResetScene(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene);
	static bool SetVoxel(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, Vector3i at, TVoxel voxel);
	static TVoxel ReadVoxel(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, Vector3i at);
	static TVoxel
	ReadVoxel(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, Vector3i at, ITMVoxelBlockHash::IndexCache& cache);
	/**
	 * \brief offset warps by a fixed amount in each direction
	 * \param scene the scene to modify
	 * \param offset the offset vector to use
	 */
	static void OffsetWarps(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, Vector3f offset);
	/**
	 * \brief Copies the slice (box-like window) specified by points extremum1 and extremum2 from the source scene into a
	 * destination scene. Clears the destination scene before copying.
	 * \tparam TVoxel type of voxel
	 * \tparam TIndex type of voxel index
	 * \param destination destination voxel grid (can be uninitialized)
	 * \param source source voxel grid
	 * \param bounds minimum point in the desired slice (inclusive), i.e. minimum x, y, and z coordinates
	 * \param maxPoint maximum point in the desired slice (inclusive), i.e. maximum x, y, and z coordinates
	 * \return true on success (destination scene contains the slice), false on failure (there are no allocated hash blocks
	 */
	static bool
	CopySceneSlice(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* destination, ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* source,
	               Vector6i bounds);

};



}//namespace ITMLib