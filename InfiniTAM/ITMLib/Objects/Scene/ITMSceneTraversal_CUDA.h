//  ================================================================
//  Created by Gregory Kramida on 7/26/18.
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

#include "ITMScene.h"
#include "ITMPlainVoxelArray.h"



namespace {
//CUDA device functions
template<typename TStaticFunctor, typename TVoxel>
__global__ void
staticVoxelTraversal_device(TVoxel* voxelArray, const ITMLib::ITMPlainVoxelArray::ITMVoxelArrayInfo* arrayInfo) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;
	TVoxel& voxel = voxelArray[locId];
	TStaticFunctor::run(voxel);
}
}// namespace

namespace ITMLib{
// region ================================ STATIC SINGLE-SCENE TRAVERSAL ===============================================
template<typename TStaticFunctor, typename TVoxel>
inline void StaticVoxelTraversal_CPU(ITMScene <TVoxel, ITMPlainVoxelArray>* scene) {
	TVoxel* voxelArray = scene->localVBA.GetVoxelBlocks();
	const ITMPlainVoxelArray::ITMVoxelArrayInfo* arrayInfo = scene->index.getIndexData();

	dim3 cudaBlockSize(8, 8, 8);
	dim3 gridSize(scene->index.getVolumeSize().x / cudaBlockSize.x,
	              scene->index.getVolumeSize().y / cudaBlockSize.y,
	              scene->index.getVolumeSize().z / cudaBlockSize.z);

	staticVoxelTraversal_device < TStaticFunctor, TVoxel> << <gridSize, cudaBlockSize >> >(voxelArray, arrayInfo);
};
// endregion ===========================================================================================================
}