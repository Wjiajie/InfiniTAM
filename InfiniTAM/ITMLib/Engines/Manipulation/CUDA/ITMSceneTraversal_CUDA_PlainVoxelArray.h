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

//stdlib
#include <cassert>

//local
#include "../Interface/ITMSceneTraversal.h"
#include "../../../Objects/Scene/ITMScene.h"
#include "../../../Objects/Scene/ITMPlainVoxelArray.h"
#include "../../../Utils/ITMLibSettings.h"


/* NOTE: this functionality is purposefully kept in headers w/o explicit instantiations to allow for further inlining by
 * the compiler*/


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

template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
__global__ void
dualVoxelPositionTraversal_device(TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
                                  const ITMLib::ITMPlainVoxelArray::ITMVoxelArrayInfo* arrayInfo,
                                  TFunctor& functor) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;

	Vector3i voxelPosition;

	voxelPosition.x = x + arrayInfo->offset.x;
	voxelPosition.y = y + arrayInfo->offset.y;
	voxelPosition.z = z + arrayInfo->offset.z;

	TVoxelPrimary& voxelPrimary = primaryVoxels[locId];
	TVoxelSecondary& voxelSecondary = secondaryVoxels[locId];
	functor(voxelPrimary, voxelSecondary, voxelPosition);
};

template<typename TFunctor, typename TVoxelPrimary, typename TVoxelSecondary>
__global__ void
dualVoxelTraversal_device(TVoxelPrimary* primaryVoxels, TVoxelSecondary* secondaryVoxels,
                          const ITMLib::ITMPlainVoxelArray::ITMVoxelArrayInfo* arrayInfo,
                          TFunctor& functor) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;

	TVoxelPrimary& voxelPrimary = primaryVoxels[locId];
	TVoxelSecondary& voxelSecondary = secondaryVoxels[locId];
	functor(voxelPrimary, voxelSecondary);
};

}// namespace

namespace ITMLib {


// static-member-only classes are used here instead of namespaces to utilize template specialization (and maximize code reuse)
template<typename TVoxel>
class ITMSceneTraversalEngine<TVoxel, ITMPlainVoxelArray, ITMLibSettings::DEVICE_CUDA> {
public:
// region ================================ STATIC SINGLE-SCENE TRAVERSAL ===============================================
	template<typename TStaticFunctor>
	inline static void StaticVoxelTraversal(ITMScene<TVoxel, ITMPlainVoxelArray>* scene) {
		TVoxel* voxelArray = scene->localVBA.GetVoxelBlocks();
		const ITMPlainVoxelArray::ITMVoxelArrayInfo* arrayInfo = scene->index.getIndexData();

		dim3 cudaBlockSize(8, 8, 8);
		dim3 gridSize(scene->index.getVolumeSize().x / cudaBlockSize.x,
		              scene->index.getVolumeSize().y / cudaBlockSize.y,
		              scene->index.getVolumeSize().z / cudaBlockSize.z);

		staticVoxelTraversal_device<TStaticFunctor, TVoxel> << < gridSize, cudaBlockSize >> > (voxelArray, arrayInfo);
		ORcudaKernelCheck;
	}
// endregion ===========================================================================================================
};

template<typename TVoxelPrimary, typename TVoxelSecondary>
class ITMDualSceneTraversalEngine<TVoxelPrimary, TVoxelSecondary, ITMPlainVoxelArray, ITMLibSettings::DEVICE_CUDA> {
public:

// region ============================== DUAL-SCENE DYNAMIC TRAVERSAL ==================================================

	template<typename TFunctor>
	inline static void
	DualVoxelPositionTraversal(
			ITMScene<TVoxelPrimary, ITMPlainVoxelArray>* primaryScene,
			ITMScene<TVoxelSecondary, ITMPlainVoxelArray>* secondaryScene,
			TFunctor& functor) {

		assert(primaryScene->index.getVolumeSize() == secondaryScene->index.getVolumeSize());

		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		const ITMPlainVoxelArray::ITMVoxelArrayInfo* arrayInfo = primaryScene->index.getIndexData();

		dim3 cudaBlockSize(8, 8, 8);
		dim3 gridSize(primaryScene->index.getVolumeSize().x / cudaBlockSize.x,
		              primaryScene->index.getVolumeSize().y / cudaBlockSize.y,
		              primaryScene->index.getVolumeSize().z / cudaBlockSize.z);

		dualVoxelPositionTraversal_device<TFunctor, TVoxelPrimary, TVoxelSecondary>
				<< < gridSize, cudaBlockSize >> >
		                       (primaryVoxels, secondaryVoxels, arrayInfo, functor);
		ORcudaKernelCheck;

	}

	template<typename TFunctor>
	inline static void
	DualVoxelTraversal(
			ITMScene<TVoxelPrimary, ITMPlainVoxelArray>* primaryScene,
			ITMScene<TVoxelSecondary, ITMPlainVoxelArray>* secondaryScene,
			TFunctor& functor) {

		assert(primaryScene->index.getVolumeSize() == secondaryScene->index.getVolumeSize());

		TVoxelPrimary* primaryVoxels = primaryScene->localVBA.GetVoxelBlocks();
		TVoxelSecondary* secondaryVoxels = secondaryScene->localVBA.GetVoxelBlocks();
		const ITMPlainVoxelArray::ITMVoxelArrayInfo* arrayInfo = primaryScene->index.getIndexData();

		dim3 cudaBlockSize(8, 8, 8);
		dim3 gridSize(primaryScene->index.getVolumeSize().x / cudaBlockSize.x,
		              primaryScene->index.getVolumeSize().y / cudaBlockSize.y,
		              primaryScene->index.getVolumeSize().z / cudaBlockSize.z);

		dualVoxelTraversal_device<TFunctor, TVoxelPrimary, TVoxelSecondary>
				<< < gridSize, cudaBlockSize >> >
		                       (primaryVoxels, secondaryVoxels, arrayInfo, functor);
		ORcudaKernelCheck;

	}

// endregion ===========================================================================================================
};


}// namespace ITMLib