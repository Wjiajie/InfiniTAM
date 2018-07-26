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

#include "ITMSceneMotionTracker_CUDA.h"
#include "../Shared/ITMSceneMotionTracker_Functors.h"
#include "../../Objects/Scene/ITMSceneTraversal_CUDA.h"

namespace {
//CUDA device functions
template<typename TVoxelMulti>
__global__ void
clearOutGradient_device(TVoxelMulti* voxelArray, const ITMPlainVoxelArray::ITMVoxelArrayInfo* arrayInfo,
                          int flagFieldIndex) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int z = blockIdx.z * blockDim.z + threadIdx.z;

	int locId = x + y * arrayInfo->size.x + z * arrayInfo->size.x * arrayInfo->size.y;
	TVoxelMulti& voxel = voxelArray[locId];

}
}

template<typename TVoxelCanonical, typename TVoxelLive>
ITMLib::ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::ITMSceneMotionTracker_CUDA() {

}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMLib::ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::ClearOutFramewiseWarp(
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene) {

	StaticVoxelTraversal_CPU<TVoxelCanonical, ClearOutFramewiseWarpStaticFunctor>(canonicalScene);

	TVoxelCanonical* canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
	const ITMPlainVoxelArray::ITMVoxelArrayInfo* arrayInfo = canonicalScene->index.getIndexData();

	dim3 cudaBlockSize(8, 8, 8);
	dim3 gridSize(canonicalScene->index.getVolumeSize().x / cudaBlockSize.x,
	              canonicalScene->index.getVolumeSize().y / cudaBlockSize.y,
	              canonicalScene->index.getVolumeSize().z / cudaBlockSize.z);

	ORcudaKernelCheck;
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMLib::ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::AddFramewiseWarpToWarp(
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene, bool clearFramewiseWarp) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMLib::ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::CalculateWarpGradient(
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene,
		ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene, int sourceFieldIndex, bool restrictZTrackingForDebugging) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMLib::ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::SmoothWarpGradient(
		ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene,
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene, int sourceFieldIndex) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<typename TVoxelCanonical, typename TVoxelLive>
float ITMLib::ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::UpdateWarps(
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene,
		ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene, int sourceSdfIndex) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMLib::ITMSceneMotionTracker_CUDA<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::ResetWarps(
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}
