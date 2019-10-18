//  ================================================================
//  Created by Gregory Kramida on 10/18/19.
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
#pragma once

#include <iostream>
#include "../../../ORUtils/PlatformIndependentAtomics.h"
#include "../../Utils/ITMMath.h"
#include "ITMSceneMotionOptimizationParameters.h"
#include "../../Utils/ITMPrintHelpers.h"
#include "../../../ORUtils/JetbrainsCUDASyntax.hpp"
#include "../../../ORUtils/CUDADefines.h"
#include <cuda_runtime.h>
#include "../../Utils/ITMCUDAUtils.h"



namespace ITMLib {
template<typename TVoxel, typename TWarp, typename TIndexData, typename TCache>
struct ITMSceneMotionDataTermCUDAFunctor {

	ITMSceneMotionDataTermCUDAFunctor(ITMSceneMotionOptimizationParameters parameters,
	                                  ITMSceneMotionOptimizationSwitches switches,
	                                  TVoxel* liveVoxels, const TIndexData* liveIndexData,
	                                  TVoxel* canonicalVoxels, const TIndexData* canonicalIndexData,
	                                  TWarp* warps, const TIndexData* warpIndexData) :
			weight(parameters.weightDataTerm), liveVoxels(liveVoxels), liveIndexData(liveIndexData), liveCache() {

		INITIALIZE_ATOMIC(float, energy, 0.0f);
	}

	ITMSceneMotionDataTermCUDAFunctor(TVoxel* liveVoxels, const TIndexData* liveIndexData,
	                                  float weight) :
			weight(weight), liveVoxels(liveVoxels), liveIndexData(liveIndexData), liveCache() {
		ORcudaSafeCall(cudaMalloc((void**)&energy, sizeof(float)));
		float val = 0.0f;
		ORcudaSafeCall(cudaMemcpy(energy,  &val, sizeof(float), cudaMemcpyHostToDevice));
	}

	~ITMSceneMotionDataTermCUDAFunctor() {
		ORcudaSafeCall (cudaFree(energy));
	}

	float weight;
	TVoxel* liveVoxels;
	const TIndexData* liveIndexData;
	TCache liveCache;
	float* energy;

	__device__
	void operator()(TVoxel& voxelLive, TVoxel& voxelCanonical, TWarp& warp, Vector3i voxelPosition) {
		if (!VoxelIsConsideredForTracking(voxelCanonical, voxelLive)
		    || !VoxelIsConsideredForDataTerm(voxelCanonical, voxelLive))
			return;

		float liveSdf = TVoxel::valueToFloat(voxelLive.sdf);
		float canonicalSdf = TVoxel::valueToFloat(voxelCanonical.sdf);
		Vector3f liveSdfJacobian;

		ComputeLiveJacobian_CentralDifferences(
				liveSdfJacobian, voxelPosition, liveVoxels, liveIndexData, liveCache);

		// Compute data term energy & gradient

		// (φ_n(Ψ)−φ_{global}) ∇φ_n(Ψ) - also denoted as - (φ_{proj}(Ψ)−φ_{model}) ∇φ_{proj}(Ψ)
		// φ_n(Ψ) = φ_n(x+u, y+v, z+w), where u = u(x,y,z), v = v(x,y,z), w = w(x,y,z)
		// φ_{global} = φ_{global}(x, y, z)
		float sdfDifferenceBetweenLiveAndCanonical = liveSdf - canonicalSdf;
		float localDataEnergy =
				weight * 0.5f * (sdfDifferenceBetweenLiveAndCanonical * sdfDifferenceBetweenLiveAndCanonical);
		ATOMIC_ADD(energy, localDataEnergy);
		Vector3f localDataEnergyGradient = weight * sdfDifferenceBetweenLiveAndCanonical * liveSdfJacobian;
		warp.gradient += localDataEnergyGradient;

	}


	float getEnergy() const {
		float var_val;
		ORcudaSafeCall(cudaMemcpy(&var_val, energy, sizeof(float), cudaMemcpyDeviceToHost));
		return var_val;
	}

	void PrintStatistics() {
		std::cout << bright_cyan << "*** Non-rigid Alignment Iteration Statistics ***" << reset << std::endl;
		PrintEnergyStatistics(true, false, false, false, 0.0f, getEnergy(), 0.0, 0.0, 0.0);
	}

};
}