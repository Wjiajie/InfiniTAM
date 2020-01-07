//  ================================================================
//  Created by Gregory Kramida on 1/5/18.
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

#include <vector>
#include "../../../ITMMath.h"
#include "../../../../Objects/Scene/ITMVoxelVolume.h"
#include "../Interface/ITMSceneStatisticsCalculatorInterface.h"
#include "../../../../ITMLibDefines.h"

namespace ITMLib {
template<typename TVoxel, typename TIndex>
class ITMSceneStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU> :
		public ITMSceneStatisticsCalculatorInterface<TVoxel, TIndex> {
public:
	static ITMSceneStatisticsCalculator& Instance() {
		static ITMSceneStatisticsCalculator instance;
		return instance;
	}

	ITMSceneStatisticsCalculator(ITMSceneStatisticsCalculator const&) = delete;
	void operator=(ITMSceneStatisticsCalculator const&) = delete;

	Vector6i ComputeVoxelBounds(const ITMVoxelVolume<TVoxel, TIndex>* scene) override;
	int ComputeAllocatedVoxelCount(ITMVoxelVolume<TVoxel, TIndex>* scene) override;
	std::vector<int> GetFilledHashBlockIds(ITMVoxelVolume<TVoxel, TIndex>* scene) override;
	int ComputeAllocatedHashBlockCount(ITMVoxelVolume<TVoxel, TIndex>* scene) override;

	int ComputeNonTruncatedVoxelCount(ITMVoxelVolume<TVoxel, TIndex>* scene) override;
	unsigned int ComputeAlteredVoxelCount(ITMVoxelVolume<TVoxel, TIndex>* scene) override;
	unsigned int CountVoxelsWithSpecificSdfValue(ITMVoxelVolume<TVoxel, TIndex>* scene, float value) override;
	double ComputeNonTruncatedVoxelAbsSdfSum(ITMVoxelVolume<TVoxel, TIndex>* scene) override;
	double ComputeTruncatedVoxelAbsSdfSum(ITMVoxelVolume<TVoxel, TIndex>* scene) override;
	double ComputeFramewiseWarpMin(ITMVoxelVolume<TVoxel,TIndex>* scene) override;
	double ComputeFramewiseWarpMax(ITMVoxelVolume<TVoxel,TIndex>* scene) override;
	double ComputeFramewiseWarpMean(ITMVoxelVolume<TVoxel,TIndex>* scene) override;

	Vector6i FindMinimumNonTruncatedBoundingBox(ITMVoxelVolume <TVoxel, TIndex>* scene) override;

	float FindMaxGradient0LengthAndPosition(ITMVoxelVolume<TVoxel, TIndex>* scene, Vector3i& positionOut) override;
	float FindMaxGradient1LengthAndPosition(ITMVoxelVolume<TVoxel, TIndex>* scene, Vector3i& positionOut) override;
private:
	ITMSceneStatisticsCalculator() = default;
	~ITMSceneStatisticsCalculator() = default;
};

typedef ITMSceneStatisticsCalculator<ITMVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU> SceneStatCalc_CPU_VBH_Voxel;
typedef ITMSceneStatisticsCalculator<ITMVoxel, ITMPlainVoxelArray, MEMORYDEVICE_CPU> SceneStatCalc_CPU_PVA_Voxel;
typedef ITMSceneStatisticsCalculator<ITMWarp, ITMVoxelBlockHash, MEMORYDEVICE_CPU> SceneStatCalc_CPU_VBH_Warp;
typedef ITMSceneStatisticsCalculator<ITMWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU> SceneStatCalc_CPU_PVA_Warp;

}//end namespace ITMLib

