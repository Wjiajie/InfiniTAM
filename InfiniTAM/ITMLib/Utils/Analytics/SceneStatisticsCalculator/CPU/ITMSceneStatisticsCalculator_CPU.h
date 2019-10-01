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
#include "../Interface/ITMSceneStatisticsCalculator.h"
#include "../../../../ITMLibDefines.h"

namespace ITMLib {
template<typename TVoxel, typename TIndex>
class ITMSceneStatisticsCalculator_CPU :
		public ITMSceneStatisticsCalculator<TVoxel, TIndex> {
public:
	static ITMSceneStatisticsCalculator_CPU& Instance() {
		static ITMSceneStatisticsCalculator_CPU instance;
		return instance;
	}

	ITMSceneStatisticsCalculator_CPU(ITMSceneStatisticsCalculator_CPU const&) = delete;
	void operator=(ITMSceneStatisticsCalculator_CPU const&) = delete;

	Vector6i ComputeVoxelBounds(const ITMVoxelVolume<TVoxel, TIndex>* scene) override;
	int ComputeAllocatedVoxelCount(ITMVoxelVolume<TVoxel, TIndex>* scene) override;
	std::vector<int> GetFilledHashBlockIds(ITMVoxelVolume<TVoxel, TIndex>* scene) override;
	int ComputeAllocatedHashBlockCount(ITMVoxelVolume<TVoxel, TIndex>* scene) override;

	int ComputeNonTruncatedVoxelCount(ITMVoxelVolume<TVoxel, TIndex>* scene) override;
	int ComputeVoxelWithNonZeroSdfCount(ITMVoxelVolume<TVoxel, TIndex>* scene, float value) override;
	double ComputeNonTruncatedVoxelAbsSdfSum(ITMVoxelVolume<TVoxel, TIndex>* scene) override;
	double ComputeTruncatedVoxelAbsSdfSum(ITMVoxelVolume<TVoxel, TIndex>* scene) override;

	float FindMaxGradient0LengthAndPosition(ITMVoxelVolume<TVoxel, TIndex>* scene, Vector3i& positionOut) override;
	float FindMaxGradient1LengthAndPosition(ITMVoxelVolume<TVoxel, TIndex>* scene, Vector3i& positionOut) override;
private:
	ITMSceneStatisticsCalculator_CPU() = default;
	~ITMSceneStatisticsCalculator_CPU() = default;
};

typedef ITMSceneStatisticsCalculator_CPU<ITMVoxel, ITMVoxelBlockHash> SceneStatCalc_CPU_VBH_Voxel;
typedef ITMSceneStatisticsCalculator_CPU<ITMVoxel, ITMPlainVoxelArray> SceneStatCalc_CPU_PVA_Voxel;
typedef ITMSceneStatisticsCalculator_CPU<ITMWarp, ITMVoxelBlockHash> SceneStatCalc_CPU_VBH_Warp;
typedef ITMSceneStatisticsCalculator_CPU<ITMWarp, ITMPlainVoxelArray> SceneStatCalc_CPU_PVA_Warp;

}//end namespace ITMLib

