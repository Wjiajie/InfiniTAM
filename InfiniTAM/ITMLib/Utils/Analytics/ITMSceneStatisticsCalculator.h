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

#include "../ITMMath.h"
#include "../../Objects/Scene/ITMVoxelVolume.h"

namespace ITMLib {
template<typename TVoxel, typename TIndex>
class ITMSceneStatisticsCalculator {
public:
	static ITMSceneStatisticsCalculator & Instance(){
		static ITMSceneStatisticsCalculator instance;
		return instance;
	}

	ITMSceneStatisticsCalculator(ITMSceneStatisticsCalculator const&) = delete;
	void operator=(ITMSceneStatisticsCalculator const&) = delete;

	Vector6i ComputeVoxelBounds(const ITMVoxelVolume<TVoxel, TIndex>* scene);
	int ComputeAllocatedVoxelCount(ITMVoxelVolume<TVoxel, TIndex>* scene);
	std::vector<int> GetFilledHashBlockIds(ITMVoxelVolume<TVoxel, TIndex>* scene);
	int ComputeAllocatedHashBlockCount(ITMVoxelVolume<TVoxel, TIndex>* scene);

	int ComputeNonTruncatedVoxelCount(ITMVoxelVolume<TVoxel, TIndex>* scene);
	int ComputeVoxelWithValueCount(ITMVoxelVolume<TVoxel, TIndex>* scene, float value);
	double ComputeNonTruncatedVoxelAbsSdfSum(ITMVoxelVolume<TVoxel, TIndex>* scene);
	double ComputeTruncatedVoxelAbsSdfSum(ITMVoxelVolume<TVoxel, TIndex>* scene);

	float FindMaxGradient1LengthAndPosition(ITMVoxelVolume<TVoxel, TIndex>* scene, Vector3i& positionOut);
	float FindMaxGradient0LengthAndPosition(ITMVoxelVolume<TVoxel, TIndex>* scene, Vector3i& positionOut);
private:
	ITMSceneStatisticsCalculator() = default;
	~ITMSceneStatisticsCalculator() = default;
};

}//end namespace ITMLib

