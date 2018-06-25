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
#include "../../Objects/Scene/ITMScene.h"

namespace ITMLib {
template<typename TVoxel, typename TIndex>
class ITMSceneStatisticsCalculator {
public:
	void ComputeVoxelBounds(const ITMScene<TVoxel, TIndex>* scene, Vector3i& minVoxelPoint, Vector3i& maxVoxelPoint);
	void ComputeVoxelBounds(const ITMScene<TVoxel, TIndex>* scene, Vector6i& bounds);
	int ComputeAllocatedVoxelCount(ITMScene<TVoxel, TIndex>* scene);
	std::vector<int> GetFilledHashBlockIds(ITMScene<TVoxel, TIndex>* scene);
	int ComputeAllocatedHashBlockCount(ITMScene<TVoxel, TIndex>* scene);


	int ComputeNonTruncatedVoxelCount(ITMScene<TVoxel, TIndex>* scene);
	int ComputeVoxelWithValueCount(ITMScene<TVoxel, TIndex>* scene, float value);
	double ComputeNonTruncatedVoxelAbsSdfSum(ITMScene<TVoxel, TIndex>* scene);
	double ComputeTruncatedVoxelAbsSdfSum(ITMScene<TVoxel, TIndex>* scene);

	float FindMaxGradient1LengthAndPosition(ITMScene<TVoxel, TIndex>* scene, Vector3i& positionOut);
	float FindMaxGradient0LengthAndPosition(ITMScene<TVoxel, TIndex>* scene, Vector3i& positionOut);

};

}//end namespace ITMLib

