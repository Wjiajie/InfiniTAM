//  ================================================================
//  Created by Gregory Kramida on 10/1/19.
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


#include <vector>
#include "../../../../Objects/Scene/ITMVoxelVolume.h"

namespace ITMLib {
template<typename TVoxel, typename TIndex>
class ITMSceneStatisticsCalculator {
public:
	virtual Vector6i ComputeVoxelBounds(const ITMVoxelVolume<TVoxel, TIndex>* scene) = 0;
	virtual int ComputeAllocatedVoxelCount(ITMVoxelVolume<TVoxel, TIndex>* scene) = 0;
	virtual std::vector<int> GetFilledHashBlockIds(ITMVoxelVolume<TVoxel, TIndex>* scene) = 0;
	virtual int ComputeAllocatedHashBlockCount(ITMVoxelVolume<TVoxel, TIndex>* scene) = 0;

	virtual int ComputeNonTruncatedVoxelCount(ITMVoxelVolume<TVoxel, TIndex>* scene) = 0;
	virtual unsigned int ComputeAlteredVoxelCount(ITMVoxelVolume<TVoxel, TIndex>* scene) = 0;
	virtual unsigned int CountVoxelsWithSpecificSdfValue(ITMVoxelVolume<TVoxel, TIndex>* scene, float value) = 0;
	virtual double ComputeNonTruncatedVoxelAbsSdfSum(ITMVoxelVolume<TVoxel, TIndex>* scene) = 0;
	virtual double ComputeTruncatedVoxelAbsSdfSum(ITMVoxelVolume<TVoxel, TIndex>* scene) = 0;
	virtual double ComputeFlowWarpMin(ITMVoxelVolume<TVoxel,TIndex>* scene) = 0;
	virtual double ComputeFlowWarpMax(ITMVoxelVolume<TVoxel,TIndex>* scene) = 0;
	virtual double ComputeFlowWarpMean(ITMVoxelVolume<TVoxel,TIndex>* scene) = 0;


	virtual float FindMaxGradient0LengthAndPosition(ITMVoxelVolume<TVoxel, TIndex>* scene, Vector3i& positionOut) = 0;
	virtual float FindMaxGradient1LengthAndPosition(ITMVoxelVolume<TVoxel, TIndex>* scene, Vector3i& positionOut) = 0;
};

}//end namespace ITMLib


