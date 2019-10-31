//  ================================================================
//  Created by Gregory Kramida on 10/30/19.
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

#ifdef __CUDACC__
#include "../../../../Utils/ITMCUDAUtils.h"
#include "../../../../Engines/Traversal/CUDA/ITMSceneTraversal_CUDA_PlainVoxelArray.h"
#include "../../../../Engines/Traversal/CUDA/ITMSceneTraversal_CUDA_VoxelBlockHash.h"
#endif

#include "../../../../../ORUtils/PlatformIndependentAtomics.h"
#include "../../../../../ORUtils/PlatformIndependence.h"
#include "../../../ITMLibSettings.h"
#include "../../../../Objects/Scene/ITMVoxelVolume.h"
#include "../../../../Engines/Traversal/CPU/ITMSceneTraversal_CPU_PlainVoxelArray.h"
#include "../../../../Engines/Traversal/CPU/ITMSceneTraversal_CPU_VoxelBlockHash.h"



using namespace ITMLib;

namespace ITMLib{
	enum Statistic{
		MINIMUM,
		MAXIMUM,
		MEAN
	};
}

template<bool hasCumulativeWarp, typename TVoxel, typename TIndex, ITMLibSettings::DeviceType TDeviceType, Statistic TStatistic>
struct ComputeFlowWarpLengthStatisticFunctor;

template<typename TVoxel, typename TIndex, ITMLibSettings::DeviceType TDeviceType, Statistic TStatistic>
struct ComputeFlowWarpLengthStatisticFunctor<false, TVoxel, TIndex, TDeviceType, TStatistic> {
	static int compute(ITMVoxelVolume<TVoxel, TIndex>* scene) {
		DIEWITHEXCEPTION("Voxels need to have flow warp information to get flow warp statistics.");
	}
};

template <typename TVoxel, Statistic TStatistic>
struct HandleAggregate;
template <typename TVoxel>
struct HandleAggregate<TVoxel, MINIMUM>{
	_DEVICE_WHEN_AVAILABLE_
	inline static void aggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, TVoxel& voxel){
		ATOMIC_MIN(value, static_cast<double>(ORUtils::length(voxel.flow_warp)));

	}
};
template <typename TVoxel>
struct HandleAggregate<TVoxel, MAXIMUM>{
	_DEVICE_WHEN_AVAILABLE_
	inline static void aggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, TVoxel& voxel){
		ATOMIC_MAX(value, static_cast<double>(ORUtils::length(voxel.flow_warp)));
	}
};
template <typename TVoxel>
struct HandleAggregate<TVoxel, MEAN>{
	_DEVICE_WHEN_AVAILABLE_
	inline static void aggregateStatistic(ATOMIC_ARGUMENT(double) value, ATOMIC_ARGUMENT(unsigned int) count, TVoxel& voxel){
		ATOMIC_ADD(value, static_cast<double>(ORUtils::length(voxel.flow_warp)));
		ATOMIC_ADD(count, 1u);
	}
};


template<typename TVoxel, typename TIndex, ITMLibSettings::DeviceType TDeviceType, Statistic TStatistic>
struct ComputeFlowWarpLengthStatisticFunctor<true, TVoxel, TIndex, TDeviceType, TStatistic> {

	static double compute(ITMVoxelVolume<TVoxel, TIndex>* scene) {
		ComputeFlowWarpLengthStatisticFunctor instance;
		INITIALIZE_ATOMIC(double, instance.aggregate, 0.0);
		INITIALIZE_ATOMIC(unsigned int, instance.count, 0u);
		ITMSceneTraversalEngine<TVoxel, TIndex, TDeviceType>::VoxelTraversal(scene, instance);
		double aggregate = GET_ATOMIC_VALUE_CPU(instance.aggregate);
		unsigned int count = GET_ATOMIC_VALUE_CPU(instance.count);
		if(TStatistic == MEAN){
			return aggregate / count;
		}else{
			return aggregate;
		}
	}

	DECLARE_ATOMIC(double, aggregate);
	DECLARE_ATOMIC(unsigned int, count);

	_DEVICE_WHEN_AVAILABLE_
	void operator()(TVoxel& voxel) {
		HandleAggregate<TVoxel,TStatistic>::aggregateStatistic(aggregate, count, voxel);
	}
};
