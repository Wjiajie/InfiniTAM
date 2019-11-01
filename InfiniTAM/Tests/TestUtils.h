//  ================================================================
//  Created by Gregory Kramida on 10/24/17.
//  Copyright (c) 2017-2025 Gregory Kramida
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

#include <chrono>
#include "../ITMLib/ITMLibDefines.h"

#include "../ITMLib/Objects/Scene/ITMRepresentationAccess.h"
#include "../ITMLib/Objects/Scene/ITMVoxelVolume.h"
#include "../ITMLib/Utils/ITMLibSettings.h"


using namespace ITMLib;

template<class TVoxel, class TIndex>
void GenerateTestScene_CPU(ITMVoxelVolume<TVoxel, TIndex>* scene);

template<class TVoxel, class TIndex>
void GenerateTestScene_CUDA(ITMVoxelVolume<TVoxel, TIndex>* scene);

void GenerateAndLogKillingScene01();

template<typename TVoxel>
void simulateVoxelAlteration(TVoxel& voxel, float newSdfValue);

template<typename TVoxel>
void simulateRandomVoxelAlteration(TVoxel& voxel);

inline
void TimeIt(std::function<void(void)> function, const std::string& description = "Timed Operation") {
	std::cout << description << std::endl;
	auto start = std::chrono::high_resolution_clock::now();
	function();
	auto finish = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = finish - start;
	std::cout << "Elapsed time: " << elapsed.count() << " s\n";
}


template<typename TVoxel, typename TIndex>
void PrepareVoxelVolumeForLoading(ITMVoxelVolume<TVoxel, TIndex>* volume, MemoryDeviceType deviceType);


template<typename TIndex>
typename TIndex::InitializationParameters GetCommonIndexParameters();

//TODO: figure out whether it still makes sense to suppress assignment operator of the ITMVoxelVolume class...
// Then restore or delete this depending on the decision. If the decision is negative, provide a constructor that loads from path instead.
//
//template<typename TVoxelA, typename TIndex>
//ITMVoxelVolume<TVoxelA, TIndex> loadSdfVolume (const std::string& path, MemoryDeviceType memoryDeviceType,
//		typename TIndex::InitializationParameters initializationParameters = GetCommonIndexParameters<TIndex>(),
//		ITMLibSettings::SwappingMode swappingMode = ITMLibSettings::SWAPPINGMODE_DISABLED);

template<typename TVoxel, typename TIndex>
void loadSdfVolume(ITMVoxelVolume<TVoxel, TIndex>** volume, const std::string& path, MemoryDeviceType memoryDeviceType,
                   typename TIndex::InitializationParameters initializationParameters = GetCommonIndexParameters<TIndex>(),
                   ITMLibSettings::SwappingMode swappingMode = ITMLibSettings::SWAPPINGMODE_DISABLED);