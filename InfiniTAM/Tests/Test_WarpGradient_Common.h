//  ================================================================
//  Created by Gregory Kramida on 10/21/19.
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


#include <boost/test/test_tools.hpp>
#include "../ORUtils/MemoryDeviceType.h"
#include "../ITMLib/ITMLibDefines.h"
#include "../ITMLib/Utils/ITMLibSettings.h"
#include "../ITMLib/Objects/Scene/ITMVoxelVolume.h"
#include "../ITMLib/Engines/Manipulation/Interface/ITMSceneManipulationEngine.h"
#include "../ITMLib/Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../ITMLib/Engines/Manipulation/CUDA/ITMSceneManipulationEngine_CUDA.h"

using namespace ITMLib;

template<typename TIndex>
std::string getIndexSuffix();

template<>
std::string getIndexSuffix<ITMVoxelBlockHash>() {
	return "VBH";
}

template<>
std::string getIndexSuffix<ITMPlainVoxelArray>() {
	return "PVA";
}

template<typename TVoxel, typename TIndex>
void PrepareVoxelVolume(ITMVoxelVolume<TVoxel, TIndex>* volume, MemoryDeviceType deviceType);

//have nothing to prep for PVA -- everything gets copied off the disk exactly
template<>
void PrepareVoxelVolume(ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* volume, MemoryDeviceType deviceType) {}

template<>
void PrepareVoxelVolume(ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* volume, MemoryDeviceType deviceType) {}

//for VBH, the scene has to be reset -- there may be
template<>
void PrepareVoxelVolume(ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* volume, MemoryDeviceType deviceType) {
	switch (deviceType) {
		case MEMORYDEVICE_CPU:
			ITMSceneManipulationEngine_CPU<ITMWarp, ITMVoxelBlockHash>::Inst().ResetScene(volume);
			break;
		case MEMORYDEVICE_CUDA:
			ITMSceneManipulationEngine_CUDA<ITMWarp, ITMVoxelBlockHash>::Inst().ResetScene(volume);
			break;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Memory/Device type not supported");
	}
}

template<>
void PrepareVoxelVolume(ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* volume, MemoryDeviceType deviceType) {
	switch (deviceType) {
		case MEMORYDEVICE_CPU:
			ITMSceneManipulationEngine_CPU<ITMVoxel, ITMVoxelBlockHash>::Inst().ResetScene(volume);
			break;
		case MEMORYDEVICE_CUDA:
			ITMSceneManipulationEngine_CUDA<ITMVoxel, ITMVoxelBlockHash>::Inst().ResetScene(volume);
			break;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Memory/Device type not supported");
	}
}

template <typename TIndex>
typename TIndex::InitializationParameters GetIndexParameters();

template <>
ITMPlainVoxelArray::InitializationParameters GetIndexParameters<ITMPlainVoxelArray>(){
	return {Vector3i(80, 96, 144), Vector3i(-64, -24, 168)};
}

template <>
ITMVoxelBlockHash::InitializationParameters GetIndexParameters<ITMVoxelBlockHash>(){
	return { 0x800, 0x20000};
}

template<MemoryDeviceType TMemoryType, typename TIndex>
struct WarpGradientDataFixture {
	WarpGradientDataFixture() :
			settings(&ITMLibSettings::Instance()),
			warp_field_data_term(nullptr), canonical_volume(nullptr), live_volume(nullptr),
			pathToData("TestData/snoopy_result_fr16-17_partial_" + getIndexSuffix<TIndex>() + "/"),
			indexParameters(GetIndexParameters<TIndex>()){

		settings->enableKillingConstraintInSmoothingTerm = false;
		settings->enableDataTerm = true;
		settings->enableSmoothingTerm = false;
		settings->enableGradientSmoothing = false;
		settings->enableLevelSetTerm = false;

		BOOST_TEST_MESSAGE("setup fixture");
		auto loadSdfVolume = [&](ITMVoxelVolume<ITMVoxel, TIndex>** scene, const std::string& pathSuffix){
			*scene = new ITMVoxelVolume<ITMVoxel, TIndex>(&settings->sceneParams,
			                                                   settings->swappingMode ==
			                                                   ITMLibSettings::SWAPPINGMODE_ENABLED,
			                                                   TMemoryType,
			                                                   indexParameters);
			PrepareVoxelVolume(*scene, TMemoryType);
			(*scene)->LoadFromDirectory(pathToData + pathSuffix);
		};
		auto loadWarpVolume = [&](ITMVoxelVolume<ITMWarp, TIndex>** scene, const std::string& pathSuffix){
			*scene = new ITMVoxelVolume<ITMWarp, TIndex>(&settings->sceneParams,
			                                             settings->swappingMode ==
			                                             ITMLibSettings::SWAPPINGMODE_ENABLED,
			                                             TMemoryType,
			                                             indexParameters);
			PrepareVoxelVolume(*scene, TMemoryType);
			(*scene)->LoadFromDirectory(pathToData + pathSuffix);
		};
		loadSdfVolume(&live_volume, "live");
		loadSdfVolume(&canonical_volume, "canonical");
		loadWarpVolume(&warp_field_data_term, "warp_field_0_data_");
		loadWarpVolume(&warp_field_iter0, "warp_field_0_data_flow_warps_");
		loadWarpVolume(&warp_field_tikhonov_term, "warp_field_1_tikhonov_");
		loadWarpVolume(&warp_field_data_and_tikhonov_term, "warp_field_1_data_and_tikhonov_");
		loadWarpVolume(&warp_field_data_and_killing_term, "warp_field_1_data_and_killing_");
		loadWarpVolume(&warp_field_data_and_level_set_term, "warp_field_1_data_and_level_set_");
	}

	~WarpGradientDataFixture() {
		BOOST_TEST_MESSAGE("teardown fixture");
		delete live_volume;
		delete canonical_volume;
		delete warp_field_data_term;
		delete warp_field_iter0;
		delete warp_field_tikhonov_term;
		delete warp_field_data_and_tikhonov_term;
		delete warp_field_data_and_killing_term;
		delete warp_field_data_and_level_set_term;
	}

	ITMLibSettings* settings;
	ITMVoxelVolume<ITMWarp, TIndex>* warp_field_data_term;
	ITMVoxelVolume<ITMWarp, TIndex>* warp_field_iter0;
	ITMVoxelVolume<ITMWarp, TIndex>* warp_field_tikhonov_term;
	ITMVoxelVolume<ITMWarp, TIndex>* warp_field_data_and_tikhonov_term;
	ITMVoxelVolume<ITMWarp, TIndex>* warp_field_data_and_killing_term;
	ITMVoxelVolume<ITMWarp, TIndex>* warp_field_data_and_level_set_term;
	ITMVoxelVolume<ITMVoxel, TIndex>* canonical_volume;
	ITMVoxelVolume<ITMVoxel, TIndex>* live_volume;
	const std::string pathToData;
	const typename TIndex::InitializationParameters indexParameters;
};
