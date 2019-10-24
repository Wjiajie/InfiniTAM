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
void prepareVoxelVolume(ITMVoxelVolume<TVoxel, TIndex>* volume, MemoryDeviceType deviceType);

//have nothing to prep for PVA -- everything gets copied off the disk exactly
template<>
void prepareVoxelVolume(ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* volume, MemoryDeviceType deviceType) {}

template<>
void prepareVoxelVolume(ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* volume, MemoryDeviceType deviceType) {}

//for VBH, the scene has to be reset -- there may be
template<>
void prepareVoxelVolume(ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* volume, MemoryDeviceType deviceType) {
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
void prepareVoxelVolume(ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* volume, MemoryDeviceType deviceType) {
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


template<MemoryDeviceType TMemoryType, typename TIndex>
struct WarpGradientDataFixture {
	WarpGradientDataFixture() :
			settings(&ITMLibSettings::Instance()),
			offsetSlice(-64, -24, 168),
			sizeSlice(80, 96, 144),
			warp_field_data_term(nullptr), canonical_volume(nullptr), live_volume(nullptr),
			pathToData("TestData/snoopy_result_fr16-17_partial_" + getIndexSuffix<TIndex>() + "/") {
		settings->enableKillingTerm = false;
		settings->enableDataTerm = true;
		settings->enableSmoothingTerm = false;
		settings->enableGradientSmoothing = false;
		settings->enableLevelSetTerm = false;

		BOOST_TEST_MESSAGE("setup fixture");
		loadCanonical();
		loadLive();
		loadWarpFieldIter0();
	}

	~WarpGradientDataFixture() {
		BOOST_TEST_MESSAGE("teardown fixture");
		clearWarpFieldIter0();
		clearCanonical();
		clearLive();
	}
	void loadLive() {
		if (!liveLoaded) {
			live_volume = new ITMVoxelVolume<ITMVoxel, TIndex>(&settings->sceneParams,
			                                                        settings->swappingMode ==
			                                                        ITMLibSettings::SWAPPINGMODE_ENABLED,
			                                                        TMemoryType,
			                                                        sizeSlice, offsetSlice);
			prepareVoxelVolume(live_volume, TMemoryType);
			live_volume->LoadFromDirectory(pathToData + "live");
			liveLoaded = true;
		}
	}

	void clearLive() {
		if (liveLoaded) {
			delete live_volume;
		}
	}

	void loadCanonical() {
		if (!canonicalLoaded) {
			canonical_volume = new ITMVoxelVolume<ITMVoxel, TIndex>(&settings->sceneParams,
			                                                        settings->swappingMode ==
			                                                        ITMLibSettings::SWAPPINGMODE_ENABLED,
			                                                        TMemoryType,
			                                                        sizeSlice, offsetSlice);
			prepareVoxelVolume(canonical_volume, TMemoryType);
			canonical_volume->LoadFromDirectory(pathToData + "canonical");
			canonicalLoaded = true;
		}
	}

	void clearCanonical() {
		if (canonicalLoaded) {
			delete canonical_volume;
		}
	}

	void loadWarpFieldDataTerm() {
		warp_field_data_term = new ITMVoxelVolume<ITMWarp, TIndex>(&settings->sceneParams,
		                                                           settings->swappingMode ==
		                                                           ITMLibSettings::SWAPPINGMODE_ENABLED,
		                                                           TMemoryType,
		                                                           sizeSlice, offsetSlice);
		prepareVoxelVolume(warp_field_data_term, TMemoryType);
		warp_field_data_term->LoadFromDirectory(pathToData + "gradient0_data_");
	}

	void clearWarpFieldDataTerm() {
		delete warp_field_data_term;
	}

	void loadWarpFieldIter0() {
		if (!warpFieldIter0Loaded) {
			warp_field_iter0 = new ITMVoxelVolume<ITMWarp, TIndex>(&settings->sceneParams,
			                                                       settings->swappingMode ==
			                                                       ITMLibSettings::SWAPPINGMODE_ENABLED,
			                                                       TMemoryType,
			                                                       sizeSlice, offsetSlice);
			prepareVoxelVolume(warp_field_iter0, TMemoryType);
			warp_field_iter0->LoadFromDirectory(pathToData + "warp_iter0");
			warpFieldIter0Loaded = true;
		}
	}

	void clearWarpFieldIter0() {
		if (warpFieldIter0Loaded) {
			delete warp_field_iter0;
			warpFieldIter0Loaded = false;
		}
	}

	void loadWarpFieldTikhonovTerm() {
		warp_field_tikhonov_term = new ITMVoxelVolume<ITMWarp, TIndex>(&settings->sceneParams,
		                                                               settings->swappingMode ==
		                                                               ITMLibSettings::SWAPPINGMODE_ENABLED,
		                                                               TMemoryType,
		                                                               sizeSlice, offsetSlice);
		prepareVoxelVolume(warp_field_tikhonov_term, TMemoryType);
		warp_field_tikhonov_term->LoadFromDirectory(pathToData + "gradient0_tikhonov_");
	}

	void clearWarpFieldTikhonovTerm() {
		delete warp_field_tikhonov_term;
	}

	void loadWarpFieldKillingTerm() {
		warp_field_killing_term = new ITMVoxelVolume<ITMWarp, TIndex>(&settings->sceneParams,
		                                                              settings->swappingMode ==
		                                                              ITMLibSettings::SWAPPINGMODE_ENABLED,
		                                                              TMemoryType,
		                                                              sizeSlice, offsetSlice);
		prepareVoxelVolume(warp_field_killing_term, TMemoryType);
		warp_field_killing_term->LoadFromDirectory(pathToData + "gradient0_killing_");
	}

	void clearWarpFieldKillingTerm() {
		delete warp_field_killing_term;
	}

	void loadWarpFieldLevelSetTerm() {
		warp_field_level_set_term = new ITMVoxelVolume<ITMWarp, TIndex>(&settings->sceneParams,
		                                                                settings->swappingMode ==
		                                                                ITMLibSettings::SWAPPINGMODE_ENABLED,
		                                                                TMemoryType,
		                                                                sizeSlice, offsetSlice);
		prepareVoxelVolume(warp_field_level_set_term, TMemoryType);
		warp_field_level_set_term->LoadFromDirectory(pathToData + "gradient0_level_set_");
	}

	void clearWarpFieldLevelSetTerm() {
		delete warp_field_level_set_term;
	}

	ITMLibSettings* settings;
	Vector3i offsetSlice;
	Vector3i sizeSlice;
	ITMVoxelVolume<ITMWarp, TIndex>* warp_field_data_term;

	ITMVoxelVolume<ITMWarp, TIndex>* warp_field_iter0;
	ITMVoxelVolume<ITMWarp, TIndex>* warp_field_tikhonov_term;
	ITMVoxelVolume<ITMWarp, TIndex>* warp_field_killing_term;
	ITMVoxelVolume<ITMWarp, TIndex>* warp_field_level_set_term;
	ITMVoxelVolume<ITMVoxel, TIndex>* canonical_volume;
	ITMVoxelVolume<ITMVoxel, TIndex>* live_volume;
	std::string pathToData;
private:
	bool warpFieldIter0Loaded = false;
	bool canonicalLoaded = false;
	bool liveLoaded = false;
};
