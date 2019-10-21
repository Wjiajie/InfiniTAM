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

using namespace ITMLib;

template<MemoryDeviceType TMemoryType>
struct WarpGradientDataFixture {
	WarpGradientDataFixture() :
			settings(&ITMLibSettings::Instance()),
			offsetSlice(-64, -24, 168),
			sizeSlice(80, 96, 144),
			warp_field_data_term(nullptr), canonical_scene(nullptr), live_scene(nullptr) {
		settings->enableKillingTerm = false;
		settings->enableDataTerm = true;
		settings->enableSmoothingTerm = false;
		settings->enableGradientSmoothing = false;
		settings->enableLevelSetTerm = false;

		BOOST_TEST_MESSAGE("setup fixture");
		warp_field_data_term = new ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>(&settings->sceneParams,
		                                                                       settings->swappingMode ==
		                                                                       ITMLibSettings::SWAPPINGMODE_ENABLED,
		                                                                       TMemoryType,
		                                                                       sizeSlice, offsetSlice);
		warp_field_data_term->LoadFromDirectory("TestData/snoopy_result_fr16-17_partial_PVA/gradient0_data_");

		warp_field_iter0 = new ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>(&settings->sceneParams,
		                                                                   settings->swappingMode ==
		                                                                   ITMLibSettings::SWAPPINGMODE_ENABLED,
		                                                                   TMemoryType,
		                                                                   sizeSlice, offsetSlice);
		warp_field_iter0->LoadFromDirectory("TestData/snoopy_result_fr16-17_partial_PVA/warp_iter0");

		warp_field_tikhonov_term = new ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>(&settings->sceneParams,
		                                                                           settings->swappingMode ==
		                                                                           ITMLibSettings::SWAPPINGMODE_ENABLED,
		                                                                           TMemoryType,
		                                                                           sizeSlice, offsetSlice);
		warp_field_tikhonov_term->LoadFromDirectory("TestData/snoopy_result_fr16-17_partial_PVA/gradient0_tikhonov_");

		warp_field_killing_term = new ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>(&settings->sceneParams,
		                                                                          settings->swappingMode ==
		                                                                          ITMLibSettings::SWAPPINGMODE_ENABLED,
		                                                                          settings->GetMemoryType(),
		                                                                          sizeSlice, offsetSlice);
		warp_field_killing_term->LoadFromDirectory("TestData/snoopy_result_fr16-17_partial_PVA/gradient0_killing_");

		warp_field_level_set_term = new ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>(&settings->sceneParams,
		                                                                            settings->swappingMode ==
		                                                                            ITMLibSettings::SWAPPINGMODE_ENABLED,
		                                                                            TMemoryType,
		                                                                            sizeSlice, offsetSlice);
		warp_field_level_set_term->LoadFromDirectory("TestData/snoopy_result_fr16-17_partial_PVA/gradient0_level_set_");


		canonical_scene = new ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>(&settings->sceneParams,
		                                                                       settings->swappingMode ==
		                                                                       ITMLibSettings::SWAPPINGMODE_ENABLED,
		                                                                   TMemoryType,
		                                                                   sizeSlice, offsetSlice);

		canonical_scene->LoadFromDirectory("TestData/snoopy_result_fr16-17_partial_PVA/canonical");
		live_scene = new ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>(&settings->sceneParams,
		                                                                  settings->swappingMode ==
		                                                                  ITMLibSettings::SWAPPINGMODE_ENABLED,
		                                                              TMemoryType,
		                                                              sizeSlice, offsetSlice);
		live_scene->LoadFromDirectory("TestData/snoopy_result_fr16-17_partial_PVA/live");


	}

	~WarpGradientDataFixture() {
		BOOST_TEST_MESSAGE("teardown fixture");
		delete warp_field_data_term;
		delete warp_field_iter0;
		delete warp_field_tikhonov_term;
		delete warp_field_killing_term;
		delete canonical_scene;
		delete live_scene;
	}

	ITMLibSettings* settings;
	Vector3i offsetSlice;
	Vector3i sizeSlice;
	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* warp_field_data_term;
	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* warp_field_iter0;
	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* warp_field_tikhonov_term;
	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* warp_field_killing_term;
	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* warp_field_level_set_term;
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* canonical_scene;
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* live_scene;
};
