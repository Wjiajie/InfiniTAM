//  ================================================================
//  Created by Gregory Kramida on 11/13/19.
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

#define BOOST_TEST_MODULE Configuration
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//local
#include "../ITMLib/Utils/Configuration.h"

using namespace ITMLib;

BOOST_AUTO_TEST_CASE(ConfigurationTest) {
	Configuration default_configuration;
#ifdef COMPILE_WITHOUT_CUDA
	//default_configuration.save_to_json_file("../../Tests/TestData/default_config_cpu.json");
	Configuration::load_configuration_from_json_file("TestData/default_config_cpu.json");
#else
	//default_configuration.save_to_json_file("../../Tests/TestData/default_config_cuda.json");
	Configuration::load_configuration_from_json_file("TestData/default_config_cuda.json");
#endif
	Configuration& current_config = Configuration::get();
	BOOST_REQUIRE_EQUAL(default_configuration, current_config);
	Configuration configuration1(
			VoxelVolumeParameters(0.005, 0.12, 4.12, 0.05, 200, true, true, 1.2f),
			SurfelVolumeParameters(0.4f, 0.5f, static_cast<float>(22 * M_PI / 180), 0.008f, 0.0003f, 3.4f, 26.0f, 5,
			                       1.1f, 4.5f, 21, 300, false, false),
			SlavchevaSurfaceTracker::Parameters(0.11f, 0.09f, 2.0f, 0.3f, 0.1f, 1e-6f),
			SlavchevaSurfaceTracker::Switches(false, true, false, true, false),
			Configuration::TelemetrySettings(true, Vector3i(20, 23, 0)),
			Configuration::InputAndOutputSettings(true, true),
			Configuration::Paths("TestData/output1",
			                     "TestData/calib_file1.txt",
			                     "", "", "",
			                     "TestData/frame_color_%%06i.png",
			                     "TestData/frame_depth_%%06i.png",
			                     "TestData/frame_mask_%%06i.png",
			                     ""),
			Configuration::UIEngineSettings(50, 16),
			NonRigidTrackingParameters(300, 0.0002f, 0.4f),
			true,
			false,
			MEMORYDEVICE_CPU,
			true,
			true,
			true,
			Configuration::FAILUREMODE_RELOCALIZE,
			Configuration::SWAPPINGMODE_ENABLED,
			Configuration::LIBMODE_BASIC,
			Configuration::INDEX_ARRAY,
			GradientFunctorType::TRACKER_SLAVCHEVA_DIAGNOSTIC,
			Configuration::VERBOSITY_SILENT,
			"type=rgb,levels=rrbb"
	);
	//configuration1.save_to_json_file("../../Tests/TestData/config1.json");
	Configuration::load_configuration_from_json_file("TestData/config1.json");
	BOOST_REQUIRE_EQUAL(configuration1.voxel_volume_parameters, Configuration::get().voxel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.surfel_volume_parameters, Configuration::get().surfel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.slavcheva_parameters, Configuration::get().slavcheva_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.slavcheva_switches, Configuration::get().slavcheva_switches);
	BOOST_REQUIRE_EQUAL(configuration1.telemetry_settings, Configuration::get().telemetry_settings);
	BOOST_REQUIRE_EQUAL(configuration1.input_and_output_settings, Configuration::get().input_and_output_settings);
	BOOST_REQUIRE_EQUAL(configuration1.input_and_output_settings_paths, Configuration::get().input_and_output_settings_paths);
	BOOST_REQUIRE_EQUAL(configuration1.ui_engine_settings, Configuration::get().ui_engine_settings);
	BOOST_REQUIRE_EQUAL(configuration1, Configuration::get());
	configuration1.save_to_json_file("TestData/config2.json");
	Configuration::load_configuration_from_json_file("TestData/config2.json");
	BOOST_REQUIRE_EQUAL(configuration1.voxel_volume_parameters, Configuration::get().voxel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.surfel_volume_parameters, Configuration::get().surfel_volume_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.slavcheva_parameters, Configuration::get().slavcheva_parameters);
	BOOST_REQUIRE_EQUAL(configuration1.slavcheva_switches, Configuration::get().slavcheva_switches);
	BOOST_REQUIRE_EQUAL(configuration1.telemetry_settings, Configuration::get().telemetry_settings);
	BOOST_REQUIRE_EQUAL(configuration1.input_and_output_settings, Configuration::get().input_and_output_settings);
	BOOST_REQUIRE_EQUAL(configuration1.input_and_output_settings_paths, Configuration::get().input_and_output_settings_paths);
	BOOST_REQUIRE_EQUAL(configuration1.ui_engine_settings, Configuration::get().ui_engine_settings);
	BOOST_REQUIRE_EQUAL(configuration1, Configuration::get());


}