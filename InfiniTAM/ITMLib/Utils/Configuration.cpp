//  ================================================================
//  Created by Gregory Kramida on 11/08/19.
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
#include <unordered_map>
#include "Configuration.h"

using namespace ITMLib;

// region ============================= Boost variables_map processing subroutines =====================================

static Vector3i vector3i_from_variable_map(const po::variables_map& vm, std::string argument) {
	Vector3i vec;
	std::vector<int> int_vector = vm[argument].as<std::vector<int>>();
	if (int_vector.size() != 3) {
		DIEWITHEXCEPTION_REPORTLOCATION("Could not parse argument as exactly 3 integers, \"x y z\"");
	}
	memcpy(vec.values, int_vector.data(), sizeof(int) * 3);
	return vec;
}

static MemoryDeviceType memory_device_type_from_variable_map(const po::variables_map& vm, std::string argument) {
	static std::unordered_map<std::string, MemoryDeviceType> memory_device_type_by_string = {
			{"CPU",   MEMORYDEVICE_CPU},
			{"CUDA",  MEMORYDEVICE_CUDA},
			{"METAL", MEMORYDEVICE_METAL},
			{"cpu",   MEMORYDEVICE_CPU},
			{"cuda",  MEMORYDEVICE_CUDA},
			{"metal", MEMORYDEVICE_METAL},
			{"Metal", MEMORYDEVICE_METAL}
	};
	std::string argument_value = vm[argument].as<std::string>();
	if (memory_device_type_by_string.find(argument_value) == memory_device_type_by_string.end()) {
		DIEWITHEXCEPTION_REPORTLOCATION("Unrecognized memory device type argument");
	} else {
		return memory_device_type_by_string[argument_value];
	}
}

static Configuration::SwappingMode swapping_mode_from_variable_map(const po::variables_map& vm, std::string argument) {
	static std::unordered_map<std::string, Configuration::SwappingMode> swapping_mode_by_string = {
			{"enabled",  Configuration::SwappingMode::SWAPPINGMODE_ENABLED},
			{"disabled", Configuration::SwappingMode::SWAPPINGMODE_DISABLED},
			{"delete",   Configuration::SwappingMode::SWAPPINGMODE_DELETE}
	};
	std::string argument_value = vm[argument].as<std::string>();
	if (swapping_mode_by_string.find(argument_value) == swapping_mode_by_string.end()) {
		DIEWITHEXCEPTION_REPORTLOCATION("Unrecognized swapping mode argument");
	} else {
		return swapping_mode_by_string[argument_value];
	}
}

static Configuration::FailureMode failure_mode_from_variable_map(const po::variables_map& vm, std::string argument) {
	static std::unordered_map<std::string, Configuration::FailureMode> failure_mode_by_string = {
			{"ignore",           Configuration::FailureMode::FAILUREMODE_IGNORE},
			{"relocalize",       Configuration::FailureMode::FAILUREMODE_RELOCALIZE},
			{"stop_integration", Configuration::FailureMode::FAILUREMODE_STOP_INTEGRATION}
	};
	std::string argument_value = vm[argument].as<std::string>();
	if (failure_mode_by_string.find(argument_value) == failure_mode_by_string.end()) {
		DIEWITHEXCEPTION_REPORTLOCATION("Unrecognized failure mode argument");
	} else {
		return failure_mode_by_string[argument_value];
	}
}

static Configuration::LibMode lib_mode_from_variable_map(const po::variables_map& vm, std::string argument) {
	static std::unordered_map<std::string, Configuration::LibMode> lib_mode_by_string = {
			{"ignore",           Configuration::LibMode::LIBMODE_BASIC},
			{"relocalize",       Configuration::LibMode::LIBMODE_BASIC_SURFELS},
			{"stop_integration", Configuration::LibMode::LIBMODE_DYNAMIC},
			{"stop_integration", Configuration::LibMode::LIBMODE_LOOPCLOSURE}
	};
	std::string argument_value = vm[argument].as<std::string>();
	if (lib_mode_by_string.find(argument_value) == lib_mode_by_string.end()) {
		DIEWITHEXCEPTION_REPORTLOCATION("Unrecognized failure mode argument");
	} else {
		return lib_mode_by_string[argument_value];
	}
}

// endregion

Configuration::Configuration(const po::variables_map& vm) :
		scene_parameters(vm),
		surfel_scene_parameters(vm),
		slavcheva_parameters(vm),
		slavcheva_switches(vm),
		telemetry_settings(vm),
		skip_points(vm["skip_points"].empty() ?
		            Configuration().skip_points :
		            vm["skip_points"].as<bool>()),
		create_meshing_engine(vm["disable_meshing"].empty() ?
		                      Configuration().create_meshing_engine :
		                      !vm["disable_meshing"].as<bool>()),
		device_type(vm["device"].empty() ? Configuration().device_type :
		            memory_device_type_from_variable_map(vm, "device")),
		swapping_mode(vm["swapping"].empty() ? Configuration().swapping_mode :
		              swapping_mode_from_variable_map(vm, "swapping")),
		use_approximate_raycast(vm["use_approximate_raycast"].empty() ?
		                        Configuration().use_approximate_raycast :
		                        !vm["use_approximate_raycast"].as<bool>()),
		use_threshold_filter(vm["use_threshold_filter"].empty() ?
		                     Configuration().use_threshold_filter :
		                     !vm["use_threshold_filter"].as<bool>()),
		use_bilateral_filter(vm["use_bilateral_filter"].empty() ?
		                     Configuration().use_bilateral_filter :
		                     !vm["use_bilateral_filter"].as<bool>()),
		behavior_on_failure(vm["failure_mode"].empty() ? Configuration().behavior_on_failure :
		                    failure_mode_from_variable_map(vm, "failure_mode")),
		library_mode(vm["mode"].empty() ? Configuration().library_mode :
		             lib_mode_from_variable_map(vm, "mode")),
		tracker_configuration(
				vm["tracker"].empty() ? (library_mode == LIBMODE_BASIC_SURFELS ? default_surfel_tracker_configuration :
				                         default_depth_only_extended_tracker_configuration)
				                      : vm["tracker"].as<std::string>().c_str()),
		surface_tracking_max_optimization_iteration_count(vm["max_iterations"].empty() ?
		                                                  Configuration().surface_tracking_max_optimization_iteration_count
		                                                                               :
		                                                  vm["max_iterations"].as<unsigned int>()),
		surface_tracking_optimization_vector_update_threshold_meters(vm["vector_update_threshold"].empty() ?
		                                                             Configuration().surface_tracking_max_optimization_iteration_count
		                                                                                                   :
		                                                             vm["vector_update_threshold"].as<float>()) {
#ifdef COMPILE_WITHOUT_CUDA
	if(device_type == MEMORYDEVICE_CUDA){
		DIEWITHEXCEPTION_REPORTLOCATION("CUDA compilation disabled, unable to use CUDA device type. Aborting!");
	}
#endif
#ifdef COMPILE_WITHOUT_METAL
	if(device_type == MEMORYDEVICE_METAL){
		DIEWITHEXCEPTION_REPORTLOCATION("Metal compilation disabled, unable to use Metal device type. Aborting!");
	}
#endif
}

Configuration::Configuration() noexcept
		:   //mu(m), maxW, voxel size(m), clipping min, clipping max, stopIntegratingAtMaxW
		scene_parameters(0.04f, 100, 0.004f, 0.2f, 3.0f, false),//corresponds to KillingFusion article //_DEBUG
		//scene_parameters(0.02f, 100, 0.005f, 0.2f, 3.0f, false),//standard InfiniTAM values
		surfel_scene_parameters(0.5f, 0.6f, static_cast<float>(20 * M_PI / 180), 0.01f, 0.004f, 3.5f, 25.0f, 4, 1.0f,
		                        5.0f, 20, 10000000, true, true),
		slavcheva_parameters(SlavchevaSurfaceTracker::ConfigurationMode::SOBOLEV_FUSION,
		                     scene_parameters.voxelSize / scene_parameters.mu),
		slavcheva_switches(SlavchevaSurfaceTracker::ConfigurationMode::SOBOLEV_FUSION),
		telemetry_settings(),
		skip_points(true),
		create_meshing_engine(true),
#ifndef COMPILE_WITHOUT_CUDA
		device_type(MEMORYDEVICE_CUDA),
#else
		device_type(MEMORYDEVICE_CPU),
#endif
		swapping_mode(SWAPPINGMODE_DISABLED),
		use_approximate_raycast(false),
		use_threshold_filter(false),
		use_bilateral_filter(false),
		behavior_on_failure(FAILUREMODE_IGNORE),
		library_mode(LIBMODE_DYNAMIC),
		tracker_configuration(library_mode == LIBMODE_BASIC_SURFELS ? default_surfel_tracker_configuration :
		                      default_depth_only_extended_tracker_configuration),
		surface_tracking_max_optimization_iteration_count(200),
		surface_tracking_optimization_vector_update_threshold_meters(0.0001f) {
}

const std::string Configuration::default_ICP_tracker_configuration =
		"type=icp,levels=rrrbb,minstep=1e-3,"
		"outlierC=0.01,outlierF=0.002,"
		"numiterC=10,numiterF=2,failureDec=5.0"; // 5 for normal, 20 for loop closure
const std::string Configuration::default_ICP_tracker_configuration_loop_closure =
		"type=icp,levels=rrrbb,minstep=1e-3,"
		"outlierC=0.01,outlierF=0.002,"
		"numiterC=10,numiterF=2,failureDec=20.0"; // 5 for normal, 20 for loop closure
const std::string Configuration::default_depth_only_extended_tracker_configuration =
		"type=extended,levels=rrbb,useDepth=1,minstep=1e-4,"
		"outlierSpaceC=0.1,outlierSpaceF=0.004,"
		"numiterC=20,numiterF=50,tukeyCutOff=8,"
		"framesToSkip=20,framesToWeight=50,failureDec=20.0";
const std::string Configuration::default_intensity_depth_extended_tracker_configuration =
		"type=extended,levels=bbb,useDepth=1,useColour=1,"
		"colourWeight=0.3,minstep=1e-4,"
		"outlierColourC=0.175,outlierColourF=0.005,"
		"outlierSpaceC=0.1,outlierSpaceF=0.004,"
		"numiterC=20,numiterF=50,tukeyCutOff=8,"
		"framesToSkip=20,framesToWeight=50,failureDec=20.0";
const std::string Configuration::default_color_only_tracker_configuration =
		"type=rgb,levels=rrbb";
const std::string Configuration::default_IMU_ICP_tracker_configuration =
		"type=imuicp,levels=tb,minstep=1e-3,outlierC=0.01,"
		"outlierF=0.005,numiterC=4,numiterF=2";
const std::string Configuration::default_IMU_extended_tracker_configuration =
		"type=extendedimu,levels=ttb,minstep=5e-4,outlierSpaceC=0.1,"
		"outlierSpaceF=0.004,numiterC=20,numiterF=5,tukeyCutOff=8,"
		"framesToSkip=20,framesToWeight=50,failureDec=20.0";

const std::string Configuration::default_surfel_tracker_configuration =
		"extended,levels=rrbb,minstep=1e-4,outlierSpaceC=0.1,outlierSpaceF=0.004,"
		"numiterC=20,numiterF=20,tukeyCutOff=8,framesToSkip=0,framesToWeight=1,failureDec=20.0";

std::unique_ptr<Configuration> Configuration::instance = std::unique_ptr<Configuration>(new Configuration());


bool Configuration::FocusCoordinatesAreSpecified() const {
	return telemetry_settings.focus_coordinates_specified;
}

MemoryDeviceType Configuration::GetMemoryType() const {
	return device_type == MEMORYDEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
}

void Configuration::load_from_variable_map(const po::variables_map& vm) {
	instance.reset(new Configuration(vm));
}

void Configuration::load_default() {
	instance.reset(new Configuration);
}

Configuration& Configuration::get() {
	return *instance;
}


Configuration::TelemetrySettings::TelemetrySettings() :
		output_path("output/"),
		focus_coordinates_specified(false),
		focus_coordinates(Vector3i(0)) {}


Configuration::TelemetrySettings::TelemetrySettings(const po::variables_map& vm) :
		output_path(vm["output"].empty() ? TelemetrySettings().output_path : vm["output"].as<std::string>().c_str()),
		focus_coordinates_specified(!vm["focus_coordinates"].empty()),
		focus_coordinates(vm["focus_coordinates"].empty() ? TelemetrySettings().focus_coordinates :
		                  vector3i_from_variable_map(vm, "focus_coordinates")) {}
