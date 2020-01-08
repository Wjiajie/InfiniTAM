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

//stdlib
#include <unordered_map>
#include <utility>
#include <regex>

//boost
#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/preprocessor/stringize.hpp>

//local
#include "Configuration.h"
#include "json_utils.h"

using namespace ITMLib;



//** serializable enum definitions
DEFINE_SERIALIZABLE_ENUM( Configuration:: VERBOSITY_LEVEL_ENUM_DESCRIPTION )

// region =============== ENUM<--->STRING CONVERSIONS ==================================================================

template<>
MemoryDeviceType string_to_enumerator<MemoryDeviceType>(const std::string& string) {
	static std::unordered_map<std::string, MemoryDeviceType> memory_device_type_by_string = {
			{"CPU",   MEMORYDEVICE_CPU},
			{"CUDA",  MEMORYDEVICE_CUDA},
			{"METAL", MEMORYDEVICE_METAL},
			{"cpu",   MEMORYDEVICE_CPU},
			{"cuda",  MEMORYDEVICE_CUDA},
			{"metal", MEMORYDEVICE_METAL},
			{"Metal", MEMORYDEVICE_METAL}
	};
	if (memory_device_type_by_string.find(string) == memory_device_type_by_string.end()) {
		DIEWITHEXCEPTION_REPORTLOCATION("Unrecognized memory device type argument");
	} else {
		return memory_device_type_by_string[string];
	}
}

template<>
std::string enumerator_to_string<MemoryDeviceType>(const MemoryDeviceType& enum_value) {
	switch (enum_value) {
		case MEMORYDEVICE_CUDA:
			return "cuda";
		case MEMORYDEVICE_CPU:
			return "cpu";
		case MEMORYDEVICE_METAL:
			return "metal";
	}
}

template<>
Configuration::SwappingMode string_to_enumerator<Configuration::SwappingMode>(const std::string& string) {
	static std::unordered_map<std::string, Configuration::SwappingMode> swapping_mode_by_string = {
			{"enabled",  Configuration::SwappingMode::SWAPPINGMODE_ENABLED},
			{"disabled", Configuration::SwappingMode::SWAPPINGMODE_DISABLED},
			{"delete",   Configuration::SwappingMode::SWAPPINGMODE_DELETE}
	};
	if (swapping_mode_by_string.find(string) == swapping_mode_by_string.end()) {
		DIEWITHEXCEPTION_REPORTLOCATION("Unrecognized swapping mode argument");
	} else {
		return swapping_mode_by_string[string];
	}
}

template<>
std::string enumerator_to_string<Configuration::SwappingMode>(const Configuration::SwappingMode& enum_value) {
	switch (enum_value) {
		case Configuration::SwappingMode::SWAPPINGMODE_ENABLED:
			return "enabled";
		case Configuration::SwappingMode::SWAPPINGMODE_DISABLED:
			return "disabled";
		case Configuration::SwappingMode::SWAPPINGMODE_DELETE:
			return "delete";
	}
}

template<>
Configuration::FailureMode string_to_enumerator<Configuration::FailureMode>(const std::string& string) {
	static std::unordered_map<std::string, Configuration::FailureMode> failure_mode_by_string = {
			{"ignore",           Configuration::FailureMode::FAILUREMODE_IGNORE},
			{"relocalize",       Configuration::FailureMode::FAILUREMODE_RELOCALIZE},
			{"stop_integration", Configuration::FailureMode::FAILUREMODE_STOP_INTEGRATION}
	};
	if (failure_mode_by_string.find(string) == failure_mode_by_string.end()) {
		DIEWITHEXCEPTION_REPORTLOCATION("Unrecognized failure mode argument");
	} else {
		return failure_mode_by_string[string];
	}
}

template<>
std::string enumerator_to_string<Configuration::FailureMode>(const Configuration::FailureMode& enum_value) {
	switch (enum_value) {
		case Configuration::FailureMode::FAILUREMODE_IGNORE:
			return "ignore";
		case Configuration::FailureMode::FAILUREMODE_RELOCALIZE:
			return "relocalize";
		case Configuration::FailureMode::FAILUREMODE_STOP_INTEGRATION:
			return "stop_integration";
	}
}

template<>
Configuration::LibMode string_to_enumerator<Configuration::LibMode>(const std::string& string) {
	static std::unordered_map<std::string, Configuration::LibMode> lib_mode_by_string = {
			{"basic",        Configuration::LibMode::LIBMODE_BASIC},
			{"surfels",      Configuration::LibMode::LIBMODE_BASIC_SURFELS},
			{"dynamic",      Configuration::LibMode::LIBMODE_DYNAMIC},
			{"loop_closure", Configuration::LibMode::LIBMODE_LOOPCLOSURE}
	};
	if (lib_mode_by_string.find(string) == lib_mode_by_string.end()) {
		DIEWITHEXCEPTION_REPORTLOCATION("Unrecognized failure mode argument");
	} else {
		return lib_mode_by_string[string];
	}
}

template<>
std::string enumerator_to_string<Configuration::LibMode>(const Configuration::LibMode& enum_value) {
	switch (enum_value) {
		case Configuration::LibMode::LIBMODE_BASIC:
			return "basic";
		case Configuration::LibMode::LIBMODE_BASIC_SURFELS:
			return "surfels";
		case Configuration::LibMode::LIBMODE_DYNAMIC:
			return "dynamic";
		case Configuration::LibMode::LIBMODE_LOOPCLOSURE:
			return "loop_closure";
	}
}

template<>
Configuration::IndexingMethod string_to_enumerator<Configuration::IndexingMethod>(const std::string& string) {
	static std::unordered_map<std::string, Configuration::IndexingMethod> indexing_method_by_string = {
			{"array", Configuration::IndexingMethod::INDEX_ARRAY},
			{"hash",  Configuration::IndexingMethod::INDEX_HASH},
			{"ARRAY", Configuration::IndexingMethod::INDEX_ARRAY},
			{"HASH",  Configuration::IndexingMethod::INDEX_HASH}
	};
	if (indexing_method_by_string.find(string) == indexing_method_by_string.end()) {
		DIEWITHEXCEPTION_REPORTLOCATION("Unrecognized indexing method argument");
	} else {
		return indexing_method_by_string[string];
	}
}

template<>
std::string enumerator_to_string<Configuration::IndexingMethod>(const Configuration::IndexingMethod& enum_value) {
	switch (enum_value) {
		case Configuration::IndexingMethod::INDEX_ARRAY:
			return "array";
		case Configuration::IndexingMethod::INDEX_HASH:
			return "hash";
	}
}

template<>
SlavchevaSurfaceTracker::ConfigurationMode string_to_enumerator<SlavchevaSurfaceTracker::ConfigurationMode>(
		const std::string& string) {
	static std::unordered_map<std::string, SlavchevaSurfaceTracker::ConfigurationMode> indexing_method_by_string = {
			{"killing", SlavchevaSurfaceTracker::ConfigurationMode::KILLING_FUSION},
			{"sobolev", SlavchevaSurfaceTracker::ConfigurationMode::SOBOLEV_FUSION},
			{"Killing", SlavchevaSurfaceTracker::ConfigurationMode::KILLING_FUSION},
			{"Sobolev", SlavchevaSurfaceTracker::ConfigurationMode::SOBOLEV_FUSION}
	};
	if (indexing_method_by_string.find(string) == indexing_method_by_string.end()) {
		DIEWITHEXCEPTION_REPORTLOCATION("Unrecognized slavcheva mode argument");
	} else {
		return indexing_method_by_string[string];
	}
}

template<>
std::string enumerator_to_string<SlavchevaSurfaceTracker::ConfigurationMode>(
		const SlavchevaSurfaceTracker::ConfigurationMode& enum_value) {
	switch (enum_value) {
		case SlavchevaSurfaceTracker::ConfigurationMode::KILLING_FUSION:
			return "killing";
		case SlavchevaSurfaceTracker::ConfigurationMode::SOBOLEV_FUSION:
			return "sobolev";
	}
}

template<>
GradientFunctorType string_to_enumerator<GradientFunctorType>(const std::string& string) {
	static std::unordered_map<std::string, GradientFunctorType> surface_tracking_functor_by_string = {
			{"slavcheva_diagnostic", GradientFunctorType::TRACKER_SLAVCHEVA_DIAGNOSTIC},
			{"slavcheva_optimized",  GradientFunctorType::TRACKER_SLAVCHEVA_OPTIMIZED},
			{"Slavcheva_diagnostic", GradientFunctorType::TRACKER_SLAVCHEVA_DIAGNOSTIC},
			{"Slavcheva_optimized",  GradientFunctorType::TRACKER_SLAVCHEVA_OPTIMIZED}
	};
	if (surface_tracking_functor_by_string.find(string) == surface_tracking_functor_by_string.end()) {
		DIEWITHEXCEPTION_REPORTLOCATION("Unrecognized slavcheva mode argument");
	} else {
		return surface_tracking_functor_by_string[string];
	}
}


template<>
std::string enumerator_to_string<GradientFunctorType>(
		const GradientFunctorType& enum_value) {
	switch (enum_value) {
		case GradientFunctorType::TRACKER_SLAVCHEVA_OPTIMIZED:
			return "slavcheva_optimized";
		case GradientFunctorType::TRACKER_SLAVCHEVA_DIAGNOSTIC:
			return "slavcheva_diagnostic";
	}
}
// endregion ===========================================================================================================

// region ====================================== boost::variables_map PROCESSING ROUTINES ==============================

static Vector3i vector3i_from_std_vector(const std::vector<int>& int_vector) {
	Vector3i vec;
	if (int_vector.size() != 3) {
		DIEWITHEXCEPTION_REPORTLOCATION("Could not parse argument as exactly 3 integers, \"x y z\"");
	}
	memcpy(vec.values, int_vector.data(), sizeof(int) * 3);
	return vec;
}

static Vector3i vector3i_from_variable_map(const po::variables_map& vm, const std::string& argument) {
	std::vector<int> int_vector = vm[argument].as<std::vector<int>>();
	return vector3i_from_std_vector(int_vector);
}

template<typename TEnum>
static TEnum enumerator_from_variable_map(const po::variables_map& vm, const std::string& argument){
	return string_to_enumerator<TEnum>(vm[argument].as<std::string>());
}
// endregion ===========================================================================================================

// region ================================== boost::property_tree::ptree PROCESSING ROUTINES ===========================

template<typename TEnum>
static boost::optional<TEnum> optional_enum_value_from_ptree(const pt::ptree& ptree, const pt::ptree::key_type& key) {
	auto child = ptree.get_child_optional(key);
	if (child) {
		return boost::optional<TEnum>(string_to_enumerator<TEnum>(ptree.get<std::string>(key)));
	} else {
		return boost::optional<TEnum>{};
	}
}

template<typename T>
static
boost::optional<std::vector<T>> as_optional_vector(pt::ptree const& pt, pt::ptree::key_type const& key) {
	if (pt.count(key) == 0) {
		return boost::optional<std::vector<T>>{};
	}
	std::vector<T> r;
	for (auto& item : pt.get_child(key))
		r.push_back(item.second.get_value<T>());
	return boost::optional<std::vector<T>>(r);
}

template<typename T>
static
std::vector<T> as_vector(pt::ptree const& pt, pt::ptree::key_type const& key) {
	std::vector<T> r;
	for (auto& item : pt.get_child(key))
		r.push_back(item.second.get_value<T>());
	return r;
}
// endregion ===========================================================================================================

// region ======================================= CONFIGURATION CONSTRUCTORS ===========================================

Configuration::Configuration()
		:   //narrow_band_half_width(m), max_integration_weight, voxel size(m), clipping min, clipping max, stop_integration_at_max_weight
		voxel_volume_parameters(),
		//voxel_volume_parameters(0.02f, 100, 0.005f, 0.2f, 3.0f, false),//standard InfiniTAM values
		surfel_volume_parameters(0.5f, 0.6f, static_cast<float>(20 * M_PI / 180), 0.01f, 0.004f, 3.5f, 25.0f, 4, 1.0f,
		                        5.0f, 20, 10000000, true, true),
		slavcheva_parameters(SlavchevaSurfaceTracker::ConfigurationMode::SOBOLEV_FUSION),
		slavcheva_switches(SlavchevaSurfaceTracker::ConfigurationMode::SOBOLEV_FUSION),
		telemetry_settings(),
		input_and_output_settings(),
		input_and_output_settings_paths(),
		ui_engine_settings(),
		non_rigid_tracking_parameters(),
		skip_points(true),
		create_meshing_engine(true),
#ifndef COMPILE_WITHOUT_CUDA
		device_type(MEMORYDEVICE_CUDA),
#else
		device_type(MEMORYDEVICE_CPU),
#endif
		use_approximate_raycast(false),
		use_threshold_filter(false),
		use_bilateral_filter(false),
		behavior_on_failure(FAILUREMODE_IGNORE),
		swapping_mode(SWAPPINGMODE_DISABLED),
		library_mode(LIBMODE_DYNAMIC),
		indexing_method(INDEX_HASH),
		surface_tracker_type(TRACKER_SLAVCHEVA_DIAGNOSTIC),
		verbosity_level(VERBOSITY_PER_FRAME),
		tracker_configuration(library_mode == LIBMODE_BASIC_SURFELS ? default_surfel_tracker_configuration :
		                      default_depth_only_extended_tracker_configuration) {
}

Configuration::Configuration(const po::variables_map& vm) :
		voxel_volume_parameters(vm),
		surfel_volume_parameters(vm),
		slavcheva_parameters(vm),
		slavcheva_switches(vm),
		telemetry_settings(vm),
		input_and_output_settings(vm),
		input_and_output_settings_paths(vm),
		ui_engine_settings(vm),
		non_rigid_tracking_parameters(vm),
		skip_points(vm["skip_points"].empty() ?
		            Configuration().skip_points :
		            vm["skip_points"].as<bool>()),
		create_meshing_engine(vm["disable_meshing"].empty() ?
		                      Configuration().create_meshing_engine :
		                      !vm["disable_meshing"].as<bool>()),
		device_type(vm["device"].empty() ? Configuration().device_type :
		            enumerator_from_variable_map<MemoryDeviceType>(vm, "device")),
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
		                    enumerator_from_variable_map<Configuration::FailureMode>(vm, "failure_mode")),
		swapping_mode(vm["swapping"].empty() ? Configuration().swapping_mode :
		              enumerator_from_variable_map<Configuration::SwappingMode>(vm, "swapping")),
		library_mode(vm["mode"].empty() ? Configuration().library_mode :
		             enumerator_from_variable_map<Configuration::LibMode>(vm, "mode")),
		indexing_method(vm["index"].empty() ? Configuration().indexing_method :
		                enumerator_from_variable_map<Configuration::IndexingMethod>(vm, "index")),
		surface_tracker_type(vm["surface_tracker_type"].empty() ? Configuration().surface_tracker_type :
		                     enumerator_from_variable_map<GradientFunctorType>(vm, "surface_tracker_type")),
		verbosity_level(vm["verbosity_level"].empty() ? Configuration().verbosity_level :
                        enumerator_from_variable_map<Configuration::VerbosityLevel>(vm, "verbosity_level")),
		tracker_configuration(
				vm["tracker"].empty() ? (library_mode == LIBMODE_BASIC_SURFELS ? default_surfel_tracker_configuration :
				                         default_depth_only_extended_tracker_configuration)
				                      : vm["tracker"].as<std::string>()){
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


Configuration::Configuration(VoxelVolumeParameters voxel_volume_parameters,
                             ITMSurfelSceneParameters surfel_volume_parameters,
                             SlavchevaSurfaceTracker::Parameters slavcheva_parameters,
                             SlavchevaSurfaceTracker::Switches slavcheva_switches,
                             Configuration::TelemetrySettings telemetry_settings,
                             Configuration::InputAndOutputSettings input_and_output_settings,
                             Configuration::InputAndOutputSettings_Paths input_and_output_settings_paths,
                             Configuration::UIEngineSettings ui_engine_settings,
                             NonRigidTrackingParameters non_rigid_tracking_parameters, bool skip_points,
                             bool create_meshing_engine, MemoryDeviceType device_type, bool use_approximate_raycast,
                             bool use_threshold_filter, bool use_bilateral_filter,
                             Configuration::FailureMode behavior_on_failure,
                             Configuration::SwappingMode swapping_mode, Configuration::LibMode library_mode,
                             Configuration::IndexingMethod indexing_method,
                             GradientFunctorType surface_tracker_type,
                             Configuration::VerbosityLevel verbosity_level,
                             std::string tracker_configuration) :

		voxel_volume_parameters(voxel_volume_parameters),
		surfel_volume_parameters(surfel_volume_parameters),
		slavcheva_parameters(slavcheva_parameters),
		slavcheva_switches(slavcheva_switches),
		telemetry_settings(telemetry_settings),
		ui_engine_settings(ui_engine_settings),
		non_rigid_tracking_parameters(non_rigid_tracking_parameters),
		input_and_output_settings(input_and_output_settings),
		input_and_output_settings_paths(std::move(input_and_output_settings_paths)),
		skip_points(skip_points),
		create_meshing_engine(create_meshing_engine),
		device_type(device_type),
		use_approximate_raycast(use_approximate_raycast),
		use_threshold_filter(use_threshold_filter),
		use_bilateral_filter(use_bilateral_filter),
		behavior_on_failure(behavior_on_failure),
		swapping_mode(swapping_mode),
		library_mode(library_mode),
		indexing_method(indexing_method),
		surface_tracker_type(surface_tracker_type),
		verbosity_level(verbosity_level),
		tracker_configuration(std::move(tracker_configuration)) {}

// endregion ===========================================================================================================

// region ===================================== CONFIGURATION CONSTANT DEFINITIONS =====================================

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

// endregion ===========================================================================================================

// region ==== CONFIGURATION SINGLETON HANDLING, variables_map & ptree CONVERSIONS, COMPARISONS ========================

std::unique_ptr<Configuration> Configuration::instance = std::unique_ptr<Configuration>(new Configuration());


void Configuration::load_configuration_from_variable_map(const po::variables_map& vm) {
	instance.reset(new Configuration(vm));
}

void Configuration::load_default() {
	instance.reset(new Configuration);
}

Configuration& Configuration::get() {
	return *instance;
}

namespace fs = boost::filesystem;


void Configuration::load_configuration_from_json_file(const std::string& path) {
	instance.reset(from_json_file(path));
}

template<typename TJsonParsable>
static boost::optional<TJsonParsable>
as_optional_parsable_config_path(const pt::ptree& tree, pt::ptree::key_type const& key,
                                 const std::string& config_path) {
	auto subtree = tree.get_child_optional(key);
	if (subtree) {
		return boost::optional<TJsonParsable>(TJsonParsable::BuildFromPTree(subtree.get(), config_path));
	} else {
		return boost::optional<TJsonParsable>{};
	}
}

template<typename TJsonParsable>
static boost::optional<TJsonParsable> as_optional_parsable(const pt::ptree& tree, pt::ptree::key_type const& key) {
	auto subtree = tree.get_child_optional(key);
	if (subtree) {
		return boost::optional<TJsonParsable>(TJsonParsable::BuildFromPTree(subtree.get()));
	} else {
		return boost::optional<TJsonParsable>{};
	}
}

template<typename TJsonParsable>
static boost::optional<TJsonParsable>
as_optional_parsable_slavcheva(const pt::ptree& tree, pt::ptree::key_type const& key,
                               SlavchevaSurfaceTracker::ConfigurationMode mode) {
	auto subtree = tree.get_child_optional(key);
	if (subtree) {
		return boost::optional<TJsonParsable>(TJsonParsable::BuildFromPTree(subtree.get(), mode));
	} else {
		return boost::optional<TJsonParsable>{};
	}
}

Configuration* Configuration::from_json_file(const std::string& path) {
	pt::ptree tree;
	pt::read_json(path, tree);

	Configuration default_configuration;

	boost::optional<VoxelVolumeParameters> voxel_volume_parameters =
			as_optional_parsable<VoxelVolumeParameters>(tree, "voxel_volume_parameters");
	boost::optional<ITMSurfelSceneParameters> surfel_volume_parameters =
			as_optional_parsable<ITMSurfelSceneParameters>(tree, "surfel_volume_parameters");
	boost::optional<SlavchevaSurfaceTracker::ConfigurationMode> mode_opt =
			optional_enum_value_from_ptree<SlavchevaSurfaceTracker::ConfigurationMode>(tree, "slavcheva.preset");
	SlavchevaSurfaceTracker::ConfigurationMode mode = mode_opt ? mode_opt.get()
	                                                           : SlavchevaSurfaceTracker::SOBOLEV_FUSION;
	boost::optional<SlavchevaSurfaceTracker::Parameters> slavcheva_parameters =
			as_optional_parsable_slavcheva<SlavchevaSurfaceTracker::Parameters>(tree, "slavcheva.parameters", mode);
	boost::optional<SlavchevaSurfaceTracker::Switches> slavcheva_switches =
			as_optional_parsable_slavcheva<SlavchevaSurfaceTracker::Switches>(tree, "slavcheva.switches", mode);
	boost::optional<TelemetrySettings> telemetry_settings =
			as_optional_parsable<TelemetrySettings>(tree, "telemetry_settings");

	boost::optional<InputAndOutputSettings> input_and_output_settings =
			as_optional_parsable<InputAndOutputSettings>(tree, "input_and_output_settings");
	boost::optional<InputAndOutputSettings_Paths> input_and_output_settings_paths =
			as_optional_parsable_config_path<InputAndOutputSettings_Paths>(tree, "input_and_output_settings.paths", path);

	boost::optional<UIEngineSettings> ui_engine_settings =
			as_optional_parsable<UIEngineSettings>(tree,"ui_engine_settings");
	boost::optional<NonRigidTrackingParameters> non_rigid_tracking_parameters =
			as_optional_parsable<NonRigidTrackingParameters>(tree,"non_rigid_tracking_parameters");

	boost::optional<bool> skip_points = tree.get_optional<bool>("skip_points");
	boost::optional<bool> disable_meshing = tree.get_optional<bool>("disable_meshing");
	boost::optional<MemoryDeviceType> device_type = optional_enum_value_from_ptree<MemoryDeviceType>(tree, "device");
	boost::optional<bool> use_approximate_raycast = tree.get_optional<bool>("use_approximate_raycast");
	boost::optional<bool> use_threshold_filter = tree.get_optional<bool>("use_threshold_filter");
	boost::optional<bool> use_bilateral_filter = tree.get_optional<bool>("use_bilateral_filter");
	boost::optional<Configuration::FailureMode> behavior_on_failure = optional_enum_value_from_ptree<FailureMode>(tree,
	                                                                                                              "failure_mode");
	boost::optional<Configuration::SwappingMode> swapping_mode = optional_enum_value_from_ptree<Configuration::SwappingMode>(
			tree, "swapping");
	boost::optional<Configuration::LibMode> library_mode = optional_enum_value_from_ptree<LibMode>(tree, "mode");
	boost::optional<Configuration::IndexingMethod> indexing_method = optional_enum_value_from_ptree<IndexingMethod>(
			tree, "index");
	boost::optional<GradientFunctorType> surface_tracker_type = optional_enum_value_from_ptree<GradientFunctorType>(
			tree, "non_rigid_tracking_parameters.functor_type");
	boost::optional<Configuration::VerbosityLevel> verbosity_level = optional_enum_value_from_ptree<VerbosityLevel>(tree, "verbosity_level");
	boost::optional<std::string> tracker_configuration = tree.get_optional<std::string>("tracker");


	Configuration default_config;

	return new Configuration(
			voxel_volume_parameters ? voxel_volume_parameters.get() : default_config.voxel_volume_parameters,
			surfel_volume_parameters ? surfel_volume_parameters.get() : default_config.surfel_volume_parameters,
			slavcheva_parameters ? slavcheva_parameters.get() : default_config.slavcheva_parameters,
			slavcheva_switches ? slavcheva_switches.get() : default_config.slavcheva_switches,
			telemetry_settings ? telemetry_settings.get() : default_config.telemetry_settings,
			input_and_output_settings ? input_and_output_settings.get() : default_config.input_and_output_settings,
			input_and_output_settings_paths ? input_and_output_settings_paths.get()
			                                : default_config.input_and_output_settings_paths,
			ui_engine_settings ? ui_engine_settings.get() : default_config.ui_engine_settings,
			non_rigid_tracking_parameters ? non_rigid_tracking_parameters.get()
			                              : default_config.non_rigid_tracking_parameters,
			skip_points ? skip_points.get() : default_config.skip_points,
			disable_meshing ? !disable_meshing.get() : default_config.create_meshing_engine,
			device_type ? device_type.get() : default_config.device_type,
			use_approximate_raycast ? use_approximate_raycast.get() : default_config.use_approximate_raycast,
			use_threshold_filter ? use_threshold_filter.get() : default_config.use_threshold_filter,
			use_bilateral_filter ? use_bilateral_filter.get() : default_config.use_bilateral_filter,
			behavior_on_failure ? behavior_on_failure.get() : default_config.behavior_on_failure,
			swapping_mode ? swapping_mode.get() : default_config.swapping_mode,
			library_mode ? library_mode.get() : default_config.library_mode,
			indexing_method ? indexing_method.get() : default_config.indexing_method,
			surface_tracker_type ? surface_tracker_type.get() : default_config.surface_tracker_type,
			verbosity_level ? verbosity_level.get() : default_config.verbosity_level,
			tracker_configuration ? tracker_configuration.get() : default_config.tracker_configuration
	);
}


pt::ptree Configuration::to_ptree(const std::string& path) const {
	pt::ptree tree;
	tree.add_child("voxel_volume_parameters", this->voxel_volume_parameters.ToPTree());
	tree.add_child("surfel_volume_parameters", this->surfel_volume_parameters.ToPTree());
	tree.add("slavcheva.preset", enumerator_to_string(SlavchevaSurfaceTracker::ConfigurationMode::SOBOLEV_FUSION));
	tree.add_child("slavcheva.parameters", this->slavcheva_parameters.ToPTree());
	tree.add_child("slavcheva.switches", slavcheva_switches.ToPTree());
	tree.add_child("telemetry_settings", this->telemetry_settings.ToPTree());
	tree.add_child("input_and_output_settings", this->input_and_output_settings.ToPTree());
	tree.add_child("input_and_output_settings.paths", this->input_and_output_settings_paths.ToPTree(path));
	tree.add_child("ui_engine_settings", this->ui_engine_settings.ToPTree());
	tree.add_child("non_rigid_tracking_parameters", this->non_rigid_tracking_parameters.ToPTree());
	tree.add("skip_points", skip_points);
	tree.add("disable_meshing", !create_meshing_engine);
	tree.add("device", enumerator_to_string(this->device_type));
	tree.add("use_approximate_raycast", this->use_approximate_raycast);
	tree.add("use_threshold_filter", this->use_threshold_filter);
	tree.add("use_bilateral_filter", this->use_bilateral_filter);
	tree.add("failure_mode", enumerator_to_string(this->behavior_on_failure));
	tree.add("swapping", enumerator_to_string(this->swapping_mode));
	tree.add("mode", enumerator_to_string(this->library_mode));
	tree.add("index", enumerator_to_string(this->indexing_method));
	tree.add("non_rigid_tracking_parameters.functor_type", enumerator_to_string(this->surface_tracker_type));
	tree.add("verbosity_level", enumerator_to_string(this->verbosity_level));
	tree.add("tracker", this->tracker_configuration);
	return tree;
}

void Configuration::save_configuration_to_json_file(const std::string& path) {
	pt::write_json_no_quotes(path, instance->to_ptree(path), true);
}


void Configuration::save_to_json_file(const std::string& path) {
	pt::write_json_no_quotes(path, this->to_ptree(path), true);
}


namespace ITMLib {
bool operator==(const Configuration& c1, const Configuration& c2) {
	return c1.voxel_volume_parameters == c2.voxel_volume_parameters &&
	       c1.surfel_volume_parameters == c2.surfel_volume_parameters &&
	       c1.slavcheva_parameters == c2.slavcheva_parameters &&
	       c1.slavcheva_switches == c2.slavcheva_switches &&
	       c1.telemetry_settings == c2.telemetry_settings &&
	       c1.input_and_output_settings == c2.input_and_output_settings &&
	       c1.input_and_output_settings_paths == c2.input_and_output_settings_paths &&
	       c1.ui_engine_settings == c2.ui_engine_settings &&
	       c1.non_rigid_tracking_parameters == c2.non_rigid_tracking_parameters &&
	       c1.skip_points == c2.skip_points &&
	       c1.create_meshing_engine == c2.create_meshing_engine &&
	       c1.device_type == c2.device_type &&
	       c1.use_approximate_raycast == c2.use_approximate_raycast &&
	       c1.use_threshold_filter == c2.use_threshold_filter &&
	       c1.use_bilateral_filter == c2.use_bilateral_filter &&
	       c1.behavior_on_failure == c2.behavior_on_failure &&
	       c1.swapping_mode == c2.swapping_mode &&
	       c1.library_mode == c2.library_mode &&
	       c1.indexing_method == c2.indexing_method &&
	       c1.surface_tracker_type == c2.surface_tracker_type &&
	       c1.verbosity_level == c2.verbosity_level &&
	       c1.tracker_configuration == c2.tracker_configuration;
}

std::ostream& operator<<(std::ostream& out, const Configuration& c) {
	pt::write_json_no_quotes(out, c.to_ptree(""), true);
	out << "";
	return out;
}
}//namespace ITMLib
// endregion ===========================================================================================================

// region ====================================== INPUT AND OUTPUT SETTINGS =============================================

bool isPathMask(const std::string& arg) {
	return arg.find('%') != std::string::npos;
}

Configuration::InputAndOutputSettings_Paths::InputAndOutputSettings_Paths(const po::variables_map& vm) :
		output_path(vm["output"].empty() ? InputAndOutputSettings_Paths().output_path : vm["output"].as<std::string>()),
		calibration_file_path(vm["calibration_file"].empty() ? InputAndOutputSettings_Paths().calibration_file_path
		                                                     : vm["calibration_file"].as<std::string>())
        {
	std::vector<std::string> inputPaths;
	if (vm.count("input_path")) {
		inputPaths = vm["input_path"].as<std::vector<std::string>>();
	}
	auto inputFileCount = inputPaths.size();
	switch (inputFileCount) {
		case 0:
			//no input files
			break;
		case 1:
			//a single OpenNI file
			openni_file_path = inputPaths[0];
			break;
		case 3:
		default:
			if (isPathMask(inputPaths[2])) { mask_image_path_mask = inputPaths[2]; }
			else { imu_input_path = inputPaths[2]; }
		case 2:
			if (isPathMask(inputPaths[0]) && isPathMask(inputPaths[1])) {
				rgb_image_path_mask = inputPaths[0];
				depth_image_path_mask = inputPaths[1];
			} else if (!isPathMask(inputPaths[0]) && !isPathMask(inputPaths[1])) {
				rgb_video_file_path = inputPaths[0];
				depth_video_file_path = inputPaths[1];
			} else {
				std::cerr << "The first & second input_path arguments need to either both be masks or both be"
				             " paths to video files." << std::endl;

				throw std::runtime_error("Could not parse command-line arguments");
			}
			break;
	}
}

Configuration::InputAndOutputSettings_Paths::InputAndOutputSettings_Paths() :
		output_path("output/"),
		calibration_file_path("calib.txt"){}

Configuration::InputAndOutputSettings_Paths::InputAndOutputSettings_Paths(std::string output_path,
                                                                          std::string calibration_file_path,
                                                                          std::string openni_file_path,
                                                                          std::string rgb_video_file_path,
                                                                          std::string depth_video_file_path,
                                                                          std::string rgb_image_path_mask,
                                                                          std::string depth_image_path_mask,
                                                                          std::string mask_image_path_mask,
                                                                          std::string imu_input_path) :
		output_path(std::move(output_path)),
		calibration_file_path(std::move(calibration_file_path)),
		openni_file_path(std::move(openni_file_path)),
		rgb_video_file_path(std::move(rgb_video_file_path)),
		depth_video_file_path(std::move(depth_video_file_path)),
		rgb_image_path_mask(std::move(rgb_image_path_mask)),
		depth_image_path_mask(std::move(depth_image_path_mask)),
		mask_image_path_mask(std::move(mask_image_path_mask)),
		imu_input_path(std::move(imu_input_path))
		{}


Configuration::InputAndOutputSettings_Paths
Configuration::InputAndOutputSettings_Paths::BuildFromPTree(const pt::ptree& tree, const std::string& config_path) {
	boost::optional<std::string> output_path_opt = tree.get_optional<std::string>("output");
	std::string output_path,
			calibration_file_path,
			openni_file_path,
			rgb_video_file_path,
			depth_video_file_path,
			rgb_image_path_mask,
			depth_image_path_mask,
			mask_image_path_mask,
			imu_input_path;
	auto value_or_empty_string = [&config_path](boost::optional<std::string> optional) {
		return optional ? preprocess_path(optional.get(), config_path) : "";
	};
	calibration_file_path = value_or_empty_string(tree.get_optional<std::string>("calibration_file_path"));
	openni_file_path = value_or_empty_string(tree.get_optional<std::string>("openni_file_path"));
	rgb_video_file_path = value_or_empty_string(tree.get_optional<std::string>("rgb_video_file_path"));
	depth_video_file_path = value_or_empty_string(tree.get_optional<std::string>("depth_video_file_path"));
	rgb_image_path_mask = value_or_empty_string(tree.get_optional<std::string>("rgb_image_path_mask"));
	depth_image_path_mask = value_or_empty_string(tree.get_optional<std::string>("depth_image_path_mask"));
	mask_image_path_mask = value_or_empty_string(tree.get_optional<std::string>("mask_image_path_mask"));
	imu_input_path = value_or_empty_string(tree.get_optional<std::string>("imp_input_path"));
	InputAndOutputSettings_Paths default_io_settings;

	return {output_path_opt ? preprocess_path(output_path_opt.get(), config_path) : default_io_settings.output_path,
	        calibration_file_path,
	        openni_file_path,
	        rgb_video_file_path,
	        depth_video_file_path,
	        rgb_image_path_mask,
	        depth_image_path_mask,
	        mask_image_path_mask,
	        imu_input_path};
}

pt::ptree Configuration::InputAndOutputSettings_Paths::ToPTree(const std::string& config_path) const {
	pt::ptree tree;
	tree.add("output", this->output_path);
	auto add_to_tree_if_not_empty = [&config_path](pt::ptree& tree, const std::string& key, const std::string& string) {
		if (string != "") {
			tree.add(key, postprocess_path(string, config_path));
		}
	};
	add_to_tree_if_not_empty(tree, "calibration_file_path", calibration_file_path);
	add_to_tree_if_not_empty(tree, "openni_file_path", openni_file_path);
	add_to_tree_if_not_empty(tree, "rgb_video_file_path", rgb_video_file_path);
	add_to_tree_if_not_empty(tree, "depth_video_file_path", depth_video_file_path);
	add_to_tree_if_not_empty(tree, "rgb_image_path_mask", rgb_image_path_mask);
	add_to_tree_if_not_empty(tree, "depth_image_path_mask", depth_image_path_mask);
	add_to_tree_if_not_empty(tree, "mask_image_path_mask", mask_image_path_mask);
	add_to_tree_if_not_empty(tree, "imu_input_path", imu_input_path);
	return tree;
}

namespace ITMLib {

bool operator==(const Configuration::InputAndOutputSettings_Paths& ios1, const Configuration::InputAndOutputSettings_Paths& ios2) {
	return ios1.output_path == ios2.output_path &&
	ios1.calibration_file_path == ios2.calibration_file_path &&
	ios1.openni_file_path == ios2.openni_file_path &&
	ios1.rgb_video_file_path == ios2.rgb_video_file_path &&
	ios1.depth_video_file_path == ios2.depth_video_file_path &&
	ios1.rgb_image_path_mask == ios2.rgb_image_path_mask &&
	ios1.depth_image_path_mask == ios2.depth_image_path_mask &&
	ios1.mask_image_path_mask == ios2.mask_image_path_mask &&
	ios1.imu_input_path == ios2.imu_input_path
	;
}

std::ostream& operator<<(std::ostream& out, const Configuration::InputAndOutputSettings_Paths& ios) {
	pt::ptree tree(ios.ToPTree(""));
	pt::write_json_no_quotes(out, tree, true);
	return out;
}

} // namespace ITMLib

// endregion ===========================================================================================================

// region =============================== TELEMETRY SETTINGS ===========================================================

Configuration::TelemetrySettings::TelemetrySettings() :
		focus_coordinates_specified(false),
		focus_coordinates(Vector3i(0)) {}


Configuration::TelemetrySettings::TelemetrySettings(const po::variables_map& vm) :
		focus_coordinates_specified(!vm["focus_coordinates"].empty()),
		focus_coordinates(vm["focus_coordinates"].empty() ? TelemetrySettings().focus_coordinates :
		                  vector3i_from_variable_map(vm, "focus_coordinates")) {}


Configuration::TelemetrySettings Configuration::TelemetrySettings::BuildFromPTree(const pt::ptree& tree) {

	boost::optional<std::vector<int>> focus_coords_opt = as_optional_vector<int>(tree, "focus_coordinates");

	TelemetrySettings default_ts;

	return {(bool) focus_coords_opt,
	        focus_coords_opt ? vector3i_from_std_vector(focus_coords_opt.get()) : default_ts.focus_coordinates};
}

Configuration::TelemetrySettings::TelemetrySettings(bool focus_coordinates_specified,
                                                    Vector3i focus_coordinates) :
		focus_coordinates_specified(focus_coordinates_specified),
		focus_coordinates(focus_coordinates) {}

pt::ptree Configuration::TelemetrySettings::ToPTree() const {
	pt::ptree tree;
	if (focus_coordinates_specified) {
		pt::ptree children;
		pt::ptree child1, child2, child3;
		child1.put("", focus_coordinates.x);
		child2.put("", focus_coordinates.y);
		child3.put("", focus_coordinates.z);
		children.push_back(std::make_pair("", child1));
		children.push_back(std::make_pair("", child2));
		children.push_back(std::make_pair("", child3));
		tree.add_child("focus_coordinates", children);
	}
	return tree;
}

namespace ITMLib {
bool operator==(const Configuration::TelemetrySettings& ts1, const Configuration::TelemetrySettings& ts2) {
	return ts1.focus_coordinates_specified == ts2.focus_coordinates_specified &&
	       (!ts1.focus_coordinates_specified || ts1.focus_coordinates == ts2.focus_coordinates);
}

std::ostream& operator<<(std::ostream& out, const Configuration::TelemetrySettings& ts) {
	pt::ptree tree(ts.ToPTree());
	pt::write_json_no_quotes(out, tree, true);
	return out;
}
}//namespace ITMLib

// endregion ===========================================================================================================

// region ======================================== UI ENGINE SETTINGS ==================================================
Configuration::UIEngineSettings::UIEngineSettings() : number_of_frames_to_process_after_launch(0),
                                                      index_of_frame_to_start_at(0) {}

Configuration::UIEngineSettings::UIEngineSettings(int number_of_frames_to_process_after_launch,
                                                  int index_of_frame_to_start_at) :
		number_of_frames_to_process_after_launch(number_of_frames_to_process_after_launch),
		index_of_frame_to_start_at(index_of_frame_to_start_at) {}

Configuration::UIEngineSettings::UIEngineSettings(const po::variables_map& vm) :
		number_of_frames_to_process_after_launch(
				vm["process_N_frames"].empty() ? UIEngineSettings().number_of_frames_to_process_after_launch
				                               : vm["process_N_frames"].as<int>()),
		index_of_frame_to_start_at(vm["start_from_frame_ix"].empty() ? UIEngineSettings().index_of_frame_to_start_at
		                                                             : vm["start_from_frame_ix"].as<int>()) {}

Configuration::UIEngineSettings Configuration::UIEngineSettings::BuildFromPTree(const pt::ptree& tree) {
	boost::optional<int> nfpal = tree.get_optional<int>("number_of_frames_to_process_after_launch");
	boost::optional<int> ifs = tree.get_optional<int>("index_of_frame_to_start_at");
	UIEngineSettings default_ui_engine_settings;
	return {nfpal ? nfpal.get() : default_ui_engine_settings.number_of_frames_to_process_after_launch,
	        ifs ? ifs.get() : default_ui_engine_settings.index_of_frame_to_start_at};
}

pt::ptree Configuration::UIEngineSettings::ToPTree() const {
	pt::ptree tree;
	tree.add("number_of_frames_to_process_after_launch", number_of_frames_to_process_after_launch);
	tree.add("index_of_frame_to_start_at", index_of_frame_to_start_at);
	return tree;
}
namespace ITMLib {
bool operator==(const Configuration::UIEngineSettings& uiEngineSettings1,
                        const Configuration::UIEngineSettings& uiEngineSettings2) {
	return uiEngineSettings1.number_of_frames_to_process_after_launch ==
	       uiEngineSettings2.number_of_frames_to_process_after_launch &&
	       uiEngineSettings1.index_of_frame_to_start_at == uiEngineSettings2.index_of_frame_to_start_at;
}

std::ostream& operator<<(std::ostream& out, const Configuration::UIEngineSettings& uiEngineSettings) {
	pt::ptree tree(uiEngineSettings.ToPTree());
	pt::write_json_no_quotes(out, tree, true);
	return out;
}
} // end namespace ITMLib

// endregion ===========================================================================================================