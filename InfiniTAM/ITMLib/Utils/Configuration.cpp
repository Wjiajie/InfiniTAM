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
#include <boost/property_tree/json_parser.hpp>

//local
#include "Configuration.h"
#include "json_utils.h"

using namespace ITMLib;


// region ============================= Boost variables_map processing subroutines =====================================

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
static TEnum enum_value_from_string(const std::string& string);

template<typename TEnum>
static std::string enum_value_to_string(const TEnum& enum_value);

template<>
MemoryDeviceType enum_value_from_string<MemoryDeviceType>(const std::string& string) {
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
std::string enum_value_to_string<MemoryDeviceType>(const MemoryDeviceType& enum_value) {
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
Configuration::SwappingMode enum_value_from_string<Configuration::SwappingMode>(const std::string& string) {
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
std::string enum_value_to_string<Configuration::SwappingMode>(const Configuration::SwappingMode& enum_value) {
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
Configuration::FailureMode enum_value_from_string<Configuration::FailureMode>(const std::string& string) {
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
std::string enum_value_to_string<Configuration::FailureMode>(const Configuration::FailureMode& enum_value) {
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
Configuration::LibMode enum_value_from_string<Configuration::LibMode>(const std::string& string) {
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
std::string enum_value_to_string<Configuration::LibMode>(const Configuration::LibMode& enum_value) {
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
Configuration::IndexingMethod enum_value_from_string<Configuration::IndexingMethod>(const std::string& string) {
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
std::string enum_value_to_string<Configuration::IndexingMethod>(const Configuration::IndexingMethod& enum_value) {
	switch (enum_value) {
		case Configuration::IndexingMethod::INDEX_ARRAY:
			return "array";
		case Configuration::IndexingMethod::INDEX_HASH:
			return "hash";
	}
}

template<>
SlavchevaSurfaceTracker::ConfigurationMode enum_value_from_string<SlavchevaSurfaceTracker::ConfigurationMode>(
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
std::string enum_value_to_string<SlavchevaSurfaceTracker::ConfigurationMode>(
		const SlavchevaSurfaceTracker::ConfigurationMode& enum_value) {
	switch (enum_value) {
		case SlavchevaSurfaceTracker::ConfigurationMode::KILLING_FUSION:
			return "killing";
		case SlavchevaSurfaceTracker::ConfigurationMode::SOBOLEV_FUSION:
			return "sobolev";
	}
}

static MemoryDeviceType memory_device_type_from_variable_map(const po::variables_map& vm, const std::string& argument) {
	return enum_value_from_string<MemoryDeviceType>(vm[argument].as<std::string>());
}

static Configuration::SwappingMode
swapping_mode_from_variable_map(const po::variables_map& vm, const std::string& argument) {
	return enum_value_from_string<Configuration::SwappingMode>(vm[argument].as<std::string>());
}

static Configuration::FailureMode
failure_mode_from_variable_map(const po::variables_map& vm, const std::string& argument) {
	return enum_value_from_string<Configuration::FailureMode>(vm[argument].as<std::string>());
}

static Configuration::LibMode lib_mode_from_variable_map(const po::variables_map& vm, const std::string& argument) {
	return enum_value_from_string<Configuration::LibMode>(vm[argument].as<std::string>());
}

static Configuration::IndexingMethod indexing_method_from_variable_map(const po::variables_map& vm,
                                                                       const std::string& argument) {
	return enum_value_from_string<Configuration::IndexingMethod>(vm[argument].as<std::string>());
}

template<typename TEnum>
static boost::optional<TEnum> optional_enum_value_from_ptree(const pt::ptree& ptree, const pt::ptree::key_type& key) {
	auto child = ptree.get_child_optional(key);
	if (child) {
		return boost::optional<TEnum>(enum_value_from_string<TEnum>(ptree.get<std::string>(key)));
	} else {
		return boost::optional<TEnum>{};
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
		swapping_mode(vm["swapping"].empty() ? Configuration().swapping_mode :
		              swapping_mode_from_variable_map(vm, "swapping")),
		library_mode(vm["mode"].empty() ? Configuration().library_mode :
		             lib_mode_from_variable_map(vm, "mode")),
		indexing_method(vm["index"].empty() ? Configuration().indexing_method :
		                indexing_method_from_variable_map(vm, "index")),
		tracker_configuration(
				vm["tracker"].empty() ? (library_mode == LIBMODE_BASIC_SURFELS ? default_surfel_tracker_configuration :
				                         default_depth_only_extended_tracker_configuration)
				                      : vm["tracker"].as<std::string>()),
		max_iteration_threshold(vm["max_iterations"].empty() ?
		                        Configuration().max_iteration_threshold
		                                                     :
		                        vm["max_iterations"].as<unsigned int>()),
		max_update_length_threshold(vm["vector_update_threshold"].empty() ?
		                            Configuration().max_update_length_threshold
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

Configuration::Configuration()
		:   //mu(m), maxW, voxel size(m), clipping min, clipping max, stopIntegratingAtMaxW
		scene_parameters(0.04f, 100, 0.004f, 0.2f, 3.0f, false),//corresponds to KillingFusion article //_DEBUG
		//scene_parameters(0.02f, 100, 0.005f, 0.2f, 3.0f, false),//standard InfiniTAM values
		surfel_scene_parameters(0.5f, 0.6f, static_cast<float>(20 * M_PI / 180), 0.01f, 0.004f, 3.5f, 25.0f, 4, 1.0f,
		                        5.0f, 20, 10000000, true, true),
		slavcheva_parameters(SlavchevaSurfaceTracker::ConfigurationMode::SOBOLEV_FUSION),
		slavcheva_switches(SlavchevaSurfaceTracker::ConfigurationMode::SOBOLEV_FUSION),
		telemetry_settings(),
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
		tracker_configuration(library_mode == LIBMODE_BASIC_SURFELS ? default_surfel_tracker_configuration :
		                      default_depth_only_extended_tracker_configuration),
		max_iteration_threshold(200),
		max_update_length_threshold(0.0001f) {
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


void Configuration::load_from_variable_map(const po::variables_map& vm) {
	instance.reset(new Configuration(vm));
}

void Configuration::load_default() {
	instance.reset(new Configuration);
}

Configuration& Configuration::get() {
	return *instance;
}

void Configuration::load_from_json_file(const std::string& path) {
	pt::ptree tree;
	pt::read_json(path, tree);
	instance.reset(from_property_tree(tree));
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


Configuration* Configuration::from_property_tree(const pt::ptree& tree) {
	Configuration default_configuration;

	boost::optional<ITMSceneParameters> scene_parameters =
			as_optional_parsable<ITMSceneParameters>(tree, "scene_parameters");
	boost::optional<ITMSurfelSceneParameters> surfel_scene_parameters =
			as_optional_parsable<ITMSurfelSceneParameters>(tree, "surfel_scene_parameters");
	boost::optional<SlavchevaSurfaceTracker::ConfigurationMode> mode_opt =
			optional_enum_value_from_ptree<SlavchevaSurfaceTracker::ConfigurationMode>(tree, "slavcheva.preset");
	SlavchevaSurfaceTracker::ConfigurationMode mode = mode_opt ? mode_opt.get()
	                                                           : SlavchevaSurfaceTracker::SOBOLEV_FUSION;
	boost::optional<SlavchevaSurfaceTracker::Parameters> slavcheva_parameters =
			as_optional_parsable_slavcheva<SlavchevaSurfaceTracker::Parameters>(tree, "slavcheva.parameters", mode);
	boost::optional<SlavchevaSurfaceTracker::Switches> slavcheva_switches =
			as_optional_parsable_slavcheva<SlavchevaSurfaceTracker::Switches>(tree, "slavcheva.switches", mode);
	boost::optional<TelemetrySettings> telemetry_settings = as_optional_parsable<TelemetrySettings>(tree,
	                                                                                                "telemetry_settings");
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
	boost::optional<std::string> tracker_configuration = tree.get_optional<std::string>("tracker");
	boost::optional<unsigned int> max_iteration_threshold =
			tree.get_optional<unsigned int>("surface_tracking.max_iterations");
	boost::optional<float> max_update_length_threshold =
			tree.get_optional<float>("surface_tracking.vector_update_threshold");

	Configuration default_config;

	return new Configuration(
			scene_parameters ? scene_parameters.get() : default_config.scene_parameters,
			surfel_scene_parameters ? surfel_scene_parameters.get() : default_config.surfel_scene_parameters,
			slavcheva_parameters ? slavcheva_parameters.get() : default_config.slavcheva_parameters,
			slavcheva_switches ? slavcheva_switches.get() : default_config.slavcheva_switches,
			telemetry_settings ? telemetry_settings.get() : default_config.telemetry_settings,
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
			tracker_configuration ? tracker_configuration.get() : default_config.tracker_configuration,
			max_iteration_threshold ? max_iteration_threshold.get() : default_config.max_iteration_threshold,
			max_update_length_threshold ? max_update_length_threshold.get() : default_config.max_update_length_threshold
	);
}

Configuration::Configuration(
		ITMSceneParameters scene_parameters, ITMSurfelSceneParameters surfel_scene_parameters,
		SlavchevaSurfaceTracker::Parameters slavcheva_parameters, SlavchevaSurfaceTracker::Switches slavcheva_switches,
		Configuration::TelemetrySettings telemetry_settings, bool skip_points, bool create_meshing_engine,
		MemoryDeviceType device_type, bool use_approximate_raycast, bool use_threshold_filter,
		bool use_bilateral_filter, Configuration::FailureMode behavior_on_failure,
		Configuration::SwappingMode swapping_mode,
		Configuration::LibMode library_mode, Configuration::IndexingMethod indexing_method,
		std::string tracker_configuration,
		unsigned int max_iteration_threshold,
		float max_update_length_threshold) :

		scene_parameters(scene_parameters),
		surfel_scene_parameters(surfel_scene_parameters),
		slavcheva_parameters(slavcheva_parameters),
		slavcheva_switches(slavcheva_switches),
		telemetry_settings(std::move(telemetry_settings)),
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
		tracker_configuration(std::move(tracker_configuration)),
		max_iteration_threshold(max_iteration_threshold),
		max_update_length_threshold(
				max_update_length_threshold) {}

namespace ITMLib {
bool operator==(const Configuration& c1, const Configuration& c2) {
	return c1.scene_parameters == c2.scene_parameters &&
	       c1.surfel_scene_parameters == c2.surfel_scene_parameters &&
	       c1.slavcheva_parameters == c2.slavcheva_parameters &&
	       c1.slavcheva_switches == c2.slavcheva_switches &&
	       c1.telemetry_settings == c2.telemetry_settings &&
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
	       c1.tracker_configuration == c2.tracker_configuration &&
	       c1.max_iteration_threshold == c2.max_iteration_threshold &&
	       c1.max_update_length_threshold == c2.max_update_length_threshold;
}

std::ostream& operator<<(std::ostream& out, const Configuration& c) {
	pt::ptree tree(c.to_ptree());
	pt::write_json_no_quotes(out, tree, true);
}
}//namespace ITMLib

pt::ptree Configuration::to_ptree() const {
	pt::ptree tree;
	tree.add_child("scene_parameters", this->scene_parameters.ToPTree());
	tree.add_child("surfel_scene_parameters", this->surfel_scene_parameters.ToPTree());
	tree.add("slavcheva.preset", enum_value_to_string(SlavchevaSurfaceTracker::ConfigurationMode::SOBOLEV_FUSION));
	tree.add_child("slavcheva.parameters", this->slavcheva_parameters.ToPTree());
	tree.add_child("slavcheva.switches", slavcheva_switches.ToPTree());
	tree.add_child("telemetry_settings", this->telemetry_settings.ToPTree());
	tree.add("skip_points", skip_points);
	tree.add("disable_meshing", !create_meshing_engine);
	tree.add("device", enum_value_to_string(this->device_type));
	tree.add("use_approximate_raycast", this->use_approximate_raycast);
	tree.add("use_threshold_filter", this->use_threshold_filter);
	tree.add("use_bilateral_filter", this->use_bilateral_filter);
	tree.add("failure_mode", enum_value_to_string(this->behavior_on_failure));
	tree.add("swapping", enum_value_to_string(this->swapping_mode));
	tree.add("mode", enum_value_to_string(this->library_mode));
	tree.add("index", enum_value_to_string(this->indexing_method));
	tree.add("tracker", this->tracker_configuration);
	tree.add("surface_tracking.max_iterations", this->max_iteration_threshold);
	tree.add("surface_tracking.vector_update_threshold", this->max_update_length_threshold);
	return tree;
}

void Configuration::save_configuration_to_json_file(const std::string& path) {
	pt::write_json_no_quotes(path, instance->to_ptree(), true);
}


void Configuration::save_to_json_file(const std::string& path) {
	pt::write_json_no_quotes(path, this->to_ptree(), true);
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

template<typename T>
static
std::vector<T> as_vector(pt::ptree const& pt, pt::ptree::key_type const& key) {
	std::vector<T> r;
	for (auto& item : pt.get_child(key))
		r.push_back(item.second.get_value<T>());
	return r;
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

Configuration::TelemetrySettings Configuration::TelemetrySettings::BuildFromPTree(const pt::ptree& tree) {
	boost::optional<std::string> output_path_opt = tree.get_optional<std::string>("output");
	boost::optional<std::vector<int>> focus_coords_opt = as_optional_vector<int>(tree, "focus_coordinates");

	TelemetrySettings default_ts;

	return {output_path_opt ? output_path_opt.get() : default_ts.output_path,
	        (bool) focus_coords_opt,
	        focus_coords_opt ? vector3i_from_std_vector(focus_coords_opt.get()) : default_ts.focus_coordinates};
}

Configuration::TelemetrySettings::TelemetrySettings(std::string output_path, bool focus_coordinates_specified,
                                                    Vector3i focus_coordinates) :
		output_path(std::move(output_path)),
		focus_coordinates_specified(focus_coordinates_specified),
		focus_coordinates(focus_coordinates) {}

pt::ptree Configuration::TelemetrySettings::ToPTree() const {
	pt::ptree tree;
	tree.add("output", this->output_path);
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
	return ts1.output_path == ts2.output_path &&
	       ts1.focus_coordinates_specified == ts2.focus_coordinates_specified &&
	       (!ts1.focus_coordinates_specified || ts1.focus_coordinates == ts2.focus_coordinates);
}

std::ostream& operator<<(std::ostream& out, const Configuration::TelemetrySettings& ts) {
	pt::ptree tree(ts.ToPTree());
	pt::write_json_no_quotes(out, tree, true);
}

}//namespace ITMLib
