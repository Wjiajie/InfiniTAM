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

using namespace ITMLib::configuration;

// *** serializable enum definitions ***

DEFINE_SERIALIZABLE_ENUM(VERBOSITY_LEVEL_ENUM_DESCRIPTION)
DEFINE_SERIALIZABLE_ENUM(FAILUREMODE_ENUM_DESCRIPTION)
DEFINE_SERIALIZABLE_ENUM(LIBMODE_ENUM_DESCRIPTION)
DEFINE_SERIALIZABLE_ENUM(INDEXING_METHOD_DESCRIPTION)

// *** serializable struct definitions ***
DEFINE_SERIALIZABLE_STRUCT(PATHS_STRUCT_DESCRIPTION)
DEFINE_SERIALIZABLE_STRUCT(TELEMETRY_SETTINGS_STRUCT_DESCRIPTION)
DEFINE_SERIALIZABLE_STRUCT(UI_ENGINE_SETTINGS_STRUCT_DESCRIPTION)


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
                             SurfelVolumeParameters surfel_volume_parameters,
                             SlavchevaSurfaceTracker::Parameters slavcheva_parameters,
                             SlavchevaSurfaceTracker::Switches slavcheva_switches,
                             Configuration::TelemetrySettings telemetry_settings,
                             Configuration::InputAndOutputSettings input_and_output_settings,
                             Configuration::Paths input_and_output_settings_paths,
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
	boost::optional<SurfelVolumeParameters> surfel_volume_parameters =
			as_optional_parsable<SurfelVolumeParameters>(tree, "surfel_volume_parameters");
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
	boost::optional<Paths> input_and_output_settings_paths =
			as_optional_parsable_config_path<Paths>(tree, "input_and_output_settings.paths", path);

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