//  ================================================================
//  Created by Gregory Kramida on 11/12/19.
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

// Inspired in part by InfiniTAM/ITMLib/Utils/ITMLibSettings of the original InfiniTAM repository, Oxford University

#pragma once

//stdlib
#include <cfloat>
#include <memory>

//boost
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>

//local
#include "VoxelVolumeParameters.h"
#include "SurfelVolumeParameters.h"
#include "../../ORUtils/MemoryDeviceType.h"
#include "ITMMath.h"
#include "../SurfaceTrackers/Interface/SurfaceTrackerInterface.h"
#include "../SurfaceTrackers/WarpGradientFunctors/WarpGradientFunctor.h"
#include "../Engines/Main/NonRigidTrackingParameters.h"

namespace po = boost::program_options;
namespace pt = boost::property_tree;


namespace ITMLib {
namespace configuration {
// region ============================================== SERIALIZABLE ENUMS ============================================
#define VERBOSITY_LEVEL_ENUM_DESCRIPTION VerbosityLevel, \
    (VERBOSITY_SILENT, "silent", "SILENT", "VERBOSITY_SILENT"), \
    (VERBOSITY_TOP_LEVEL, "top_level", "TOP_LEVEL", "Top-level operations", "VERBOSITY_TOP_LEVEL", "top-level", "top-level operations"), \
    (VERBOSITY_PER_FRAME, "per_frame", "PER_FRAME", "Per-frame operations", "VERBOSITY_PER_FRAME", "per-frame", "per-frame operations"), \
    (VERBOSITY_PER_ITERATION, "per_iteration", "PER_ITERATION", "Per-iteration operations", "VERBOSITY_PER_ITERATION", "per-iteration", "per-iteration operations"), \
    (VERBOSITY_FOCUS_SPOTS, "focus_spots", "FOCUS_SPOTS", "focus_coordinates", "Interesting details", "trouble spots")

DECLARE_SERIALIZABLE_ENUM(VERBOSITY_LEVEL_ENUM_DESCRIPTION)

#define FAILUREMODE_ENUM_DESCRIPTION FailureMode, \
    (FAILUREMODE_RELOCALIZE, "relocalize"), \
    (FAILUREMODE_IGNORE, "ignore"), \
    (FAILUREMODE_STOP_INTEGRATION, "stop_integration")

DECLARE_SERIALIZABLE_ENUM(FAILUREMODE_ENUM_DESCRIPTION)

#define LIBMODE_ENUM_DESCRIPTION LibMode, \
    (LIBMODE_BASIC, "basic"), \
    (LIBMODE_BASIC_SURFELS, "surfels"), \
    (LIBMODE_LOOPCLOSURE, "loop_closure"), \
    (LIBMODE_DYNAMIC, "dynamic")

DECLARE_SERIALIZABLE_ENUM(LIBMODE_ENUM_DESCRIPTION)

#define INDEXING_METHOD_DESCRIPTION IndexingMethod, \
    (INDEX_HASH, "hash", "HASH"), \
    (INDEX_ARRAY, "array", "ARRAY")

DECLARE_SERIALIZABLE_ENUM(INDEXING_METHOD_DESCRIPTION)
//endregion ========================================================================================================

// region ======================================== SERIALIZABLE STRUCTS ============================================
#define PATHS_STRUCT_DESCRIPTION Paths,\
	(std::string, output_path, "", PATH),\
	(std::string, calibration_file_path, "", PATH),\
	(std::string, openni_file_path, "", PATH),\
	(std::string, rgb_video_file_path, "", PATH),\
	(std::string, depth_video_file_path, "", PATH),\
	(std::string, rgb_image_path_mask, "", PATH),\
	(std::string, depth_image_path_mask, "", PATH),\
	(std::string, mask_image_path_mask, "", PATH),\
	(std::string, imu_input_path, "", PATH)

DECLARE_SERIALIZABLE_STRUCT(PATHS_STRUCT_DESCRIPTION);

///For focus_coordinates to be used, VerbosityLevel must be set to VERBOSITY_FOCUS_SPOTS or above
#define TELEMETRY_SETTINGS_STRUCT_DESCRIPTION TelemetrySettings, \
	(Vector3i, focus_coordinates, Vector3i(0), VECTOR), \
	(bool, record_reconstruction_video, false, PRIMITIVE), \
	(bool, save_benchmarks_to_disk, false, PRIMITIVE)

DECLARE_SERIALIZABLE_STRUCT(TELEMETRY_SETTINGS_STRUCT_DESCRIPTION);

#define UI_ENGINE_SETTINGS_STRUCT_DESCRIPTION UIEngineSettings, \
	(int, number_of_frames_to_process_after_launch, 0, PRIMITIVE), \
	(int, index_of_frame_to_start_at, 0, PRIMITIVE)

DECLARE_SERIALIZABLE_STRUCT(UI_ENGINE_SETTINGS_STRUCT_DESCRIPTION);

#define CONFIGURATION_STRUCT_DESCRIPTION Configuration, \
	(VoxelVolumeParameters, voxel_volume_parameters, VoxelVolumeParameters(), STRUCT),\
	(SurfelVolumeParameters, surfel_volume_parameters, SurfelVolumeParameters(), STRUCT),\
	()

DECLARE_SERIALIZABLE_STRUCT(CONFIGURATION_STRUCT_DESCRIPTION);


/// Scene-specific parameters such as voxel size
const VoxelVolumeParameters voxel_volume_parameters;
const SurfelVolumeParameters surfel_volume_parameters;
/// Surface tracking energy parameters
const SlavchevaSurfaceTracker::Parameters slavcheva_parameters;
/// Surface tracking energy switches
const SlavchevaSurfaceTracker::Switches slavcheva_switches;
/// Telemetry / diagnostics / logging settings
TelemetrySettings telemetry_settings;
/// Input / output paths
const InputAndOutputSettings input_and_output_settings;
const Paths input_and_output_settings_paths;
const UIEngineSettings ui_engine_settings;
const NonRigidTrackingParameters non_rigid_tracking_parameters;
/// For ITMColorTracker: skips every other point when using the colour renderer for creating a point cloud
const bool skip_points;
/// create all the things required for marching cubes and mesh extraction (uses lots of additional memory)
const bool create_meshing_engine;
/// Select the type of device to use
const MemoryDeviceType device_type;
/// enables or disables approximate raycast
const bool use_approximate_raycast;
/// enable or disable threshold depth filtering
const bool use_threshold_filter;
/// enable or disable bilateral depth filtering
const bool use_bilateral_filter;
/// what to do on tracker failure: ignore, relocalize or stop integration - not supported in loop closure version
const FailureMode behavior_on_failure;
/// how swapping works: disabled, fully enabled (still with dragons) and delete what's not visible - not supported in loop closure version
const SwappingMode swapping_mode;
/// switch between various library modes - basic, with loop closure, etc.
const LibMode library_mode;
/// switch between different types of indexing for the spatial data structures
const IndexingMethod indexing_method;
/// switch between different versions of the tracker
const GradientFunctorType surface_tracker_type;
/// control how much diagnostic text is output by the program
const VerbosityLevel verbosity_level;
/*Note: library_mode declaration has to precede that of tracker_configuration,
 since the latter uses the former for initialization in lists*/
/// tracker configuration string
/// (see Tracker documentation & code for details, code for this class for default "examples")
std::string tracker_configuration;

private:

static std::unique_ptr<Configuration> instance;
struct TrackerConfigurationStringDefaults{
static const std::string default_ICP_tracker_configuration;
static const std::string default_ICP_tracker_configuration_loop_closure;
static const std::string default_depth_only_extended_tracker_configuration;
static const std::string default_intensity_depth_extended_tracker_configuration;
static const std::string default_color_only_tracker_configuration;
static const std::string default_IMU_ICP_tracker_configuration;
static const std::string default_IMU_extended_tracker_configuration;
static const std::string default_surfel_tracker_configuration;
};

bool operator==(const Configuration::TelemetrySettings& ts1, const Configuration::TelemetrySettings& ts2);
std::ostream& operator<<(std::ostream& out, const Configuration::TelemetrySettings& ts);
bool operator==(const Configuration& c1, const Configuration& c2);
std::ostream& operator<<(std::ostream& out, const Configuration& c);
} // namespace configuration
} // namespace ITMLib