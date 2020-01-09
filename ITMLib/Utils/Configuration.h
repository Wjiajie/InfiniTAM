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

#define SWAPPINGMODE_ENUM_DESCRIPTION SwappingMode, \
    (SWAPPINGMODE_DISABLED, "enabled"), \
    (SWAPPINGMODE_ENABLED, "disabled"), \
    (SWAPPINGMODE_DELETE, "delete")

DECLARE_SERIALIZABLE_ENUM(SWAPPINGMODE_ENUM_DESCRIPTION)

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

#ifndef COMPILE_WITHOUT_CUDA
#define DEFAULT_DEVICE MEMORYDEVICE_CUDA
#else
#define DEFAULT_DEVICE MEMORYDEVICE_CPU
#endif

struct TrackerConfigurationStringPresets{
	static const std::string default_ICP_tracker_configuration;
	static const std::string default_ICP_tracker_configuration_loop_closure;
	static const std::string default_depth_only_extended_tracker_configuration;
	static const std::string default_intensity_depth_extended_tracker_configuration;
	static const std::string default_color_only_tracker_configuration;
	static const std::string default_IMU_ICP_tracker_configuration;
	static const std::string default_IMU_extended_tracker_configuration;
	static const std::string default_surfel_tracker_configuration;
};

#define CONFIGURATION_STRUCT_DESCRIPTION Configuration, \
	(VoxelVolumeParameters, voxel_volume_parameters, VoxelVolumeParameters(), STRUCT),\
	(SurfelVolumeParameters, surfel_volume_parameters, SurfelVolumeParameters(), STRUCT),\
	(SlavchevaSurfaceTracker::Parameters, slavcheva_parameters, SlavchevaSurfaceTracker::Parameters(), STRUCT),\
	(SlavchevaSurfaceTracker::Switches, slavcheva_switches, SlavchevaSurfaceTracker::Switches(), STRUCT),\
	(TelemetrySettings, telemetry_settings, TelemetrySettings(), STRUCT),\
	(Paths, paths, Paths(), STRUCT),\
	(UIEngineSettings, ui_engine_settings, UIEngineSettings(), STRUCT),\
	(NonRigidTrackingParameters, non_rigid_tracking_parameters, NonRigidTrackingParameters(), STRUCT ),\
	(bool, skip_points, true, PRIMITIVE),\
	(bool, create_meshing_engine, true, PRIMITIVE),\
	(MemoryDeviceType, device_type, DEFAULT_DEVICE, ENUM),\
	(bool, use_approximate_raycast, true, PRIMITIVE),\
	(bool, use_threshold_filter, true, PRIMITIVE),\
	(bool, use_bilateral_filter, true, PRIMITIVE),\
	(FailureMode, behavior_on_failure, FAILUREMODE_IGNORE, ENUM),\
	(SwappingMode, swapping_mode, SWAPPINGMODE_DISABLED, ENUM),\
	(LibMode, library_mode, LIBMODE_DYNAMIC, ENUM),\
	(IndexingMethod, indexing_method, INDEX_HASH, ENUM),\
	(GradientFunctorType, surface_tracker_type, ITMLib::TRACKER_SLAVCHEVA_DIAGNOSTIC, ENUM),\
	(VerbosityLevel, verbosity_level, VERBOSITY_PER_FRAME, ENUM),\
	(std::string, tracker_configuration, TrackerConfigurationStringPresets::default_depth_only_extended_tracker_configuration, PRIMITIVE)


DECLARE_SERIALIZABLE_STRUCT(CONFIGURATION_STRUCT_DESCRIPTION);

Configuration& get();
void load_configuration_from_variable_map(const po::variables_map& vm);
void load_default();
void load_configuration_from_json_file(const std::string& path);
void save_configuration_to_json_file(const std::string& path);
void save_configuration_to_json_file(const std::string& path, const Configuration& configuration);

} // namespace configuration
} // namespace ITMLib