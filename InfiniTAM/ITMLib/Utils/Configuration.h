// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

//stdlib
#include <cfloat>
#include <memory>

//boost
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>

//local
#include "ITMSceneParameters.h"
#include "ITMSurfelSceneParameters.h"
#include "../../ORUtils/MemoryDeviceType.h"
#include "ITMMath.h"
#include "../SurfaceTrackers/Interface/SurfaceTrackerInterface.h"

namespace po = boost::program_options;
namespace pt = boost::property_tree;

namespace ITMLib {
class Configuration {
public:

	static Configuration& get();

	static void load_default();
	static void load_from_variable_map(const po::variables_map& vm);

	~Configuration() = default;

	// Suppress the default copy constructor and assignment operator
	Configuration(const Configuration&) = delete;
	Configuration& operator=(const Configuration&) = delete;

	typedef enum {
		FAILUREMODE_RELOCALIZE,
		FAILUREMODE_IGNORE,
		FAILUREMODE_STOP_INTEGRATION
	} FailureMode;

	typedef enum {
		SWAPPINGMODE_DISABLED,
		SWAPPINGMODE_ENABLED,
		SWAPPINGMODE_DELETE
	} SwappingMode;

	typedef enum {
		LIBMODE_BASIC,
		LIBMODE_BASIC_SURFELS,
		LIBMODE_LOOPCLOSURE,
		LIBMODE_DYNAMIC
	} LibMode;


	//TODO: group these settings into structs as appropriate, re-organize the structs by class usage, i.e.
	// those settings exclusively used by a specific class should be in a public inner struct of this class

	/// Select the type of device to use
	MemoryDeviceType device_type;
	/// enables or disables approximate raycast
	const bool use_approximate_raycast;
	/// enable or disable threshold depth filtering
	const bool use_threshold_filter;
	/// enable or disable bilateral depth filtering
	const bool use_bilateral_filter;
	/// For ITMColorTracker: skips every other point when using the colour renderer for creating a point cloud
	const bool skip_points;
	/// create all the things required for marching cubes and mesh extraction (uses lots of additional memory)
	const bool create_meshing_engine;
	/// what to do on tracker failure: ignore, relocalize or stop integration - not supported in loop closure version
	const FailureMode behavior_on_failure;
	/// how swapping works: disabled, fully enabled (still with dragons) and delete what's not visible - not supported in loop closure version
	const SwappingMode swapping_mode;
	/// switch between various library modes - basic, with loop closure, etc.
	const LibMode library_mode;
	/*Note: library_mode declaration has to precede that of tracker_configuration,
	 since the latter uses the former for initialization in lists*/
	/// tracker configuration string
	/// (see Tracker documentation & code for details, code for this class for default "examples")
	std::string tracker_configuration;

	/// Further, scene-specific parameters such as voxel size
	const ITMSceneParameters scene_parameters;
	const ITMSurfelSceneParameters surfel_scene_parameters;

	MemoryDeviceType GetMemoryType() const;

	/// Dynamic Fusion parameters
	struct TelemetrySettings {
		explicit TelemetrySettings(const po::variables_map& vm);
		TelemetrySettings();
		/// Where to write any kind of output (intended to be used application-wise)
		const std::string output_path = "./State/";
		/// Whether telemetry / diagnostics / logging for a trouble spot is enabled
		const bool focus_coordinates_specified = false;
		/// A trouble spot for additional telemetry / diagnostics / loggging
		const Vector3i focus_coordinates;
	};

	bool FocusCoordinatesAreSpecified() const;// = false; // CLI flag made in InfiniTAM_bpo

	const TelemetrySettings telemetry_settings;

	/// Surface tracking energy parameters
	const SlavchevaSurfaceTracker::Parameters slavcheva_parameters;
	/// Surface tracking energy switches
	const SlavchevaSurfaceTracker::Switches slavcheva_switches;

	/// Surface tracking optimization termination parameters - iteration threshold
	const unsigned int surface_tracking_max_optimization_iteration_count;
	/// Surface tracking optimization termination parameters - threshold on minimal warp update (in meters)
	const float surface_tracking_optimization_vector_update_threshold_meters;


private:
	Configuration() noexcept;
	explicit Configuration(const po::variables_map& vm);
//TODO
//	explicit Configuration(const pt::ptree& tree);
	static std::unique_ptr<Configuration> instance;
	static const std::string default_ICP_tracker_configuration;
	static const std::string default_ICP_tracker_configuration_loop_closure;
	static const std::string default_depth_only_extended_tracker_configuration;
	static const std::string default_intensity_depth_extended_tracker_configuration;
	static const std::string default_color_only_tracker_configuration;
	static const std::string default_IMU_ICP_tracker_configuration;
	static const std::string default_IMU_extended_tracker_configuration;
	static const std::string default_surfel_tracker_configuration;
};
}
