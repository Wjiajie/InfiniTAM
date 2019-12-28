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
#include "../SurfaceTrackers/WarpGradientFunctors/WarpGradientFunctor.h"

namespace po = boost::program_options;
namespace pt = boost::property_tree;

namespace ITMLib {
class Configuration {
public:
	static Configuration& get();

	static void load_default();
	static void load_configuration_from_variable_map(const po::variables_map& vm);
	static void load_configuration_from_json_file(const std::string& path);
	static void save_configuration_to_json_file(const std::string& path);
	void save_to_json_file(const std::string& path);

	static Configuration* from_property_tree(const pt::ptree& tree);

	~Configuration() = default;

	pt::ptree to_ptree() const;

	// region ============================================== NESTED ENUMS ==============================================

	enum FailureMode {
		FAILUREMODE_RELOCALIZE,
		FAILUREMODE_IGNORE,
		FAILUREMODE_STOP_INTEGRATION
	};

	enum SwappingMode {
		SWAPPINGMODE_DISABLED,
		SWAPPINGMODE_ENABLED,
		SWAPPINGMODE_DELETE
	};

	enum LibMode {
		LIBMODE_BASIC,
		LIBMODE_BASIC_SURFELS,
		LIBMODE_LOOPCLOSURE,
		LIBMODE_DYNAMIC
	};

	enum IndexingMethod {
		INDEX_HASH,
		INDEX_ARRAY
	};

	//endregion ========================================================================================================

	struct InputAndOutputSettings {
		/// Where to write any kind of output (intended to be used application-wise)
		const std::string output_path;
		std::string calibration_file_path;
		std::string openni_file_path = "";
		std::string rgb_video_file_path = "";
		std::string depth_video_file_path = "";
		std::string rgb_image_path_mask = "";
		std::string depth_image_path_mask = "";
		std::string mask_image_path_mask = "";
		std::string imu_input_path = "";

		explicit InputAndOutputSettings(const po::variables_map& vm);
		static InputAndOutputSettings BuildFromPTree(const pt::ptree& tree);
		InputAndOutputSettings();
		InputAndOutputSettings(std::string output_path,
		                       std::string calibration_file_path = "",
		                       std::string openni_file_path = "",
		                       std::string rgb_video_file_path = "",
		                       std::string depth_video_file_path = "",
		                       std::string rgb_image_path_mask = "",
		                       std::string depth_image_path_mask = "",
		                       std::string mask_image_path_mask = "",
		                       std::string imu_input_path = "");
		friend bool operator==(const InputAndOutputSettings& ts1, const InputAndOutputSettings& ts2);
		friend std::ostream& operator<<(std::ostream& out, const InputAndOutputSettings& ts);
		pt::ptree ToPTree() const;
	};

	struct TelemetrySettings {
		explicit TelemetrySettings(const po::variables_map& vm);
		static TelemetrySettings BuildFromPTree(const pt::ptree& tree);
		TelemetrySettings();
		TelemetrySettings(bool focus_coordinates_specified,
		                  Vector3i focus_coordinates = Vector3i(0));
		friend bool operator==(const TelemetrySettings& ts1, const TelemetrySettings& ts2);
		friend std::ostream& operator<<(std::ostream& out, const TelemetrySettings& ts);
		pt::ptree ToPTree() const;

		/// Whether telemetry / diagnostics / logging for a trouble spot is enabled
		bool focus_coordinates_specified = false;
		/// A trouble spot for additional telemetry / diagnostics / loggging
		Vector3i focus_coordinates;
	};

	Configuration();
	explicit Configuration(const po::variables_map& vm);
	Configuration(ITMSceneParameters scene_parameters,
	              ITMSurfelSceneParameters surfel_scene_parameters,
	              SlavchevaSurfaceTracker::Parameters slavcheva_parameters,
	              SlavchevaSurfaceTracker::Switches slavcheva_switches,
	              Configuration::TelemetrySettings telemetry_settings,
	              Configuration::InputAndOutputSettings input_and_output_settings,
	              bool skip_points,
	              bool create_meshing_engine,
	              MemoryDeviceType device_type,
	              bool use_approximate_raycast,
	              bool use_threshold_filter,
	              bool use_bilateral_filter,
	              Configuration::FailureMode behavior_on_failure,
	              Configuration::SwappingMode swapping_mode,
	              Configuration::LibMode library_mode,
	              Configuration::IndexingMethod indexing_method,
	              GradientFunctorType surface_tracker_type,
	              std::string tracker_configuration,
	              unsigned int max_iteration_threshold,
	              float max_update_length_threshold);

	friend bool operator==(const Configuration& c1, const Configuration& c2);
	friend std::ostream& operator<<(std::ostream& out, const Configuration& c);

	//TODO: group these settings into structs as appropriate, re-organize the structs by class usage, i.e.
	// those settings exclusively used by a specific class should be in a public inner struct of this class

	/// Scene-specific parameters such as voxel size
	const ITMSceneParameters scene_parameters;
	const ITMSurfelSceneParameters surfel_scene_parameters;
	/// Surface tracking energy parameters
	const SlavchevaSurfaceTracker::Parameters slavcheva_parameters;
	/// Surface tracking energy switches
	const SlavchevaSurfaceTracker::Switches slavcheva_switches;
	/// Telemetry / diagnostics / logging settings
	TelemetrySettings telemetry_settings;
	/// Input / output paths
	InputAndOutputSettings input_and_output_settings;
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
	/*Note: library_mode declaration has to precede that of tracker_configuration,
	 since the latter uses the former for initialization in lists*/
	/// tracker configuration string
	/// (see Tracker documentation & code for details, code for this class for default "examples")
	std::string tracker_configuration;
	//TODO: group and refactor->rename the below two
	/// Surface tracking optimization termination parameters - iteration threshold
	const unsigned int max_iteration_threshold;
	/// Surface tracking optimization termination parameters - threshold on minimal warp update (in meters)
	const float max_update_length_threshold;


private:

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

bool operator==(const Configuration::TelemetrySettings& ts1, const Configuration::TelemetrySettings& ts2);
std::ostream& operator<<(std::ostream& out, const Configuration::TelemetrySettings& ts);
bool operator==(const Configuration& c1, const Configuration& c2);
std::ostream& operator<<(std::ostream& out, const Configuration& c);
}
