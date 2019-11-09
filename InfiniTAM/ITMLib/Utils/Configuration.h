// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <cfloat>
#include "ITMSceneParameters.h"
#include "ITMSurfelSceneParameters.h"
#include "../../ORUtils/MemoryDeviceType.h"
#include "ITMMath.h"
#include "../SurfaceTrackers/Interface/SurfaceTrackerInterface.h"

//boost
#include <boost/program_options.hpp>
namespace po = boost::program_options;

namespace ITMLib
{
	class Configuration
	{
	public:

		static Configuration& Instance(){
			static Configuration instance;
			return instance;
		}
		explicit Configuration(const po::variables_map& vm);

		~Configuration() = default;

		// Suppress the default copy constructor and assignment operator
		Configuration(const Configuration &) = delete;
		Configuration& operator=(const Configuration &) = delete;

		typedef enum
		{
			FAILUREMODE_RELOCALISE,
			FAILUREMODE_IGNORE,
			FAILUREMODE_STOP_INTEGRATION
		} FailureMode;
        
		typedef enum
		{
			SWAPPINGMODE_DISABLED,
			SWAPPINGMODE_ENABLED,
			SWAPPINGMODE_DELETE
		} SwappingMode;

		typedef enum
		{
			LIBMODE_BASIC,
			LIBMODE_BASIC_SURFELS,
			LIBMODE_LOOPCLOSURE,
			LIBMODE_DYNAMIC
		}LibMode;



		/// Select the type of device to use
		MemoryDeviceType deviceType;

		const bool useApproximateRaycast;

		const bool useThresholdFilter;
		const bool useBilateralFilter;

		/// For ITMColorTracker: skip every other point in energy function evaluation.
		const bool skipPoints;

		const bool createMeshingEngine;
        
		const FailureMode behaviourOnFailure;
		const SwappingMode swappingMode;
		const LibMode libMode;

		const char *trackerConfig;

		/// Further, scene-specific parameters such as voxel size
		const ITMSceneParameters scene_parameters;
		const ITMSurfelSceneParameters surfel_scene_parameters;

		MemoryDeviceType GetMemoryType() const;

		/// Dynamic Fusion parameters
		struct AnalysisSettings{
			explicit AnalysisSettings(const po::variables_map& vm);
			AnalysisSettings();
			const std::string output_path;
			const bool focus_coordinates_specified = false;
			const Vector3i focus_coordinates;
		};

		const bool restrictZtrackingForDebugging;// = false;
		bool FocusCoordinatesAreSpecified() const;// = false; // CLI flag made in InfiniTAM_bpo

		const AnalysisSettings analysisSettings;

		//*** Scene Tracking ITMSceneMotionOptimizationSwitches ***

		const SlavchevaSurfaceTracker::Parameters slavcheva_parameters;
		const SlavchevaSurfaceTracker::Switches slavcheva_switches;

		//*** Scene Tracking ITMSceneMotionOptimizationParameters ***
		//** optimization loop
		const unsigned int surface_tracking_max_optimization_iteration_count;
		const float surface_tracking_optimization_vector_update_threshold_meters;
		//** gradient calculation

	private:
		Configuration();
	};
}
