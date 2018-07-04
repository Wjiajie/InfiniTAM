// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <cfloat>
#include "ITMSceneParams.h"
#include "ITMSurfelSceneParams.h"
#include "../../ORUtils/MemoryDeviceType.h"
#include "ITMMath.h"

namespace ITMLib
{
	class ITMLibSettings
	{
	public:
		//TODO: settings that are not intended to be changed during runtime should be set to cost and initialized
		// right away in the constructor. Settings that are intended to change should be protected by class access
		// modifiers / getters / setters as to grant access only to the objects that should be able to change them.
		/// The device used to run the DeviceAgnostic code
		typedef enum {
			DEVICE_CPU,
			DEVICE_CUDA,
			DEVICE_METAL
		} DeviceType;

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
		DeviceType deviceType;

		bool useApproximateRaycast;

		bool useThresholdFilter;
		bool useBilateralFilter;

		/// For ITMColorTracker: skip every other point in energy function evaluation.
		bool skipPoints;

		bool createMeshingEngine;
        
		FailureMode behaviourOnFailure;
		SwappingMode swappingMode;
		LibMode libMode;

		const char *trackerConfig;

		/// Further, scene-specific parameters such as voxel size
		ITMSceneParams sceneParams;
		ITMSurfelSceneParams surfelSceneParams;

		ITMLibSettings();
		virtual ~ITMLibSettings() = default;

		// Suppress the default copy constructor and assignment operator
		ITMLibSettings(const ITMLibSettings&) = delete;
		ITMLibSettings& operator=(const ITMLibSettings&) = delete;

		MemoryDeviceType GetMemoryType() const;

		/// Dynamic Fusion parameters

		struct AnalysisSettings{
			std::string outputPath;
			bool focusCoordinatesSpecified = false;
			Vector3i focusCoordinates;
		};

		bool restrictZtrackingForDebugging;// = false;
		bool simpleSceneExperimentModeEnabled;// = false;
		bool FocusCoordinatesAreSpecified() const;// = false; // CLI flag made in InfiniTAM_bpo

		Vector3i GetFocusCoordinates() const;
		void SetFocusCoordinates(const Vector3i& coordiantes);

		AnalysisSettings analysisSettings;

		//*** Scene Tracking Switches ***
		bool enableDataTerm;
		bool enableLevelSetTerm;
		bool enableSmoothingTerm;
		bool enableKillingTerm;
		bool enableGradientSmoothing;
		bool usePreviousUpdateVectorsForSmoothing;

		//*** Scene Tracking Parameters ***
		//** optimization loop
		unsigned int sceneTrackingMaxOptimizationIterationCount;
		float sceneTrackingOptimizationVectorUpdateThresholdMeters;
		//** gradient calculation
		float sceneTrackingGradientDescentLearningRate;
		float sceneTrackingRigidityEnforcementFactor;
		float sceneTrackingWeightDataTerm;
		float sceneTrackingWeightSmoothingTerm;
		float sceneTrackingWeightLevelSetTerm;
		float sceneTrackingLevelSetTermEpsilon;


	};
}
