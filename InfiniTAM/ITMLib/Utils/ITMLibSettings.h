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
			LIBMODE_DYNAMIC //for basic KillingFusion mode, refer to DOI 10.1109/CVPR.2017.581
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

		const char *outputPath;

		ITMLibSettings();
		virtual ~ITMLibSettings(void) = default;

		// Suppress the default copy constructor and assignment operator
		ITMLibSettings(const ITMLibSettings&) = delete;
		ITMLibSettings& operator=(const ITMLibSettings&) = delete;

		MemoryDeviceType GetMemoryType() const;

		/// Dynamic Fusion parameters
		//*** Analysis/debugging Switches
		bool rasterizeLiveSceneSlices;// = false;
		bool rasterizeCanonicalSceneSlices;// = false;
		// = false; // CLI flag made in InfiniTAM_bpo

		bool restrictZtrackingForDebugging;// = false;
		bool simpleSceneExperimentModeEnabled;// = false;
		bool FocusCoordinatesAreSpecified() const;// = false; // CLI flag made in InfiniTAM_bpo

		Vector3i GetFocusCoordinates() const;
		void SetFocusCoordinates(const Vector3i& coordiantes);


		//*** Scene Tracking Switches ***
		bool enableDataTerm;
		bool enableLevelSetTerm;
		bool enableSmoothingTerm;
		bool enableKillingTerm;
		bool enableGradientSmoothing;

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


	private:
		/// Parameters for logging/debugging dynamic fusion
		bool focusCoordinatesSpecified = false;
		Vector3i focusCoordinates;

	};
}
