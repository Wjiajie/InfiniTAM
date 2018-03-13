// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMDenseDynamicMapper.h"
#include "ITMMainEngine.h"
#include "ITMTrackingController.h"
#include "../Engines/LowLevel/Interface/ITMLowLevelEngine.h"
#include "../Engines/Meshing/Interface/ITMMeshingEngine.h"
#include "../Engines/ViewBuilding/Interface/ITMViewBuilder.h"
#include "../Engines/Visualisation/Interface/ITMVisualisationEngine.h"
#include "../Objects/Misc/ITMIMUCalibrator.h"

#include "../../FernRelocLib/Relocaliser.h"


namespace ITMLib
{
	template <typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
	class ITMDynamicEngine : public ITMMainEngine
	{
	private:
		const ITMLibSettings *settings;

		bool trackingActive, fusionActive, mainProcessingActive, trackingInitialised;
		int framesProcessed, relocalisationCount;

		ITMLowLevelEngine* lowLevelEngine;
		ITMVisualisationEngine<TVoxelLive, TIndex>* liveVisualisationEngine;
		ITMVisualisationEngine<TVoxelCanonical, TIndex>* canonicalVisualisationEngine;

		ITMMeshingEngine<TVoxelCanonical, TIndex>* meshingEngine;

		ITMViewBuilder* viewBuilder;
		ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>* denseMapper;
		ITMTrackingController* cameraTrackingController;

		ITMScene<TVoxelCanonical, TIndex>* canonicalScene;
		ITMScene<TVoxelLive, TIndex>* liveScene;
		ITMRenderState* renderState_live;
		ITMRenderState* renderState_freeview;

		ITMTracker* tracker;
		ITMIMUCalibrator* imuCalibrator;

		FernRelocLib::Relocaliser<float>* relocaliser;
		ITMUChar4Image* kfRaycast;

		/// Pointer for storing the current input frame
		ITMView* view;

		/// Pointer to the current camera pose and additional tracking information
		ITMTrackingState* trackingState;

	public:
		ITMView* GetView(void) { return view; }
		ITMTrackingState* GetTrackingState(void) { return trackingState; }

		/// Gives access to the internal world representation
		ITMScene<TVoxelCanonical, TIndex>* GetScene(void) { return canonicalScene; }

		ITMTrackingState::TrackingResult ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement = NULL);

		/// Extracts a mesh from the current scene and saves it to the model file specified by the file name
		void SaveSceneToMesh(const char *fileName);

		/// save and load the full scene and relocaliser (if any) to/from file
		void SaveToFile();
		void LoadFromFile();

		/// Get a result image as output
		Vector2i GetImageSize(void) const;

		void GetImage(ITMUChar4Image *out, GetImageType getImageType, ORUtils::SE3Pose *pose = NULL, ITMIntrinsics *intrinsics = NULL);

		/// resets the scene and the tracker
		void resetAll();

		/// switch for turning tracking on/off
		void turnOnTracking() override;
		void turnOffTracking() override;

		/// switch for turning integration on/off
		void turnOnIntegration() override;
		void turnOffIntegration() override;

		/// switch for turning main processing on/off
		void turnOnMainProcessing() override;
		void turnOffMainProcessing() override;

		/** \brief Constructor
			Omitting a separate image size for the depth images
			will assume same resolution as for the RGB images.
		*/
		ITMDynamicEngine(const ITMLibSettings* settings, const ITMRGBDCalib& calib, Vector2i imgSize_rgb,
		                 Vector2i imgSize_d);
		~ITMDynamicEngine();
	};
}