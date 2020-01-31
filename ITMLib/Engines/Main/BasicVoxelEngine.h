// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "Mappers/DenseMapper.h"
#include "MainEngine.h"
#include "TrackingController.h"
#include "../LowLevel/Interface/ITMLowLevelEngine.h"
#include "../Meshing/Interface/MeshingEngine.h"
#include "../ViewBuilding/Interface/ITMViewBuilder.h"
#include "../Visualization/Interface/VisualizationEngine.h"
#include "../../Objects/Misc/ITMIMUCalibrator.h"

#include "../../../FernRelocLib/Relocaliser.h"

namespace ITMLib
{
	template <typename TVoxel, typename TIndex>
	class BasicVoxelEngine : public MainEngine
	{
	private:
		bool trackingActive, fusionActive, mainProcessingActive, trackingInitialised;
		int framesProcessed, relocalisationCount;

		ITMLowLevelEngine *lowLevelEngine;
		VisualizationEngine<TVoxel, TIndex> *visualizationEngine;

		MeshingEngine<TVoxel, TIndex> *meshingEngine;

		ITMViewBuilder *viewBuilder;
		DenseMapper<TVoxel, TIndex> *denseMapper;
		TrackingController *trackingController;

		ITMVoxelVolume<TVoxel, TIndex> *scene;
		ITMRenderState *renderState_live;
		ITMRenderState *renderState_freeview;

		CameraTracker *tracker;
		ITMIMUCalibrator *imuCalibrator;

		FernRelocLib::Relocaliser<float> *relocaliser;
		ITMUChar4Image *kfRaycast;

		/// Pointer for storing the current input frame
		ITMView *view;

		/// Pointer to the current camera pose and additional tracking information
		ITMTrackingState *trackingState;

	public:
		ITMView* GetView(void) { return view; }
		ITMTrackingState* GetTrackingState(void) { return trackingState; }

		/// Gives access to the internal world representation
		ITMVoxelVolume<TVoxel, TIndex>* GetScene(void) { return scene; }

		ITMTrackingState::TrackingResult ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, ITMIMUMeasurement *imuMeasurement = NULL);

		/// Extracts a mesh from the current scene and saves it to the model file specified by the file name
		void SaveSceneToMesh(const char *fileName);

		/// save and load the full scene and relocaliser (if any) to/from file
		void SaveToFile();
		void LoadFromFile();

		/// Get a result image as output
		Vector2i GetImageSize(void) const;

		void GetImage(ITMUChar4Image *out, GetImageType getImageType, ORUtils::SE3Pose *pose = NULL, ITMIntrinsics *intrinsics = NULL);

		/// switch for turning tracking on/off
		void turnOnTracking() override;
		void turnOffTracking() override;

		/// switch for turning integration on/off
		void turnOnIntegration() override;
		void turnOffIntegration() override;

		/// switch for turning main processing on/off
		void turnOnMainProcessing() override;
		void turnOffMainProcessing() override;

		/// resets the scene and the tracker
		void resetAll();

		/** \brief Constructor
			Omitting a separate image size for the depth images
			will assume same resolution as for the RGB images.
		*/
		BasicVoxelEngine(const ITMRGBDCalib& calib, Vector2i imgSize_rgb, Vector2i imgSize_d);
		~BasicVoxelEngine();
	};
}
