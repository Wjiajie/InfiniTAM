// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMDynamicEngine.h"

#include "../Engines/LowLevel/ITMLowLevelEngineFactory.h"
#include "../Engines/Meshing/ITMMeshingEngineFactory.h"
#include "../Engines/ViewBuilding/ITMViewBuilderFactory.h"
#include "../Engines/Visualisation/ITMVisualisationEngineFactory.h"
#include "../Engines/SceneFileIO/ITMSceneFileIOEngine.h"
#include "../Objects/RenderStates/ITMRenderStateFactory.h"
#include "../CameraTrackers/ITMCameraTrackerFactory.h"

#include "../../ORUtils/NVTimer.h"
#include "../../ORUtils/FileUtils.h"

//#define OUTPUT_TRAJECTORY_QUATERNIONS

#include "../../ORUtils/FileUtils.h"
#include "../Engines/Manipulation/ITMSceneManipulation.h"

using namespace ITMLib;

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMDynamicEngine<TVoxelCanonical, TVoxelLive, TIndex>::ITMDynamicEngine(const ITMLibSettings* settings,
                                                                        const ITMRGBDCalib& calib, Vector2i imgSize_rgb,
                                                                        Vector2i imgSize_d){

	this->settings = settings;

	if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;

	MemoryDeviceType memoryType = settings->GetMemoryType();
	this->canonicalScene = new ITMScene<TVoxelCanonical, TIndex>(
			&settings->sceneParams, settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED, memoryType);

	this->liveScene = new ITMScene<TVoxelLive, TIndex>(
			&settings->sceneParams, settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED, memoryType);
	const ITMLibSettings::DeviceType deviceType = settings->deviceType;
	ITMDynamicFusionLogger::Instance().SetScenes(canonicalScene,liveScene);

	lowLevelEngine = ITMLowLevelEngineFactory::MakeLowLevelEngine(deviceType);
	viewBuilder = ITMViewBuilderFactory::MakeViewBuilder(calib, deviceType);
	liveVisualisationEngine = ITMVisualisationEngineFactory::MakeVisualisationEngine<TVoxelLive, TIndex>(deviceType);
	canonicalVisualisationEngine = ITMVisualisationEngineFactory::MakeVisualisationEngine<TVoxelCanonical, TIndex>(
			deviceType);

	meshingEngine = nullptr;
	if (settings->createMeshingEngine)
		meshingEngine = ITMMeshingEngineFactory::MakeMeshingEngine<TVoxelCanonical, TIndex>(deviceType);

	denseMapper = new ITMDenseDynamicMapper<TVoxelCanonical, TVoxelLive, TIndex>(settings);
	denseMapper->ResetCanonicalScene(canonicalScene);
	denseMapper->ResetLiveScene(liveScene);

	imuCalibrator = new ITMIMUCalibrator_iPad();
	tracker = ITMCameraTrackerFactory::Instance().Make(imgSize_rgb, imgSize_d, settings, lowLevelEngine, imuCalibrator,
	                                                   liveScene->sceneParams);
	cameraTrackingController = new ITMTrackingController(tracker, settings);

	Vector2i trackedImageSize = cameraTrackingController->GetTrackedImageSize(imgSize_rgb, imgSize_d);

	renderState_live = ITMRenderStateFactory<TIndex>::CreateRenderState(trackedImageSize, canonicalScene->sceneParams,
	                                                                    memoryType);
	renderState_freeview = nullptr; //will be created if needed

	trackingState = new ITMTrackingState(trackedImageSize, memoryType);
	tracker->UpdateInitialPose(trackingState);

	view = nullptr; // will be allocated by the view builder

	if (settings->behaviourOnFailure == settings->FAILUREMODE_RELOCALISE)
		relocaliser = new FernRelocLib::Relocaliser<float>(imgSize_d, Vector2f(settings->sceneParams.viewFrustum_min,
		                                                                       settings->sceneParams.viewFrustum_max),
		                                                   0.2f, 500, 4);
	else relocaliser = nullptr;

	kfRaycast = new ITMUChar4Image(imgSize_d, memoryType);

	trackingActive = true;
	fusionActive = true;
	mainProcessingActive = true;
	trackingInitialised = false;
	relocalisationCount = 0;
	framesProcessed = 0;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMDynamicEngine<TVoxelCanonical, TVoxelLive, TIndex>::~ITMDynamicEngine() {
	delete renderState_live;
	if (renderState_freeview != nullptr) delete renderState_freeview;

	delete canonicalScene;
	delete liveScene;

	delete denseMapper;
	delete cameraTrackingController;

	delete tracker;
	delete imuCalibrator;

	delete lowLevelEngine;
	delete viewBuilder;

	delete trackingState;
	if (view != nullptr) delete view;

	delete liveVisualisationEngine;
	delete canonicalVisualisationEngine;

	if (relocaliser != nullptr) delete relocaliser;
	delete kfRaycast;

	if (meshingEngine != nullptr) delete meshingEngine;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicEngine<TVoxelCanonical, TVoxelLive, TIndex>::SaveSceneToMesh(const char* objFileName) {
	if (meshingEngine == nullptr) return;
	ITMMesh* mesh = new ITMMesh(settings->GetMemoryType());
	meshingEngine->MeshScene(mesh, canonicalScene);
	mesh->WriteSTL(objFileName);
	delete mesh;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicEngine<TVoxelCanonical, TVoxelLive, TIndex>::SaveToFile() {
	std::string nextFrameOutputPath = ITMDynamicFusionLogger::Instance().GetOutputDirectory();
	// throws error if any of the saves fail
	if (relocaliser) relocaliser->SaveToDirectory(nextFrameOutputPath + "/Relocaliser/");
	ITMSceneFileIOEngine<TVoxelCanonical,TIndex>::SaveToDirectoryCompact(canonicalScene,nextFrameOutputPath + "/canonical");
	ITMSceneFileIOEngine<TVoxelLive,TIndex>::SaveToDirectoryCompact(liveScene,nextFrameOutputPath + "/live");
	std::cout << "Saving scenes in a compact way to '" << nextFrameOutputPath << "'." << std::endl;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicEngine<TVoxelCanonical, TVoxelLive, TIndex>::LoadFromFile() {
	std::string nextFrameOutputPath = ITMDynamicFusionLogger::Instance().GetOutputDirectory();
	std::string relocaliserInputDirectory = nextFrameOutputPath + "Relocaliser/";

	////TODO: add factory for relocaliser and rebuild using config from relocaliserOutputDirectory + "config.txt"
	////TODO: add proper management of case when scene load fails (keep old scene or also reset relocaliser)

	this->resetAll();

	if(view != nullptr){
		try // load relocaliser
		{
			FernRelocLib::Relocaliser<float>* relocaliser_temp = new FernRelocLib::Relocaliser<float>(view->depth->noDims,
			                                                                                          Vector2f(
					                                                                                          settings->sceneParams.viewFrustum_min,
					                                                                                          settings->sceneParams.viewFrustum_max),
			                                                                                          0.2f, 500, 4);

			relocaliser_temp->LoadFromDirectory(relocaliserInputDirectory);

			delete relocaliser;
			relocaliser = relocaliser_temp;
		}
		catch (std::runtime_error& e) {
			throw std::runtime_error("Could not load relocaliser: " + std::string(e.what()));
		}
	}

	try // load scene
	{
		std::cout << "Loading scenes from '" << nextFrameOutputPath << "'." << std::endl;
		ITMSceneFileIOEngine<TVoxelCanonical,TIndex>::LoadFromDirectoryCompact(canonicalScene,nextFrameOutputPath + "/canonical");
		ITMSceneFileIOEngine<TVoxelLive,TIndex>::LoadFromDirectoryCompact(liveScene,nextFrameOutputPath + "/live");
		if(framesProcessed == 0){
			framesProcessed = 1; //to skip initialization
		}
	}
	catch (std::runtime_error& e) {
		denseMapper->ResetCanonicalScene(canonicalScene);
		throw std::runtime_error("Could not load scene:" + std::string(e.what()));
	}
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicEngine<TVoxelCanonical, TVoxelLive, TIndex>::resetAll() {
	denseMapper->ResetCanonicalScene(canonicalScene);
	denseMapper->ResetLiveScene(liveScene);
	trackingState->Reset();
}

#ifdef OUTPUT_TRAJECTORY_QUATERNIONS
static int QuaternionFromRotationMatrix_variant(const double *matrix)
{
	int variant = 0;
	if
		((matrix[4]>-matrix[8]) && (matrix[0]>-matrix[4]) && (matrix[0]>-matrix[8]))
	{
		variant = 0;
	}
	else if ((matrix[4]<-matrix[8]) && (matrix[0]>
		matrix[4]) && (matrix[0]> matrix[8])) {
		variant = 1;
	}
	else if ((matrix[4]> matrix[8]) && (matrix[0]<
		matrix[4]) && (matrix[0]<-matrix[8])) {
		variant = 2;
	}
	else if ((matrix[4]<
		matrix[8]) && (matrix[0]<-matrix[4]) && (matrix[0]< matrix[8])) {
		variant = 3;
	}
	return variant;
}

static void QuaternionFromRotationMatrix(const double *matrix, double *q) {
	/* taken from "James Diebel. Representing Attitude: Euler
	Angles, Quaternions, and Rotation Vectors. Technical Report, Stanford
	University, Palo Alto, CA."
	*/

	// choose the numerically best variant...
	int variant = QuaternionFromRotationMatrix_variant(matrix);
	double denom = 1.0;
	if (variant == 0) {
		denom += matrix[0] + matrix[4] + matrix[8];
	}
	else {
		int tmp = variant * 4;
		denom += matrix[tmp - 4];
		denom -= matrix[tmp % 12];
		denom -= matrix[(tmp + 4) % 12];
	}
	denom = sqrt(denom);
	q[variant] = 0.5*denom;

	denom *= 2.0;
	switch (variant) {
	case 0:
		q[1] = (matrix[5] - matrix[7]) / denom;
		q[2] = (matrix[6] - matrix[2]) / denom;
		q[3] = (matrix[1] - matrix[3]) / denom;
		break;
	case 1:
		q[0] = (matrix[5] - matrix[7]) / denom;
		q[2] = (matrix[1] + matrix[3]) / denom;
		q[3] = (matrix[6] + matrix[2]) / denom;
		break;
	case 2:
		q[0] = (matrix[6] - matrix[2]) / denom;
		q[1] = (matrix[1] + matrix[3]) / denom;
		q[3] = (matrix[5] + matrix[7]) / denom;
		break;
	case 3:
		q[0] = (matrix[1] - matrix[3]) / denom;
		q[1] = (matrix[6] + matrix[2]) / denom;
		q[2] = (matrix[5] + matrix[7]) / denom;
		break;
	}

	if (q[0] < 0.0f) for (int i = 0; i < 4; ++i) q[i] *= -1.0f;
}
#endif


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMTrackingState::TrackingResult
ITMDynamicEngine<TVoxelCanonical, TVoxelLive, TIndex>::ProcessFrame(ITMUChar4Image* rgbImage,
                                                                    ITMShortImage* rawDepthImage,
                                                                    ITMIMUMeasurement* imuMeasurement) {

	BeginProcessingFrame(rgbImage,rawDepthImage,imuMeasurement);
	if(!mainProcessingActive) return ITMTrackingState::TRACKING_FAILED;
	fusionSucceeded = false;
	if ((lastTrackerResult == ITMTrackingState::TRACKING_GOOD || !trackingInitialised) && (fusionActive) &&
	    (relocalisationCount == 0)) {
		if (framesProcessed > 0) {
			denseMapper->ProcessFrame(view, trackingState, canonicalScene, liveScene, renderState_live);
		} else {
			denseMapper->ProcessInitialFrame(view, trackingState, canonicalScene, liveScene, renderState_live);
		}
		fusionSucceeded = true;
		if (framesProcessed > 50) trackingInitialised = true;
		framesProcessed++;
	}

	if (lastTrackerResult == ITMTrackingState::TRACKING_GOOD ||
			lastTrackerResult == ITMTrackingState::TRACKING_POOR) {
		if (!fusionSucceeded) denseMapper->UpdateVisibleList(view, trackingState, liveScene, renderState_live);

		// raycast to renderState_live for tracking and free visualisation
		cameraTrackingController->Prepare(trackingState, liveScene, view, liveVisualisationEngine, renderState_live);
	} else *trackingState->pose_d = previousFramePose;

#ifdef OUTPUT_TRAJECTORY_QUATERNIONS
	const ORUtils::SE3Pose *p = trackingState->pose_d;
	double t[3];
	double R[9];
	double q[4];
	for (int i = 0; i < 3; ++i) t[i] = p->GetInvM().m[3 * 4 + i];
	for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c)
		R[r * 3 + c] = p->GetM().m[c * 4 + r];
	QuaternionFromRotationMatrix(R, q);
	fprintf(stderr, "%f %f %f %f %f %f %f\n", t[0], t[1], t[2], q[1], q[2], q[3], q[0]);
#endif

	return lastTrackerResult;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
Vector2i ITMDynamicEngine<TVoxelCanonical, TVoxelLive, TIndex>::GetImageSize(void) const {
	return renderState_live->raycastImage->noDims;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicEngine<TVoxelCanonical, TVoxelLive, TIndex>::GetImage(ITMUChar4Image* out, GetImageType getImageType,
                                                                     ORUtils::SE3Pose* pose,
                                                                     ITMIntrinsics* intrinsics) {
	if (view == nullptr) return;

	out->Clear();

	switch (getImageType) {
		case ITMDynamicEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:
			out->ChangeDims(view->rgb->noDims);
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
				out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
			else out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
			break;
		case ITMDynamicEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:
			out->ChangeDims(view->depth->noDims);
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA) view->depth->UpdateHostFromDevice();
			ITMVisualisationEngine<TVoxelCanonical, TIndex>::DepthToUchar4(out, view->depth);
			break;
		case ITMDynamicEngine::InfiniTAM_IMAGE_SCENERAYCAST:
		case ITMDynamicEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:
		case ITMDynamicEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
		case ITMDynamicEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE: {
			// use current raycast or forward projection?
			IITMVisualisationEngine::RenderRaycastSelection raycastType;
			if (trackingState->age_pointCloud <= 0) raycastType = IITMVisualisationEngine::RENDER_FROM_OLD_RAYCAST;
			else raycastType = IITMVisualisationEngine::RENDER_FROM_OLD_FORWARDPROJ;

			// what sort of image is it?
			IITMVisualisationEngine::RenderImageType imageType;
			switch (getImageType) {
				case ITMDynamicEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:
					imageType = IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE;
					break;
				case ITMDynamicEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
					imageType = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
					break;
				case ITMDynamicEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:
					imageType = IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
					break;
				default:
					imageType = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE_IMAGENORMALS;
			}

			liveVisualisationEngine->RenderImage(liveScene, trackingState->pose_d, &view->calib.intrinsics_d,
			                                     renderState_live, renderState_live->raycastImage, imageType,
			                                     raycastType);


			ORUtils::Image<Vector4u>* srcImage = nullptr;
			if (relocalisationCount != 0) srcImage = kfRaycast;
			else srcImage = renderState_live->raycastImage;

			out->ChangeDims(srcImage->noDims);
			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
				out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
			else out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);

			break;
		}
		case ITMDynamicEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
		case ITMDynamicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
		case ITMDynamicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
		case ITMDynamicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE: {
			IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;
			if (getImageType == ITMDynamicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME)
				type = IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
			else if (getImageType == ITMDynamicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL)
				type = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
			else if (getImageType == ITMDynamicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE)
				type = IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE;

			if (renderState_freeview == nullptr) {
				renderState_freeview = ITMRenderStateFactory<TIndex>::CreateRenderState(out->noDims,
				                                                                        liveScene->sceneParams,
				                                                                        settings->GetMemoryType());
			}

			liveVisualisationEngine->FindVisibleBlocks(liveScene, pose, intrinsics, renderState_freeview);
			liveVisualisationEngine->CreateExpectedDepths(liveScene, pose, intrinsics, renderState_freeview);
			liveVisualisationEngine->RenderImage(liveScene, pose, intrinsics, renderState_freeview,
			                                     renderState_freeview->raycastImage, type);

			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
				out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
			else out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
			break;
		}
		case ITMMainEngine::InfiniTAM_IMAGE_STEP_BY_STEP: {

			if (renderState_freeview == nullptr) {
				renderState_freeview = ITMRenderStateFactory<TIndex>::CreateRenderState(out->noDims,
				                                                                        liveScene->sceneParams,
				                                                                        settings->GetMemoryType());
			}

			liveVisualisationEngine->FindVisibleBlocks(liveScene, pose, intrinsics, renderState_freeview);
			liveVisualisationEngine->CreateExpectedDepths(liveScene, pose, intrinsics, renderState_freeview);
			liveVisualisationEngine->RenderImage(liveScene, pose, intrinsics, renderState_freeview,
			                                     renderState_freeview->raycastImage,
			                                     IITMVisualisationEngine::RENDER_SHADED_GREEN);
			canonicalVisualisationEngine->FindVisibleBlocks(canonicalScene, pose, intrinsics, renderState_freeview);
			canonicalVisualisationEngine->CreateExpectedDepths(canonicalScene, pose, intrinsics, renderState_freeview);
			canonicalVisualisationEngine->RenderImage(canonicalScene, pose, intrinsics, renderState_freeview,
			                                          renderState_freeview->raycastImage,
			                                          IITMVisualisationEngine::RENDER_SHADED_OVERLAY);


			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
				out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
			else out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);


			break;
		}
		case ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_CANONICAL: {
			IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;

			if (renderState_freeview == nullptr) {
				renderState_freeview = ITMRenderStateFactory<TIndex>::CreateRenderState(out->noDims,
				                                                                        canonicalScene->sceneParams,
				                                                                        settings->GetMemoryType());
			}

			canonicalVisualisationEngine->FindVisibleBlocks(canonicalScene, pose, intrinsics, renderState_freeview);
			canonicalVisualisationEngine->CreateExpectedDepths(canonicalScene, pose, intrinsics, renderState_freeview);
			canonicalVisualisationEngine->RenderImage(canonicalScene, pose, intrinsics, renderState_freeview,
			                                          renderState_freeview->raycastImage, type);

			if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
				out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
			else out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
			break;
		}

		case ITMMainEngine::InfiniTAM_IMAGE_UNKNOWN:
			break;
	};
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicEngine<TVoxelCanonical, TVoxelLive, TIndex>::turnOnTracking() { trackingActive = true; }

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicEngine<TVoxelCanonical, TVoxelLive, TIndex>::turnOffTracking() { trackingActive = false; }

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicEngine<TVoxelCanonical, TVoxelLive, TIndex>::turnOnIntegration() { fusionActive = true; }

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicEngine<TVoxelCanonical, TVoxelLive, TIndex>::turnOffIntegration() { fusionActive = false; }

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicEngine<TVoxelCanonical, TVoxelLive, TIndex>::turnOnMainProcessing() { mainProcessingActive = true; }

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicEngine<TVoxelCanonical, TVoxelLive, TIndex>::turnOffMainProcessing() { mainProcessingActive = false; }

// region ==================================== STEP-BY-STEP MODE =======================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicEngine<TVoxelCanonical, TVoxelLive, TIndex>::BeginProcessingFrameInStepByStepMode(
		ITMUChar4Image* rgbImage,
		ITMShortImage* rawDepthImage,
		ITMIMUMeasurement* imuMeasurement) {

	BeginProcessingFrame(rgbImage,rawDepthImage,imuMeasurement);
	fusionSucceeded = false;
	canFuse = false;
	if ((lastTrackerResult == ITMTrackingState::TRACKING_GOOD || !trackingInitialised) && (fusionActive) &&
	    (relocalisationCount == 0)) {
		canFuse = true;
		denseMapper->BeginProcessingFrameInStepByStepMode(view, trackingState, canonicalScene, liveScene,
		                                                  renderState_live);
	}
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
bool ITMDynamicEngine<TVoxelCanonical, TVoxelLive, TIndex>::UpdateCurrentFrameSingleStep() {

	bool trackingNotFinished;
	if (canFuse) {
		trackingNotFinished = denseMapper->TakeNextStepInStepByStepMode(canonicalScene, liveScene, renderState_live);
		if (trackingNotFinished) {
			fusionSucceeded = true;
			if (framesProcessed > 50) trackingInitialised = true;
			framesProcessed++;
		}
	} else {
		trackingNotFinished = false;
	}
	if (!trackingNotFinished) {
		if (lastTrackerResult == ITMTrackingState::TRACKING_GOOD ||
		    lastTrackerResult == ITMTrackingState::TRACKING_POOR) {
			if (!fusionSucceeded) denseMapper->UpdateVisibleList(view, trackingState, liveScene, renderState_live);

			// raycast to renderState_live for tracking and free visualisation
			cameraTrackingController->Prepare(trackingState, liveScene, view, liveVisualisationEngine,
			                                  renderState_live);
		} else *trackingState->pose_d = previousFramePose;
	}
	return trackingNotFinished;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMTrackingState::TrackingResult ITMDynamicEngine<TVoxelCanonical, TVoxelLive, TIndex>::GetStepByStepTrackingResult() {
	return this->lastTrackerResult;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMDynamicEngine<TVoxelCanonical, TVoxelLive, TIndex>::BeginProcessingFrame(ITMUChar4Image* rgbImage,
                                                                                 ITMShortImage* rawDepthImage,
                                                                                 ITMIMUMeasurement* imuMeasurement) {

	// prepare image and turn it into a depth image
	if (imuMeasurement == nullptr)
		viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useThresholdFilter,
		                        settings->useBilateralFilter, false, false);
	else
		viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useThresholdFilter,
		                        settings->useBilateralFilter, imuMeasurement, false, false);

	if (!mainProcessingActive){
		lastTrackerResult = ITMTrackingState::TRACKING_FAILED;
		return;
	}

	// tracking
	previousFramePose = (*(trackingState->pose_d));
	if (trackingActive) cameraTrackingController->Track(trackingState, view);

	lastTrackerResult = ITMTrackingState::TRACKING_GOOD;

	switch (settings->behaviourOnFailure) {
		case ITMLibSettings::FAILUREMODE_RELOCALISE:
			//relocalisation
			lastTrackerResult = trackingState->trackerResult;
			if (lastTrackerResult == ITMTrackingState::TRACKING_GOOD && relocalisationCount > 0)
				relocalisationCount--;

			view->depth->UpdateHostFromDevice();

			{
				int NN;
				float distances;
				//find and add keyframe, if necessary
				bool hasAddedKeyframe = relocaliser->ProcessFrame(view->depth, trackingState->pose_d, 0, 1, &NN,
				                                                  &distances,
				                                                  lastTrackerResult ==
				                                                  ITMTrackingState::TRACKING_GOOD &&
				                                                  relocalisationCount == 0);

				//frame not added and tracking failed -> we need to relocalise
				if (!hasAddedKeyframe && lastTrackerResult == ITMTrackingState::TRACKING_FAILED) {
					relocalisationCount = 10;

					// Reset previous rgb frame since the rgb image is likely different than the one acquired when setting the keyframe
					view->rgb_prev->Clear();

					const FernRelocLib::PoseDatabase::PoseInScene& keyframe = relocaliser->RetrievePose(NN);
					trackingState->pose_d->SetFrom(&keyframe.pose);

					denseMapper->UpdateVisibleList(view, trackingState, liveScene, renderState_live, true);

					cameraTrackingController->Prepare(trackingState, liveScene, view, liveVisualisationEngine,
					                                  renderState_live);
					cameraTrackingController->Track(trackingState, view);

					lastTrackerResult = trackingState->trackerResult;
				}
			}
			break;
		case ITMLibSettings::FAILUREMODE_STOP_INTEGRATION:
			if (trackingState->trackerResult != ITMTrackingState::TRACKING_FAILED)
				lastTrackerResult = trackingState->trackerResult;
			else lastTrackerResult = ITMTrackingState::TRACKING_POOR;
			break;
		default:
			break;
	}




}

// endregion ===========================================================================================================