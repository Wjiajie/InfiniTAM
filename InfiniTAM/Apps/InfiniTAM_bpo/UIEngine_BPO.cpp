// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "UIEngine_BPO.h"

#include <string.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else

#include <GL/glut.h>

#endif

#ifdef FREEGLUT

#include <GL/freeglut.h>

#else
#if (!defined USING_CMAKE) && (defined _MSC_VER)
#pragma comment(lib, "glut64")
#endif
#endif

#include "../../ITMLib/ITMLibDefines.h"
#include "../../ITMLib/Core/ITMBasicEngine.h"
#include "../../ITMLib/Core/ITMBasicSurfelEngine.h"
#include "../../ITMLib/Core/ITMMultiEngine.h"

#include "../../ORUtils/FileUtils.h"
#include "../../InputSource/FFMPEGWriter.h"
#include "../../ITMLib/Utils/ITMBenchmarkUtils.h"
#include "../../ITMLib/Core/ITMDynamicEngine.h"

using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;


namespace bench = ITMLib::Bench;

UIEngine_BPO* UIEngine_BPO::instance;


/**
 * \brief Initialize the UIEngine using the specified settings
 * \param argc arguments to the main function
 * \param argv number of arguments to the main function
 * \param imageSource source for images
 * \param imuSource source for IMU data
 * \param mainEngine main engine to process the frames
 * \param outFolder output folder for saving results
 * \param deviceType type of device to use for some tasks
 * \param frameIntervalLength automatically process this number of frames after launching the UIEngine,
 * \param skipFirstNFrames skip this number of frames before beginning to process
 * \param recordReconstructionResult start recording the reconstruction result into a video files as soon as the next frame is processed
 * set interval to this number of frames
 */
void UIEngine_BPO::Initialise(int& argc, char** argv, InputSource::ImageSourceEngine* imageSource,
                              InputSource::IMUSourceEngine* imuSource,
                              ITMLib::ITMMainEngine* mainEngine, const char* outFolder,
                              ITMLib::ITMLibSettings::DeviceType deviceType,
                              int frameIntervalLength, int skipFirstNFrames, bool recordReconstructionResult) {
	this->inStepByStepMode = false;
	this->freeviewActive = true;
	this->integrationActive = true;
	this->currentColourMode = 0;
	this->autoIntervalFrameCount = frameIntervalLength;

	this->colourModes_main.emplace_back("shaded greyscale", ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST);
	this->colourModes_main.emplace_back("integrated colours", ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME);
	this->colourModes_main.emplace_back("surface normals", ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL);
	this->colourModes_main.emplace_back("confidence", ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE);

	this->colourModes_freeview.emplace_back("canonical", ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_CANONICAL);
	this->colourModes_freeview.emplace_back("shaded greyscale", ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED);
	this->colourModes_freeview.emplace_back("integrated colours",
	                                        ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME);
	this->colourModes_freeview.emplace_back("surface normals",
	                                        ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL);
	this->colourModes_freeview.emplace_back("confidence",
	                                        ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE);

	this->imageSource = imageSource;
	this->imuSource = imuSource;
	this->mainEngine = mainEngine;
	{
		size_t len = strlen(outFolder);
		this->outFolder = new char[len + 1];
		strcpy(this->outFolder, outFolder);
	}

	int textHeight = 60; // Height of text area, 2 lines

	winSize.x = (int) (1.5f * (float) (imageSource->getDepthImageSize().x));
	winSize.y = imageSource->getDepthImageSize().y + textHeight;
	float h1 = textHeight / (float) winSize.y, h2 = (1.f + h1) / 2;
	winReg[0] = Vector4f(0.0f, h1, 0.665f, 1.0f);   // Main render
	winReg[1] = Vector4f(0.665f, h2, 1.0f, 1.0f);   // Side sub window 0
	winReg[2] = Vector4f(0.665f, h1, 1.0f, h2);     // Side sub window 2

	this->isRecordingImages = false;
	this->currentFrameNo = 0;
	this->rgbVideoWriter = nullptr;
	this->depthVideoWriter = nullptr;

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(winSize.x, winSize.y);
	glutCreateWindow("InfiniTAM");
	glGenTextures(NUM_WIN, textureId);

	glutDisplayFunc(UIEngine_BPO::GlutDisplayFunction);
	glutKeyboardUpFunc(UIEngine_BPO::GlutKeyUpFunction);
	glutMouseFunc(UIEngine_BPO::GlutMouseButtonFunction);
	glutMotionFunc(UIEngine_BPO::GlutMouseMoveFunction);
	glutIdleFunc(UIEngine_BPO::GlutIdleFunction);

#ifdef FREEGLUT
	glutMouseWheelFunc(UIEngine_BPO::GlutMouseWheelFunction);
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, 1);
#endif

	bool allocateGPU = false;
	if (deviceType == ITMLibSettings::DEVICE_CUDA) allocateGPU = true;

	for (int w = 0; w < NUM_WIN; w++)
		outImage[w] = new ITMUChar4Image(imageSource->getDepthImageSize(), true, allocateGPU);

	inputRGBImage = new ITMUChar4Image(imageSource->getRGBImageSize(), true, allocateGPU);
	inputRawDepthImage = new ITMShortImage(imageSource->getDepthImageSize(), true, allocateGPU);
	inputIMUMeasurement = new ITMIMUMeasurement();

	saveImage = new ITMUChar4Image(imageSource->getDepthImageSize(), true, false);

	outImageType[0] = this->freeviewActive ? this->colourModes_freeview[this->currentColourMode].type
	                                       : this->colourModes_main[this->currentColourMode].type;
	outImageType[1] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH;
	outImageType[2] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB;
	if (inputRGBImage->noDims == Vector2i(0, 0)) outImageType[2] = ITMMainEngine::InfiniTAM_IMAGE_UNKNOWN;

	mainLoopAction = autoIntervalFrameCount ? PROCESS_N_FRAMES : PROCESS_PAUSED;
	autoIntervalFrameStart = 0;
	mouseState = 0;
	mouseWarped = false;
	needsRefresh = false;
	processedFrameNo = 0;
	processedTime = 0.0f;

#ifndef COMPILE_WITHOUT_CUDA
	ORcudaSafeCall(cudaThreadSynchronize());
#endif

	sdkCreateTimer(&timer_instant);
	sdkCreateTimer(&timer_average);

	sdkResetTimer(&timer_average);
	if (skipFirstNFrames > 0) {
		printf("Skipping the first %d frames.\n", skipFirstNFrames);
		SkipFrames(skipFirstNFrames);
	}
	if(recordReconstructionResult){
		this->reconstructionVideoWriter = new FFMPEGWriter();
	}
	printf("initialised.\n");
}

void UIEngine_BPO::SaveScreenshot(const char* filename) const {
	ITMUChar4Image screenshot(GetWindowSize(), true, false);
	GetScreenshot(&screenshot);
	SaveImageToFile(&screenshot, filename, true);
}

void UIEngine_BPO::GetScreenshot(ITMUChar4Image* dest) const {
	glReadPixels(0, 0, dest->noDims.x, dest->noDims.y, GL_RGBA, GL_UNSIGNED_BYTE, dest->GetData(MEMORYDEVICE_CPU));
}

void UIEngine_BPO::SkipFrames(int numberOfFramesToSkip) {
	for (int iFrame = 0; iFrame < numberOfFramesToSkip && imageSource->hasMoreImages(); iFrame++) {
		imageSource->getImages(inputRGBImage, inputRawDepthImage);
	}
	this->startedProcessingFromFrameIx += numberOfFramesToSkip;
}


void UIEngine_BPO::ProcessFrame() {
	if (!imageSource->hasMoreImages()) return;
	imageSource->getImages(inputRGBImage, inputRawDepthImage);

	if (imuSource != NULL) {
		if (!imuSource->hasMoreMeasurements()) return;
		else imuSource->getMeasurement(inputIMUMeasurement);
	}

	RecordDepthAndRGBInputToImages();
	RecordDepthAndRGBInputToVideo();

	sdkResetTimer(&timer_instant);
	sdkStartTimer(&timer_instant);
	sdkStartTimer(&timer_average);

	ITMTrackingState::TrackingResult trackerResult;
	mainEngine->recordNextFrameWarps = this->recordWarpUpdatesForNextFrame;
	//actual processing on the mailEngine
	if (imuSource != nullptr)
		trackerResult = mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, inputIMUMeasurement);
	else trackerResult = mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);

	trackingResult = (int) trackerResult;

#ifndef COMPILE_WITHOUT_CUDA
	ORcudaSafeCall(cudaThreadSynchronize());
#endif
	sdkStopTimer(&timer_instant);
	sdkStopTimer(&timer_average);

	//processedTime = sdkGetTimerValue(&timer_instant);
	processedTime = sdkGetAverageTimerValue(&timer_average);

	RecordReconstructionToVideo();

	currentFrameNo++;
}

void UIEngine_BPO::Run() { glutMainLoop(); }

void UIEngine_BPO::Shutdown() {
	sdkDeleteTimer(&timer_instant);
	sdkDeleteTimer(&timer_average);

	if (rgbVideoWriter != NULL) delete rgbVideoWriter;
	if (depthVideoWriter != NULL) delete depthVideoWriter;
	if (reconstructionVideoWriter != NULL) delete reconstructionVideoWriter;

	for (int w = 0; w < NUM_WIN; w++)
		delete outImage[w];

	delete inputRGBImage;
	delete inputRawDepthImage;
	delete inputIMUMeasurement;

	delete[] outFolder;
	delete saveImage;
	delete instance;
	instance = NULL;
}

bool UIEngine_BPO::BeginStepByStepModeForFrame() {
	if (!imageSource->hasMoreImages()) return false;

	auto* dynamicEngine = dynamic_cast<ITMDynamicEngine<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>*>(mainEngine);

	if (dynamicEngine == nullptr) return false;


	imageSource->getImages(inputRGBImage, inputRawDepthImage);

	if (imuSource != NULL) {
		if (!imuSource->hasMoreMeasurements()) return false;
		else imuSource->getMeasurement(inputIMUMeasurement);
	}

	RecordDepthAndRGBInputToImages();
	RecordDepthAndRGBInputToVideo();

	//ITMTrackingState::TrackingResult trackerResult;
	mainEngine->recordNextFrameWarps = this->recordWarpUpdatesForNextFrame;
	//actual processing on the mailEngine
	if (imuSource != NULL)
		dynamicEngine->BeginProcessingFrameInStepByStepMode(inputRGBImage, inputRawDepthImage, inputIMUMeasurement);
	else dynamicEngine->BeginProcessingFrameInStepByStepMode(inputRGBImage, inputRawDepthImage);

	return true;
}

bool UIEngine_BPO::ContinueStepByStepModeForFrame() {
	auto* dynamicEngine = dynamic_cast<ITMDynamicEngine<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>*>(mainEngine);
	if (dynamicEngine == nullptr) return false;
	bool keepProcessingFrame = dynamicEngine->UpdateCurrentFrameSingleStep();
	if (!keepProcessingFrame) {
		trackingResult = dynamicEngine->GetStepByStepTrackingResult();
#ifndef COMPILE_WITHOUT_CUDA
		ORcudaSafeCall(cudaThreadSynchronize());
#endif
		currentFrameNo++;
	} else {
		RecordReconstructionToVideo();
	}
	return keepProcessingFrame;
}

void UIEngine_BPO::RecordReconstructionToVideo() {
	if ((reconstructionVideoWriter != nullptr)) {
		mainEngine->GetImage(outImage[0], outImageType[0], &this->freeviewPose, &freeviewIntrinsics);
		if (outImage[0]->noDims.x != 0) {
			if (!reconstructionVideoWriter->isOpen())
				reconstructionVideoWriter->open((std::string(this->outFolder) + "/out_reconstruction.avi").c_str(),
				                                outImage[0]->noDims.x, outImage[0]->noDims.y,
				                                false, 30);
			reconstructionVideoWriter->writeFrame(outImage[0]);
		}
	}
}

void UIEngine_BPO::RecordDepthAndRGBInputToVideo() {
	if ((rgbVideoWriter != nullptr) && (inputRGBImage->noDims.x != 0)) {
		if (!rgbVideoWriter->isOpen())
			rgbVideoWriter->open((std::string(this->outFolder) + "/out_rgb.avi").c_str(),
			                     inputRGBImage->noDims.x, inputRGBImage->noDims.y, false, 30);
		rgbVideoWriter->writeFrame(inputRGBImage);
	}
	if ((depthVideoWriter != nullptr) && (inputRawDepthImage->noDims.x != 0)) {
		if (!depthVideoWriter->isOpen())
			depthVideoWriter->open((std::string(this->outFolder) + "/out_depth.avi").c_str(),
			                       inputRawDepthImage->noDims.x, inputRawDepthImage->noDims.y, true, 30);
		depthVideoWriter->writeFrame(inputRawDepthImage);
	}
}

void UIEngine_BPO::RecordDepthAndRGBInputToImages() {
	if (isRecordingImages) {
		char str[250];

		sprintf(str, "%s/%04d.pgm", outFolder, currentFrameNo);
		SaveImageToFile(inputRawDepthImage, str);

		if (inputRGBImage->noDims != Vector2i(0, 0)) {
			sprintf(str, "%s/%04d.ppm", outFolder, currentFrameNo);
			SaveImageToFile(inputRGBImage, str);
		}
	}
}

