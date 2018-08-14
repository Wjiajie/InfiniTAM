// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "CLIEngine_BPO.h"

#include <string.h>

//VTK
#include <vtkCommand.h>
#include <vtkRenderWindowInteractor.h>

//ITMLib
#include "../../ITMLib/ITMLibDefines.h"
#include "../../ITMLib/Core/ITMBasicEngine.h"
#include "../../ITMLib/Core/ITMBasicSurfelEngine.h"
#include "../../ITMLib/Core/ITMMultiEngine.h"

#include "../../ORUtils/FileUtils.h"
#include "../../InputSource/FFMPEGWriter.h"
#include "../../ITMLib/Utils/Analytics/ITMBenchmarkUtils.h"
#include "../../ITMLib/Core/ITMDynamicEngine.h"
#include "../../ITMLib/Utils/ITMPrintHelpers.h"
#include "../../ITMLib/Objects/Scene/ITMIndexEnumeration.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>


//TODO: we should never have to downcast the main engine to some other engine type, architecture needs to be altered
// (potentially by introducting empty method stubs) -Greg (GitHub:Algomorph)

using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;


namespace bench = ITMLib::Bench;


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
void CLIEngine_BPO::Initialise(int& argc, char** argv, InputSource::ImageSourceEngine* imageSource,
                               InputSource::IMUSourceEngine* imuSource,
                               ITMLib::ITMMainEngine* mainEngine, const char* outFolder,
                               ITMLib::ITMLibSettings::DeviceType deviceType,
                               int frameIntervalLength, int skipFirstNFrames, bool recordReconstructionResult,
                               bool startInStepByStep,
                               bool saveAfterFirstNFrames, bool loadBeforeProcessing,
                               ITMLib::ITMDynamicFusionLogger_Interface* logger,
                               ITMLib::IndexingMethod indexingMethod) {
	this->logger = logger;
	this->indexingMethod = indexingMethod;

	this->inStepByStepMode = startInStepByStep;
	this->saveAfterAutoprocessing = saveAfterFirstNFrames;

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

	allocateGPU = false;
	if (deviceType == ITMLibSettings::DEVICE_CUDA) allocateGPU = true;

	for (int w = 0; w < NUM_WIN; w++) {
		outImage[w] = new ITMUChar4Image(imageSource->getDepthImageSize(), true, allocateGPU);
	}

	inputRGBImage = new ITMUChar4Image(imageSource->getRGBImageSize(), true, allocateGPU);
	inputRawDepthImage = new ITMShortImage(imageSource->getDepthImageSize(), true, allocateGPU);
	inputIMUMeasurement = new ITMIMUMeasurement();

	saveImage = new ITMUChar4Image(imageSource->getDepthImageSize(), true, false);


	outImageType[1] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH;
	outImageType[2] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB;
	if (inputRGBImage->noDims == Vector2i(0, 0)) outImageType[2] = ITMMainEngine::InfiniTAM_IMAGE_UNKNOWN;


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

	if (startInStepByStep) {
		BeginStepByStepModeForFrame();
		mainLoopAction = autoIntervalFrameCount ? PROCESS_STEPS_CONTINUOUS : PROCESS_SINGLE_STEP;
		outImageType[0] = this->colourMode_stepByStep.type;
	} else {
		mainLoopAction = autoIntervalFrameCount ? PROCESS_N_FRAMES : PROCESS_PAUSED;
		outImageType[0] = this->freeviewActive ? this->colourModes_freeview[this->currentColourMode].type
		                                       : this->colourModes_main[this->currentColourMode].type;
	}

	if (recordReconstructionResult) {
		this->reconstructionVideoWriter = new FFMPEGWriter();
	}

	if (loadBeforeProcessing) {
		logger->SetOutputDirectory(
				this->GenerateCurrentFrameOutputDirectory());
		mainEngine->LoadFromFile();
		SkipFrames(1);
	}
	logger->SetShutdownRequestedFlagLocation(
			&this->shutdownRequested);
	printf("initialised.\n");
}

void CLIEngine_BPO::SkipFrames(int numberOfFramesToSkip) {
	for (int iFrame = 0; iFrame < numberOfFramesToSkip && imageSource->hasMoreImages(); iFrame++) {
		imageSource->getImages(inputRGBImage, inputRawDepthImage);
	}
	this->startedProcessingFromFrameIx += numberOfFramesToSkip;
}


bool CLIEngine_BPO::ProcessFrame() {
	if (logger->IsRecording3DSceneAndWarpProgression()) {
		std::cout << yellow << "***" << bright_cyan << "PROCESSING FRAME " << GetCurrentFrameIndex()
		          << " (WITH RECORDING 3D SCENES ON)" << yellow << "***" << reset << std::endl;
	} else {
		std::cout << yellow << "***" << bright_cyan << "PROCESSING FRAME " << GetCurrentFrameIndex() << yellow << "***"
		          << reset << std::endl;
	}

	if (!imageSource->hasMoreImages()) return false;
	imageSource->getImages(inputRGBImage, inputRawDepthImage);

	if (imuSource != nullptr) {
		if (!imuSource->hasMoreMeasurements()) return false;
		else imuSource->getMeasurement(inputIMUMeasurement);
	}

	logger->SetOutputDirectory(
			this->GenerateCurrentFrameOutputDirectory());
	RecordDepthAndRGBInputToImages();
	RecordDepthAndRGBInputToVideo();

	sdkResetTimer(&timer_instant);
	sdkStartTimer(&timer_instant);
	sdkStartTimer(&timer_average);

	//actual processing on the mailEngine
	if (imuSource != nullptr)
		this->trackingResult = mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, inputIMUMeasurement);
	else trackingResult = mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);

#ifndef COMPILE_WITHOUT_CUDA
	ORcudaSafeCall(cudaThreadSynchronize());
#endif
	sdkStopTimer(&timer_instant);
	sdkStopTimer(&timer_average);

	//processedTime = sdkGetTimerValue(&timer_instant);
	processedTime = sdkGetAverageTimerValue(&timer_average);

	RecordReconstructionToVideo();

	currentFrameNo++;
	return true;
}

int CLIEngine_BPO::GetCurrentFrameIndex() const {
	return startedProcessingFromFrameIx + currentFrameNo;
}

void CLIEngine_BPO::Run() {
	while (true) {
		if (!ProcessFrame()) break;
	}
}


//TODO: should just be in the destructor and triggered when the object goes out of scope -Greg (GitHub:Algomorph)
void CLIEngine_BPO::Shutdown() {
	sdkDeleteTimer(&timer_instant);
	sdkDeleteTimer(&timer_average);

	delete rgbVideoWriter;
	delete depthVideoWriter;
	delete reconstructionVideoWriter;

	for (int w = 0; w < NUM_WIN; w++)
		delete outImage[w];

	delete inputRGBImage;
	delete inputRawDepthImage;
	delete inputIMUMeasurement;

	delete[] outFolder;
	delete saveImage;
}

bool CLIEngine_BPO::BeginStepByStepModeForFrame() {
	if (!imageSource->hasMoreImages()) return false;

	switch (indexingMethod) {
		case HASH: {
			auto* dynamicEngine = dynamic_cast<ITMDynamicEngine<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelBlockHash>*>(mainEngine);
			if (dynamicEngine == nullptr) return false;
		}
			break;
		case ARRAY: {
			auto* dynamicEngine = dynamic_cast<ITMDynamicEngine<ITMVoxelCanonical, ITMVoxelLive, ITMPlainVoxelArray>*>(mainEngine);
			if (dynamicEngine == nullptr) return false;
		}
			break;
	}


	imageSource->getImages(inputRGBImage, inputRawDepthImage);

	if (imuSource != nullptr) {
		if (!imuSource->hasMoreMeasurements()) return false;
		else imuSource->getMeasurement(inputIMUMeasurement);
	}

	logger->SetOutputDirectory(
			this->GenerateCurrentFrameOutputDirectory());
	RecordDepthAndRGBInputToImages();
	RecordDepthAndRGBInputToVideo();

	//actual processing on the mailEngine
	switch (indexingMethod) {
		case HASH: {
			auto* dynamicEngine = dynamic_cast<ITMDynamicEngine<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelBlockHash>*>(mainEngine);
			if (imuSource != nullptr)
				dynamicEngine->BeginProcessingFrameInStepByStepMode(inputRGBImage, inputRawDepthImage,
				                                                    inputIMUMeasurement);
			else dynamicEngine->BeginProcessingFrameInStepByStepMode(inputRGBImage, inputRawDepthImage);
		}
			break;
		case ARRAY: {
			auto* dynamicEngine = dynamic_cast<ITMDynamicEngine<ITMVoxelCanonical, ITMVoxelLive, ITMPlainVoxelArray>*>(mainEngine);
			if (imuSource != nullptr)
				dynamicEngine->BeginProcessingFrameInStepByStepMode(inputRGBImage, inputRawDepthImage,
				                                                    inputIMUMeasurement);
			else dynamicEngine->BeginProcessingFrameInStepByStepMode(inputRGBImage, inputRawDepthImage);
		}
			break;
	}
	return true;
}

std::string CLIEngine_BPO::GenerateNextFrameOutputPath() const {
	fs::path path(std::string(this->outFolder) + "/Frame_" + std::to_string(GetCurrentFrameIndex() + 1));
	if (!fs::exists(path)) {
		fs::create_directories(path);
	}
	return path.string();
}

std::string CLIEngine_BPO::GenerateCurrentFrameOutputDirectory() const {
	fs::path path(std::string(this->outFolder) + "/Frame_" + std::to_string(GetCurrentFrameIndex()));
	if (!fs::exists(path)) {
		fs::create_directories(path);
	}
	return path.string();
}

bool CLIEngine_BPO::ContinueStepByStepModeForFrame() {
	bool keepProcessingFrame = false;
	switch (indexingMethod) {
		case HASH: {
			auto* dynamicEngine = dynamic_cast<ITMDynamicEngine<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelBlockHash>*>(mainEngine);
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
		}
			break;
		case ARRAY: {
			auto* dynamicEngine = dynamic_cast<ITMDynamicEngine<ITMVoxelCanonical, ITMVoxelLive, ITMPlainVoxelArray>*>(mainEngine);
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
		}
			break;
	}

	return keepProcessingFrame;
}

//TODO: Group all recording & make it toggleable with a single keystroke / command flag
void CLIEngine_BPO::RecordReconstructionToVideo() {
	if ((reconstructionVideoWriter != nullptr)) {
		mainEngine->GetImage(outImage[0], outImageType[0], &this->freeviewPose, &freeviewIntrinsics);
		if (outImage[0]->noDims.x != 0) {
			if (!reconstructionVideoWriter->isOpen())
				reconstructionVideoWriter->open((std::string(this->outFolder) + "/out_reconstruction.avi").c_str(),
				                                outImage[0]->noDims.x, outImage[0]->noDims.y,
				                                false, 30);
			//TODO This image saving/reading/saving is a production hack -Greg (GitHub:Algomorph)
			//TODO move to a separate function and apply to all recorded video
			std::string fileName = (std::string(this->outFolder) + "/out_reconstruction.png");
			SaveImageToFile(outImage[0], fileName.c_str());
			cv::Mat img = cv::imread(fileName, cv::IMREAD_UNCHANGED);
			cv::putText(img, std::to_string(GetCurrentFrameIndex()), cv::Size(10, 50), cv::FONT_HERSHEY_SIMPLEX,
			            1, cv::Scalar(128, 255, 128), 1, cv::LINE_AA);
			cv::imwrite(fileName, img);
			ITMUChar4Image* imageWithText = new ITMUChar4Image(imageSource->getDepthImageSize(), true, allocateGPU);
			ReadImageFromFile(imageWithText, fileName.c_str());
			reconstructionVideoWriter->writeFrame(imageWithText);
			delete imageWithText;
		}
	}
}

void CLIEngine_BPO::RecordDepthAndRGBInputToVideo() {
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

void CLIEngine_BPO::RecordDepthAndRGBInputToImages() {
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

void CLIEngine_BPO::PrintProcessingFrameHeader() const {
	std::cout << bright_cyan << "PROCESSING FRAME " << GetCurrentFrameIndex() + 1;
	if (logger->IsRecording3DSceneAndWarpProgression()) {
		std::cout << " [3D SCENE AND WARP UPDATE RECORDING: ON]";
	}
	if (logger->IsRecordingScene2DSlicesWithUpdates()) {
		std::cout << " [2D SCENE SLICE & WARP UPDATE RECORDING: ON]";
	}
	std::cout << reset << std::endl;
}
