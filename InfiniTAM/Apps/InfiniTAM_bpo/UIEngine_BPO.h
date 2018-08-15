// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

//local
#include "../../InputSource/ImageSourceEngine.h"
#include "../../InputSource/IMUSourceEngine.h"
#include "../../InputSource/FFMPEGWriter.h"
#include "../../ITMLib/Core/ITMMainEngine.h"
#include "../../ITMLib/Objects/Tracking/ITMTrackingState.h"
#include "../../ITMLib/Utils/ITMLibSettings.h"
#include "../../ORUtils/FileUtils.h"
#include "../../ORUtils/NVTimer.h"
#include "../../ITMLib/Utils/FileIO/ITMDynamicFusionLogger.h"
#include "../../ITMLib/Objects/Scene/ITMIndexEnumeration.h"

//stdlib
#include <vector>

//boost
#include <boost/filesystem.hpp>


namespace InfiniTAM {
namespace Engine {
class UIEngine_BPO {
private:

	enum MainLoopAction {
		PROCESS_PAUSED, PROCESS_FRAME, PROCESS_VIDEO, EXIT,
		PROCESS_N_FRAMES, PROCESS_SINGLE_STEP, PROCESS_STEPS_CONTINUOUS
	} mainLoopAction;

	struct UIColourMode {
		const char* name;
		ITMLib::ITMMainEngine::GetImageType type;

		UIColourMode(const char* _name, ITMLib::ITMMainEngine::GetImageType _type)
				: name(_name), type(_type) {}
	};
	std::vector<UIColourMode> colourModes_main, colourModes_freeview;
	UIColourMode colourMode_stepByStep = UIColourMode("step_by_step",
	                                                  ITMLib::ITMMainEngine::InfiniTAM_IMAGE_STEP_BY_STEP);
	int currentColourMode;

	int autoIntervalFrameStart;
	int autoIntervalFrameCount;
	bool saveAfterAutoprocessing = false;
	int startedProcessingFromFrameIx = 0;

	InputSource::ImageSourceEngine* imageSource;
	InputSource::IMUSourceEngine* imuSource;
	ITMLib::ITMMainEngine* mainEngine;

	StopWatchInterface* timer_instant;
	StopWatchInterface* timer_average;

	// For UI layout
	static const int NUM_WIN = 3;
	Vector4f winReg[NUM_WIN]; // (x1, y1, x2, y2)
	Vector2i winSize;
	uint textureId[NUM_WIN];
	ITMUChar4Image* outImage[NUM_WIN];
	ITMLib::ITMMainEngine::GetImageType outImageType[NUM_WIN];

	ITMUChar4Image* inputRGBImage;
	ITMShortImage* inputRawDepthImage;
	ITMLib::ITMIMUMeasurement* inputIMUMeasurement;

	bool freeviewActive;
	bool integrationActive;
	ORUtils::SE3Pose freeviewPose;
	ITMLib::ITMIntrinsics freeviewIntrinsics;

	int mouseState;
	Vector2i mouseLastClick;
	bool mouseWarped; // To avoid the extra motion generated by glutWarpPointer

	int currentFrameNo;
	bool isRecordingImages;
	bool inStepByStepMode;

	InputSource::FFMPEGWriter* reconstructionVideoWriter = nullptr;
	InputSource::FFMPEGWriter* rgbVideoWriter = nullptr;
	InputSource::FFMPEGWriter* depthVideoWriter = nullptr;
public:
	static UIEngine_BPO& Instance() {
		static UIEngine_BPO instance;
		return instance;
	}

	static void GlutDisplayFunction();
	static void GlutIdleFunction();
	static void GlutKeyUpFunction(unsigned char key, int x, int y);
	static void GlutMouseButtonFunction(int button, int state, int x, int y);
	static void GlutMouseMoveFunction(int x, int y);
	static void GlutMouseWheelFunction(int button, int dir, int x, int y);

	const Vector2i& GetWindowSize() const { return winSize; }

	float processedTime;
	int processedFrameNo;
	ITMLib::ITMTrackingState::TrackingResult trackingResult;
	char* outFolder;
	bool needsRefresh;

	bool allocateGPU;
	bool shutdownRequested = false;
	ITMUChar4Image* saveImage;
	ITMLib::ITMDynamicFusionLogger_Interface* logger;
	ITMLib::IndexingMethod indexingMethod;

	void Initialise(int& argc, char** argv, InputSource::ImageSourceEngine* imageSource, InputSource::IMUSourceEngine* imuSource,
		                ITMLib::ITMMainEngine* mainEngine, const char* outFolder, ITMLib::ITMLibSettings::DeviceType deviceType,
		                int frameIntervalLength, int skipFirstNFrames, bool recordReconstructionResult, bool startInStepByStep,
		                bool saveAfterFirstNFrames, bool loadBeforeProcessing, ITMLib::ITMDynamicFusionLogger_Interface* logger,
		                ITMLib::IndexingMethod indexingMethod);
	void Shutdown();

	void Run();
	void PrintProcessingFrameHeader() const;
	void ProcessFrame();

	//For scene-tracking updates
	bool BeginStepByStepMode();
	bool InStepByStepMode() { return this->inStepByStepMode;}
	bool ContinueStepByStepModeForFrame();

	void GetScreenshot(ITMUChar4Image* dest) const;
	void SaveScreenshot(const char* filename) const;

	void SkipFrames(int numberOfFramesToSkip);
	void RecordReconstructionToVideo();
	void RecordDepthAndRGBInputToVideo();
	void RecordDepthAndRGBInputToImages();
	int GetCurrentFrameIndex() const;
	std::string GenerateNextFrameOutputPath() const;
	std::string GenerateCurrentFrameOutputDirectory() const;
};
}
}
