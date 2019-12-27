//  ================================================================
//  Created by Gregory Kramida on 7/4/18.
//  Copyright (c) 2018-2025 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================

#include "CreateDefaultImageSource.h"

void
CreateDefaultImageSource(ImageSourceEngine*& imageSource, IMUSourceEngine*& imuSource, const InputPaths& inputPaths) {


	if (inputPaths.calibFilePath == "viewer") {
		imageSource = new BlankImageGenerator("", Vector2i(640, 480));
		printf("starting in viewer mode: make sure to press n first to initialize the views ... \n");
		return;
	}

	const char* calibFile = inputPaths.calibFilePath.empty() ? nullptr : inputPaths.calibFilePath.c_str();

	if (calibFile) {
		printf("using calibration file: %s\n", inputPaths.calibFilePath.c_str());
	} else {
		printf("using default calibration file.\n");
	}

	if (imageSource == nullptr && !inputPaths.rgbImageFileMask.empty() && !inputPaths.depthImageFileMask.empty()) {
		printf("using rgb images: %s\nusing depth images: %s\n", inputPaths.rgbImageFileMask.c_str(), inputPaths.depthImageFileMask.c_str());
		if (!inputPaths.maskImageFileMask.empty() && inputPaths.imuInputPath.empty()) {
			printf("using mask images: %s\n", inputPaths.maskImageFileMask.c_str());
		}
		if (inputPaths.imuInputPath.empty()) {
			ImageMaskPathGenerator pathGenerator(inputPaths.rgbImageFileMask.c_str(), inputPaths.depthImageFileMask.c_str(),
			                                     inputPaths.maskImageFileMask.empty() ? nullptr : inputPaths.maskImageFileMask.c_str());
			imageSource = new ImageFileReader<ImageMaskPathGenerator>(inputPaths.calibFilePath.c_str(), pathGenerator);
		} else {
			printf("using imu data: %s\n", inputPaths.imuInputPath.c_str());
			imageSource = new RawFileReader(inputPaths.calibFilePath.c_str(), inputPaths.rgbImageFileMask.c_str(),
			                                inputPaths.depthImageFileMask.c_str(), Vector2i(320, 240), 0.5f);
			imuSource = new IMUSourceEngine(inputPaths.imuInputPath.c_str());
		}
		int depthWidth = imageSource->getDepthImageSize().x;
		if (depthWidth == 0) {
			delete imageSource;
			if (imuSource != nullptr) delete imuSource;
			imuSource = nullptr;
			imageSource = nullptr;
		}
	}
	if ((imageSource == nullptr) && (!inputPaths.rgbVideoFilePath.empty() && !inputPaths.depthVideoFilePath.empty()) &&
	    (inputPaths.imuInputPath.empty())) {
		imageSource = new InputSource::FFMPEGReader(inputPaths.calibFilePath.c_str(), inputPaths.rgbVideoFilePath.c_str(),
		                                            inputPaths.depthVideoFilePath.c_str());
		if (imageSource->getDepthImageSize().x == 0) {
			delete imageSource;
			imageSource = nullptr;
		}
	}

	if (imageSource == nullptr) {
		// If no calibration file specified, use the factory default calibration
		bool useInternalCalibration = !calibFile || strlen(calibFile) == 0;

		printf("trying OpenNI device: %s - calibration: %s\n",
		       inputPaths.openniFilePath.empty() ? "<OpenNI default device>" : inputPaths.openniFilePath.c_str(),
		       useInternalCalibration ? "internal" : "from file");
		imageSource = new OpenNIEngine(calibFile, inputPaths.openniFilePath.empty() ? nullptr : inputPaths.openniFilePath.c_str(),
		                               useInternalCalibration);
		if (imageSource->getDepthImageSize().x == 0) {
			delete imageSource;
			imageSource = nullptr;
		}
	}

	if (imageSource == nullptr) {
		printf("trying UVC device\n");
		imageSource = new LibUVCEngine(calibFile);
		if (imageSource->getDepthImageSize().x == 0) {
			delete imageSource;
			imageSource = nullptr;
		}
	}

	if (imageSource == nullptr) {
		printf("trying RealSense device\n");
		imageSource = new RealSenseEngine(calibFile);
		if (imageSource->getDepthImageSize().x == 0) {
			delete imageSource;
			imageSource = nullptr;
		}
	}

	if (imageSource == nullptr) {
		printf("trying MS Kinect 2 device\n");
		imageSource = new Kinect2Engine(calibFile);
		if (imageSource->getDepthImageSize().x == 0) {
			delete imageSource;
			imageSource = nullptr;
		}
	}

	if (imageSource == nullptr) {
		printf("trying PMD PicoFlexx device\n");
		imageSource = new PicoFlexxEngine(calibFile);
		if (imageSource->getDepthImageSize().x == 0) {
			delete imageSource;
			imageSource = nullptr;
		}
	}
}
