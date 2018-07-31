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
CreateDefaultImageSource(ImageSourceEngine*& imageSource, IMUSourceEngine*& imuSource, const std::string& calibFilePath,
                         const std::string& openniFilePath, const std::string& rgbVideoFilePath,
                         const std::string& depthVideoFilePath, const std::string& rgbImageFileMask,
                         const std::string& depthImageFileMask, const std::string& maskImageFileMask,
                         const std::string& imuInputPath) {


	if (calibFilePath == "viewer") {
		imageSource = new BlankImageGenerator("", Vector2i(640, 480));
		printf("starting in viewer mode: make sure to press n first to initialize the views ... \n");
		return;
	}

	const char* calibFile = calibFilePath.empty() ? nullptr : calibFilePath.c_str();

	if (calibFile) {
		printf("using calibration file: %s\n", calibFilePath.c_str());
	} else {
		printf("using default calibration file.\n");
	}

	if (imageSource == nullptr && !rgbImageFileMask.empty() && !depthImageFileMask.empty()) {
		printf("using rgb images: %s\nusing depth images: %s\n", rgbImageFileMask.c_str(), depthImageFileMask.c_str());
		if (!maskImageFileMask.empty() && imuInputPath.empty()) {
			printf("using mask images: %s\n", maskImageFileMask.c_str());
		}
		if (imuInputPath.empty()) {
			ImageMaskPathGenerator pathGenerator(rgbImageFileMask.c_str(), depthImageFileMask.c_str(),
			                                     maskImageFileMask.empty() ? nullptr : maskImageFileMask.c_str());
			imageSource = new ImageFileReader<ImageMaskPathGenerator>(calibFilePath.c_str(), pathGenerator);
		} else {
			printf("using imu data: %s\n", imuInputPath.c_str());
			imageSource = new RawFileReader(calibFilePath.c_str(), rgbImageFileMask.c_str(),
			                                depthImageFileMask.c_str(), Vector2i(320, 240), 0.5f);
			imuSource = new IMUSourceEngine(imuInputPath.c_str());
		}
		int depthWidth = imageSource->getDepthImageSize().x;
		if (depthWidth == 0) {
			delete imageSource;
			if (imuSource != nullptr) delete imuSource;
			imuSource = nullptr;
			imageSource = nullptr;
		}
	}
	if ((imageSource == nullptr) && (!rgbVideoFilePath.empty() && !depthVideoFilePath.empty()) &&
	    (imuInputPath.empty())) {
		imageSource = new InputSource::FFMPEGReader(calibFilePath.c_str(), rgbVideoFilePath.c_str(),
		                                            depthVideoFilePath.c_str());
		if (imageSource->getDepthImageSize().x == 0) {
			delete imageSource;
			imageSource = nullptr;
		}
	}

	if (imageSource == nullptr) {
		// If no calibration file specified, use the factory default calibration
		bool useInternalCalibration = !calibFile || strlen(calibFile) == 0;

		printf("trying OpenNI device: %s - calibration: %s\n",
		       openniFilePath.empty() ? "<OpenNI default device>" : openniFilePath.c_str(),
		       useInternalCalibration ? "internal" : "from file");
		imageSource = new OpenNIEngine(calibFile, openniFilePath.empty() ? nullptr : openniFilePath.c_str(),
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
