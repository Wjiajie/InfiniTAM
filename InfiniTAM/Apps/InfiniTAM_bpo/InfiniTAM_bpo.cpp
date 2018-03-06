// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

//stdlib
#include <cstdlib>
#include <iostream>
#include <string>

//ITMLib
#include "UIEngine.h"

#include "../../InputSource/OpenNIEngine.h"
#include "../../InputSource/Kinect2Engine.h"
#include "../../InputSource/LibUVCEngine.h"
#include "../../InputSource/PicoFlexxEngine.h"
#include "../../InputSource/RealSenseEngine.h"
#include "../../InputSource/LibUVCEngine.h"
#include "../../InputSource/RealSenseEngine.h"
#include "../../InputSource/FFMPEGReader.h"
#include "../../ITMLib/ITMLibDefines.h"
#include "../../ITMLib/Core/ITMBasicEngine.h"
#include "../../ITMLib/Core/ITMBasicSurfelEngine.h"
#include "../../ITMLib/Core/ITMMultiEngine.h"
#include "../../ITMLib/Core/ITMDynamicEngine.h"

//boost
#include <boost/program_options.hpp>

//local
#include "prettyprint.hpp"

// *** namespaces ***

using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;

namespace po = boost::program_options;

/** Create a default source of depth images from a list of command line
    arguments. Typically, @para arg1 would identify the calibration file to
    use, @para arg2 the colour images, @para arg3 the depth images and
    @para arg4 the IMU images. If images are omitted, some live sources will
    be tried.
*/
static void CreateDefaultImageSource(ImageSourceEngine*& imageSource, IMUSourceEngine*& imuSource,
                                     const std::string& calibFilePath = "",
                                     const std::string& openniFilePath = "",
                                     const std::string& rgbVideoFilePath = "",
                                     const std::string& depthVideoFilePath = "",
                                     const std::string& rgbImageFileMask = "",
                                     const std::string& depthImageFileMask = "",
                                     const std::string& maskImageFileMask = "",
                                     const std::string& imuInputPath = "") {


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

		if (imageSource->getDepthImageSize().x == 0) {
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

bool isPathMask(std::string arg) {
	return arg.find("%") != std::string::npos;
}

int main(int argc, char** argv) {
	try {
		po::options_description arguments{"Arguments"};
		po::positional_options_description positional_arguments;
		arguments.add_options()
				("help,h", "Print help screen")
				("calib_file,c", po::value<std::string>(), "Calibration file, e.g.: ./Files/Teddy/calib.txt")
				("input_file,i", po::value<std::vector<std::string>>(), "Input file. 0-3 total arguments. "
						"Usage scenarios:\n"
						" (0) No arguments: tries to get frames from attached device (OpenNI, RealSense, etc.). \n"
						" (1) One argument: tries to load OpenNI video at that location. \n"
						" (2) Two arguments (files): tries to load rgb & depth from two separate video files "
						"file path masks NOT containing '%'.\n"
						" (3) Two arguments (masks): tries to load rgb & depth frames as single images using the "
						"provided two arguments as file path masks containing '%', "
						"e.g.: ./Files/Teddy/Frames/%%04i.ppm ./Files/Teddy/Frames/%%04i.pgm\n"
						" (4) An extra mask argument (beyond 2) containing '%' will signify a file mask to mask images. \n"
						" (5) An extra path argument (beyond 2) NOT containing the '%' character will signify a "
						" path to the IMU file source.\n "
						"Currently, either (4) or (5) can only be combined with (3), but NOT both at the same time."
						" No other usage scenario takes them into account."
				)
				("output,o", po::value<std::string>()->default_value("./State/"), "Output directory, e.g.: ./State/");
		positional_arguments.add("calib_file", 1);
		positional_arguments.add("input_file", 3);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).options(arguments).positional(positional_arguments).run(), vm);
		po::notify(vm);

		if (vm.count("help")) {
			std::cout << arguments << std::endl;
			std::cout << "Positional arguments: " << std::endl;
			std::cout << "   --" << positional_arguments.name_for_position(0) << std::endl;
			std::cout << "   --" << positional_arguments.name_for_position(1) << std::endl;
			printf("examples:\n"
					       "  %s ./Files/Teddy/calib.txt ./Files/Teddy/Frames/%%04i.ppm ./Files/Teddy/Frames/%%04i.pgm\n"
					       "  %s ./Files/Teddy/calib.txt\n\n", argv[0], argv[0]);
			return EXIT_SUCCESS;
		}

		std::string calibFilePath;
		if (vm.count("calib_file")) {
			calibFilePath = vm["calib_file"].as<std::string>();
		}
		//all initialized to empty string by default
		std::string openniFilePath, rgbVideoFilePath, depthVideoFilePath, rgbImageFileMask, depthImageFileMask,
				maskImageFileMask, imuInputPath;
		int inputFileCount = static_cast<int>(vm.count("input_file"));
		std::vector<std::string> inputFiles = vm["input_file"].as<std::vector<std::string>>();
		switch (inputFileCount) {
			case 0:
				//no input files
				break;
			case 1:
				//a single OpenNI file
				openniFilePath = inputFiles[0];
				break;
			case 3:
				if (isPathMask(inputFiles[2])) { maskImageFileMask = inputFiles[2]; }
				else { imuInputPath = inputFiles[2]; }
			case 2:
			default:
				if (isPathMask(inputFiles[1]) && isPathMask(inputFiles[2])) {
					rgbImageFileMask = inputFiles[0];
					depthImageFileMask = inputFiles[1];
				} else if (!isPathMask(inputFiles[1]) && !isPathMask(inputFiles[2])) {
					rgbVideoFilePath = inputFiles[0];
					depthVideoFilePath = inputFiles[1];
				} else {
					DIEWITHEXCEPTION(
							"The first & second input_file arguments need to either both be masks or both be paths to video files.");
				}
				break;
		}

		printf("initialising ...\n");
		ImageSourceEngine* imageSource = nullptr;
		IMUSourceEngine* imuSource = nullptr;

		CreateDefaultImageSource(imageSource, imuSource, calibFilePath, openniFilePath, rgbVideoFilePath,
		                         depthVideoFilePath, rgbImageFileMask, depthImageFileMask, maskImageFileMask,
		                         imuInputPath);
		if (imageSource == nullptr) {
			std::cout << "failed to open any image stream" << std::endl;
			return EXIT_FAILURE;
		}

		auto* internalSettings = new ITMLibSettings();
		internalSettings->outputPath = vm["output"].as<std::string>().c_str();

		ITMMainEngine* mainEngine = nullptr;
		switch (internalSettings->libMode) {
			case ITMLibSettings::LIBMODE_BASIC:
				mainEngine = new ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(internalSettings, imageSource->getCalib(),
				                                                         imageSource->getRGBImageSize(),
				                                                         imageSource->getDepthImageSize());
				break;
			case ITMLibSettings::LIBMODE_BASIC_SURFELS:
				mainEngine = new ITMBasicSurfelEngine<ITMSurfelT>(internalSettings, imageSource->getCalib(),
				                                                  imageSource->getRGBImageSize(),
				                                                  imageSource->getDepthImageSize());
				break;
			case ITMLibSettings::LIBMODE_LOOPCLOSURE:
				mainEngine = new ITMMultiEngine<ITMVoxel, ITMVoxelIndex>(internalSettings, imageSource->getCalib(),
				                                                         imageSource->getRGBImageSize(),
				                                                         imageSource->getDepthImageSize());
				break;
			case ITMLibSettings::LIBMODE_DYNAMIC:
				mainEngine = new ITMDynamicEngine<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>(internalSettings,
				                                                                                  imageSource->getCalib(),
				                                                                                  imageSource->getRGBImageSize(),
				                                                                                  imageSource->getDepthImageSize());
				break;
			default:
				throw std::runtime_error("Unsupported library mode!");
				break;
		}

		UIEngine::Instance()->Initialise(argc, argv, imageSource, imuSource, mainEngine, "./Files/Out",
		                                 internalSettings->deviceType);
		UIEngine::Instance()->Run();
		UIEngine::Instance()->Shutdown();

		delete mainEngine;
		delete internalSettings;
		delete imageSource;
		delete imuSource;
		return EXIT_SUCCESS;
	} catch (std::exception& e) {
		std::cerr << e.what() << '\n';
		return EXIT_FAILURE;
	}
}

