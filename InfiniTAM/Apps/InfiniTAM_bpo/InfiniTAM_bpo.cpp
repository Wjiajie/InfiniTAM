// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

//stdlib
#include <cstdlib>
#include <iostream>
#include <string>

//ITMLib
#include "UIEngine_BPO.h"

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

bool isPathMask(const std::string& arg) {
	return arg.find('%') != std::string::npos;
}

int main(int argc, char** argv) {
	try {
		po::options_description arguments{"Arguments"};
		po::positional_options_description positional_arguments;
		bool fixCamera = false;
		//@formatter:off
		arguments.add_options()
				("help,h", "Print help screen")
				("calib_file,c", po::value<std::string>(), "Calibration file, e.g.: ./Files/Teddy/calib.txt")
				("input_file,i", po::value<std::vector<std::string>>(), "Input file. 0-3 total arguments. "
						"Usage scenarios:\n"
						"    (0) No arguments: tries to get frames from attached device (OpenNI, RealSense, etc.). \n"
						"    (1) One argument: tries to load OpenNI video at that location. \n"
						"    (2) Two arguments (files): tries to load rgb & depth from two separate video files "
						"    file path masks NOT containing '%'.\n"
						"    (3) Two arguments (masks): tries to load rgb & depth frames as single images using the "
						"        provided two arguments as file path masks containing '%', "
						"        e.g.: ./Files/Teddy/Frames/%%04i.ppm ./Files/Teddy/Frames/%%04i.pgm\n"
						"    (4) An extra mask argument (beyond 2) containing '%' will signify a file mask to mask images. \n"
						"    (5) An extra path argument (beyond 2) NOT containing the '%' character will signify a "
						"        path to the IMU file source.\n "
						"    Currently, either (4) or (5) can only be combined with (3), but NOT both at the same time."
						"    No other usage scenario takes them into account."
				)
				("focus_coordinates,f", po::value<std::vector<int>>()->multitoken(), "The coordinates of the voxel"
						" which to focus on for logging/debugging, as 3 integers separated by spaces, \"x y z\"."
	                    " When specified:\n"
						"    (1) Voxel-specific debug information will be printed about this voxel.\n"
	                    "    (2) The record-scene feature will work differently from standard behavior:"
					    "        it will, instead of recording the whole scene, record a small slice of voxels "
		                "        around this voxel."
				)
				("output,o", po::value<std::string>()->default_value("./Output"), "Output directory, e.g.: ./Output")
				("fix_camera", po::bool_switch(&fixCamera)->default_value(false));
		//@formatter:on
		positional_arguments.add("calib_file", 1);
		positional_arguments.add("input_file", 3);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).options(arguments).positional(positional_arguments).style(po::command_line_style::unix_style ^ po::command_line_style::allow_short).run(), vm);
		po::notify(vm);

		auto printHelp = [&arguments, &positional_arguments, &argv]() {
			std::cout << arguments << std::endl;
			std::cout << "Positional arguments: " << std::endl;
			std::cout << "   --" << positional_arguments.name_for_position(0) << std::endl;
			std::cout << "   --" << positional_arguments.name_for_position(1) << std::endl;
			printf("examples:\n"
			       "  %s ./Files/Teddy/calib.txt ./Files/Teddy/Frames/%%04i.ppm ./Files/Teddy/Frames/%%04i.pgm\n"
			       "  %s ./Files/Teddy/calib.txt\n\n", argv[0], argv[0]);
		};

		if (vm.count("help")) {
			printHelp();
			return EXIT_SUCCESS;
		}


		std::string calibFilePath;
		if (vm.count("calib_file")) {
			calibFilePath = vm["calib_file"].as<std::string>();
		}


		//all initialized to empty string by default
		std::string openniFilePath, rgbVideoFilePath, depthVideoFilePath, rgbImageFileMask, depthImageFileMask,
				maskImageFileMask, imuInputPath;
		std::vector<std::string> inputFiles = vm["input_file"].as<std::vector<std::string>>();
		auto inputFileCount = inputFiles.size();
		switch (inputFileCount) {
			case 0:
				//no input files
				break;
			case 1:
				//a single OpenNI file
				openniFilePath = inputFiles[0];
				break;
			case 3:
			default:
				if (isPathMask(inputFiles[2])) { maskImageFileMask = inputFiles[2]; }
				else { imuInputPath = inputFiles[2]; }
			case 2:
				if (isPathMask(inputFiles[0]) && isPathMask(inputFiles[1])) {
					rgbImageFileMask = inputFiles[0];
					depthImageFileMask = inputFiles[1];
				} else if (!isPathMask(inputFiles[0]) && !isPathMask(inputFiles[1])) {
					rgbVideoFilePath = inputFiles[0];
					depthVideoFilePath = inputFiles[1];
				} else {
					std::cerr << "The first & second input_file arguments need to either both be masks or both be"
					             " paths to video files." << std::endl;
					printHelp();
					return EXIT_FAILURE;

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
			std::cerr << "Failed to open any image stream." << std::endl;
			printHelp();
			return EXIT_FAILURE;
		}

		auto* settings = new ITMLibSettings();
		settings->outputPath = vm["output"].as<std::string>().c_str();
		bool haveFocusCoordinate = !vm["focus_coordinates"].empty();
		Vector3i focusCoordiantes(0);
		if (haveFocusCoordinate) {
			std::vector<int> focusCoordsVec = vm["focus_coordinates"].as<std::vector<int> >();
			if (focusCoordsVec.size() != 3) {
				std::cerr << "Could not parse focus coordiantes vector as exactly 3 integers, \"x y z\"" << std::endl;
				printHelp();
				return EXIT_FAILURE;
			}
			memcpy(focusCoordiantes.values, focusCoordsVec.data(), sizeof(int) * 3);
		}
		settings->SetFocusCoordinates(focusCoordiantes);

		ITMMainEngine* mainEngine = nullptr;
		switch (settings->libMode) {
			case ITMLibSettings::LIBMODE_BASIC:
				mainEngine = new ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(settings, imageSource->getCalib(),
				                                                         imageSource->getRGBImageSize(),
				                                                         imageSource->getDepthImageSize());
				break;
			case ITMLibSettings::LIBMODE_BASIC_SURFELS:
				mainEngine = new ITMBasicSurfelEngine<ITMSurfelT>(settings, imageSource->getCalib(),
				                                                  imageSource->getRGBImageSize(),
				                                                  imageSource->getDepthImageSize());
				break;
			case ITMLibSettings::LIBMODE_LOOPCLOSURE:
				mainEngine = new ITMMultiEngine<ITMVoxel, ITMVoxelIndex>(settings, imageSource->getCalib(),
				                                                         imageSource->getRGBImageSize(),
				                                                         imageSource->getDepthImageSize());
				break;
			case ITMLibSettings::LIBMODE_DYNAMIC:
				mainEngine = new ITMDynamicEngine<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>(
						settings, imageSource->getCalib(), imageSource->getRGBImageSize(),
						imageSource->getDepthImageSize());
				break;
			default:
				throw std::runtime_error("Unsupported library mode!");
				break;
		}
		if (fixCamera) {
			std::cout << "fix_camera flag passed, automatically locking camera if possible "
			             "(attempting to disable tracking)." << std::endl;
			mainEngine->turnOffTracking();
		}

		UIEngine_BPO::Instance()->Initialise(argc, argv, imageSource, imuSource, mainEngine,
		                                     settings->outputPath,
		                                     settings->deviceType);
		UIEngine_BPO::Instance()->Run();
		UIEngine_BPO::Instance()->Shutdown();

		delete mainEngine;
		delete settings;
		delete imageSource;
		delete imuSource;
		return EXIT_SUCCESS;
	} catch (std::exception& e) {
		std::cerr << e.what() << '\n';
		return EXIT_FAILURE;
	}
}

