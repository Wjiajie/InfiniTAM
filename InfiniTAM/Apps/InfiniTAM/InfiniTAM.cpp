// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include <cstdlib>
#include <iostream>

#include "UIEngine.h"
#include "CreateDefaultImageSource.h"

#include "../../ITMLib/ITMLibDefines.h"

#include "../../ITMLib/Core/ITMBasicEngine.h"
#include "../../ITMLib/Core/ITMBasicSurfelEngine.h"
#include "../../ITMLib/Core/ITMMultiEngine.h"
#include "../../ITMLib/Core/ITMDynamicEngine.h"

using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;


int main(int argc, char** argv)
try
{
	const char *arg1 = "";
	const char *arg2 = NULL;
	const char *arg3 = NULL;
	const char *arg4 = NULL;
	const char *arg5 = NULL;

	int arg = 1;
	do {
		if (argv[arg] != NULL) arg1 = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) arg2 = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) arg3 = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) arg4 = argv[arg]; else break;
		++arg;
		if (argv[arg] != NULL) arg5 = argv[arg]; else break;
	} while (false);

	if (arg == 1) {
		printf("usage: %s [<calibfile> [<imagesource>] ]\n"
		       "  <calibfile>   : path to a file containing intrinsic calibration parameters\n"
		       "  <imagesource> : either one argument to specify OpenNI device ID\n"
		       "                  or two arguments specifying rgb and depth file masks \n"
			   "                  or three arguments specifying rgb, depth, and mask file masks \n"
		       "\n"
		       "examples:\n"
		       "  %s ./Files/Teddy/calib.txt ./Files/Teddy/Frames/%%04i.ppm ./Files/Teddy/Frames/%%04i.pgm\n"
		       "  %s ./Files/Teddy/calib.txt\n\n", argv[0], argv[0], argv[0]);
	}

	printf("initialising ...\n");
	ImageSourceEngine *imageSource = NULL;
	IMUSourceEngine *imuSource = NULL;

	CreateDefaultImageSource(imageSource, imuSource, arg1, arg2, arg3, arg4, arg5);
	if (imageSource==NULL)
	{
		std::cout << "failed to open any image stream" << std::endl;
		return -1;
	}

	Configuration *internalSettings = &Configuration::Instance();

	ITMMainEngine *mainEngine = NULL;
	switch (internalSettings->library_mode)
	{
	case Configuration::LIBMODE_BASIC:
		mainEngine = new ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(imageSource->getCalib(),
		                                                         imageSource->getRGBImageSize(),
		                                                         imageSource->getDepthImageSize());
		break;
	case Configuration::LIBMODE_BASIC_SURFELS:
		mainEngine = new ITMBasicSurfelEngine<ITMSurfelT>(imageSource->getCalib(), imageSource->getRGBImageSize(),
		                                                  imageSource->getDepthImageSize());
		break;
	case Configuration::LIBMODE_LOOPCLOSURE:
		mainEngine = new ITMMultiEngine<ITMVoxel, ITMVoxelIndex>(imageSource->getCalib(),
		                                                         imageSource->getRGBImageSize(),
		                                                         imageSource->getDepthImageSize());
		break;
	case Configuration::LIBMODE_DYNAMIC:
		mainEngine = new ITMDynamicEngine<ITMVoxel, ITMWarp, ITMVoxelIndex>(imageSource->getCalib(),
		                                                                     imageSource->getRGBImageSize(),
		                                                                     imageSource->getDepthImageSize());
		break;
	default: 
		throw std::runtime_error("Unsupported library mode!");
		break;
	}

	UIEngine::Instance()->Initialise(argc, argv, imageSource, imuSource, mainEngine, "./Files/Out", internalSettings->deviceType);
	UIEngine::Instance()->Run();
	UIEngine::Instance()->Shutdown();

	delete mainEngine;
	delete internalSettings;
	delete imageSource;
	if (imuSource != NULL) delete imuSource;
	return 0;
}
catch(std::exception& e)
{
	std::cerr << e.what() << '\n';
	return EXIT_FAILURE;
}

