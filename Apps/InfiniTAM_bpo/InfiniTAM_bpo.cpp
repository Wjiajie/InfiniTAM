// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

//TODO: not supported on platforms besides Linux, adjust via CMake -Greg(GitHub: Algomorph)
#ifndef WIN32
#include <X11/Xlib.h>
#endif

//stdlib
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#define BOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE
//boost
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

//VTK
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkContextScene.h>

//ITMLib
#include "../../ITMLib/ITMLibDefines.h"
#include "../../ITMLib/Engines/Main/ITMBasicEngine.h"
#include "../../ITMLib/Engines/Main/ITMBasicSurfelEngine.h"
#include "../../ITMLib/Engines/Main/ITMMultiEngine.h"
#include "../../ITMLib/Engines/Main/ITMDynamicEngine.h"
#include "../../ITMLib/Utils/Visualization/ITMVisualizationWindowManager.h"
#include "../../ITMLib/Engines/Main/MianEngineFactory.h"

//local
#include "UIEngine_BPO.h"
#include "prettyprint.hpp"
#include "CreateDefaultImageSource.h"
#include "ProgramOptions.h"

// *** namespaces ***

using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;

namespace po = boost::program_options;
namespace pt = boost::property_tree;


ITMDynamicFusionLogger_Interface& GetLogger(Configuration::IndexingMethod method) {
	switch (method) {
		case Configuration::INDEX_HASH: {
			return static_cast<ITMDynamicFusionLogger_Interface&>(ITMDynamicFusionLogger<ITMVoxel, ITMWarp, ITMVoxelBlockHash>::Instance());
		}
		case Configuration::INDEX_ARRAY: {
			return static_cast<ITMDynamicFusionLogger_Interface&>(ITMDynamicFusionLogger<ITMVoxel, ITMWarp, ITMPlainVoxelArray>::Instance());
		}
	}
};

void process_UI_options_CLI(int& processNFramesOnLaunch, int& skipFirstNFrames, const po::variables_map& vm){
	if (!vm["process_N_frames"].empty()) {
		processNFramesOnLaunch = vm["process_N_frames"].as<int>();
	}
	if (!vm["start_from_frame_ix"].empty()) {
		skipFirstNFrames = vm["start_from_frame_ix"].as<int>();
	}
}

void process_UI_options_JSON(int& processNFramesOnLaunch, int& skipFirstNFrames, std::string config_path) {
	pt::ptree tree;
	pt::read_json(config_path, tree);
	boost::optional<int> processNFramesOnLaunch_opt = tree.get_optional<int>("input.process_N_frames");
	boost::optional<int> skipFirstNFrames_opt = tree.get_optional<int>("input.start_from_frame_ix");
	if (processNFramesOnLaunch_opt) {
		processNFramesOnLaunch = processNFramesOnLaunch_opt.get();
	}
	if (skipFirstNFrames_opt) {
		skipFirstNFrames = skipFirstNFrames_opt.get();
	}
}

int main(int argc, char** argv) {
	try {
		po::options_description arguments{"Arguments"};
		po::positional_options_description positional_arguments;
		RunOptions runOptions;
		LoggingOptions loggingOptions;


		PopulateOptionsDescription(arguments,runOptions,loggingOptions);


		positional_arguments.add("calibration_file", 1);
		positional_arguments.add("input_path", 3);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).options(arguments).positional(positional_arguments).style(
				po::command_line_style::unix_style ^ po::command_line_style::allow_short).run(), vm);
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

		if (vm.count("halp")) {
			printf("Ya didn't think it would work, did ya now?\n");
			printHelp();
			return EXIT_SUCCESS;
		}

		int processNFramesOnLaunch = 0;
		int skipFirstNFrames = 0;
		if (vm["config"].empty()) {
			process_UI_options_CLI(processNFramesOnLaunch, skipFirstNFrames, vm);
			Configuration::load_configuration_from_variable_map(vm);
		} else {
			std::string configPath = vm["config"].as<std::string>();
			process_UI_options_JSON(processNFramesOnLaunch, skipFirstNFrames, configPath);
			Configuration::load_configuration_from_json_file(configPath);
		}
		auto& settings = Configuration::get();

		printf("initialising ...\n");
		ImageSourceEngine* imageSource = nullptr;
		IMUSourceEngine* imuSource = nullptr;

		CreateDefaultImageSource(imageSource, imuSource, settings.input_and_output_settings);
		if (imageSource == nullptr) {
			std::cerr << "Failed to open any image stream." << std::endl;
			printHelp();
			return EXIT_FAILURE;
		}

// region ================================ BUILD MAIN ENGINE ========================================================
		Configuration::IndexingMethod chosenIndexingMethod = settings.indexing_method;
		ITMDynamicFusionLogger_Interface& logger = GetLogger(chosenIndexingMethod);
		ITMMainEngine* mainEngine = BuildMainEngine(imageSource->getCalib(),
		                                            imageSource->getRGBImageSize(),
		                                            imageSource->getDepthImageSize(),
		                                            runOptions.fixCamera);

// endregion ===========================================================================================================
// region =========================== SET LOGGER / VISUALIZERS WITH CLI ARGUMENTS ======================================
		//NB: Logger's focus coordinates set above together with main engine settings, if provided
		if (loggingOptions.plotEnergies) logger.TurnPlottingEnergiesOn();
		if (loggingOptions.record3DSceneAndWarps) logger.TurnRecording3DSceneAndWarpProgressionOn();
		if (loggingOptions.recordCanonicalSceneAsSlices) logger.TurnRecordingCanonicalSceneAs2DSlicesOn();
		if (loggingOptions.recordLiveSceneAsSlices) logger.TurnRecordingLiveSceneAs2DSlicesOn();
		if (loggingOptions.record1DSlices) logger.TurnRecordingScene1DSlicesWithUpdatesOn();
		if (loggingOptions.record2DSlices) logger.TurnRecordingScene2DSlicesWithUpdatesOn();
		if (loggingOptions.record3DSlices) logger.TurnRecordingScene3DSlicesWithUpdatesOn();
		if (settings.telemetry_settings.focus_coordinates_specified) {
			logger.SetFocusCoordinates(settings.telemetry_settings.focus_coordinates);
		}
		logger.SetOutputDirectory(settings.input_and_output_settings.output_path);

		logger.SetPlaneFor2Dand3DSlices(loggingOptions.planeFor2Dand3DSlices);
		logger.Set3DSliceInPlaneRadius(loggingOptions._3DSliceRadius);
		logger.Set3DSliceOutOfPlaneRadius(loggingOptions._3DSliceExtraThicknessMargin);

// endregion
// region =========================== SET UI ENGINE SETTINGS WITH CLI ARGUMENTS ========================================


		//TODO (see top of file)
#ifndef WIN32
		XInitThreads();
#endif
		UIEngine_BPO::Instance().Initialize(argc, argv, imageSource, imuSource, mainEngine,
		                                    settings.input_and_output_settings.output_path.c_str(), settings.device_type,
		                                    processNFramesOnLaunch, skipFirstNFrames, runOptions,
		                                    &logger, chosenIndexingMethod);


// endregion ===========================================================================================================

		//ITMVisualizationWindowManager::get().Run();
		UIEngine_BPO::Instance().Run();
		UIEngine_BPO::Instance().Shutdown();

// region ========================================= CLEANUP ============================================================

		delete mainEngine;
		delete imageSource;
		delete imuSource;

		//ITMVisualizationWindowManager::get().ShutDown();
// endregion ===========================================================================================================
		return EXIT_SUCCESS;
	} catch (std::exception& e) {
		std::cerr << e.what() << '\n';
		return EXIT_FAILURE;
	}
}

