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


//VTK
#include <vtkSmartPointer.h>
#include <vtkContextView.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkContextScene.h>
#include <vtkChartXY.h>

//ITMLib
#include "../../ITMLib/ITMLibDefines.h"
#include "../../ITMLib/Core/ITMBasicEngine.h"
#include "../../ITMLib/Core/ITMBasicSurfelEngine.h"
#include "../../ITMLib/Core/ITMMultiEngine.h"
#include "../../ITMLib/Core/ITMDynamicEngine.h"
#include "../../ITMLib/Utils/Visualization/ITMVisualizationWindowManager.h"
#include "../../ITMLib/Utils/Visualization/ITMVisualizationCommon.h"

//local
#include "UIEngine_BPO.h"
#include "prettyprint.hpp"
#include "CreateDefaultImageSource.h"


// *** namespaces ***

using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;

namespace po = boost::program_options;


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

int main(int argc, char** argv) {
	try {
		po::options_description arguments{"Arguments"};
		po::positional_options_description positional_arguments;

		// boolean flags
		// TODO: these should all pe part of the settings object, categorized by internal structs
		bool fixCamera = false;
		bool disableDataTerm = false;
		bool enableLevelSetTerm = false;
		bool disableSmoothingTerm = false;
		bool enableKillingTerm = false;
		bool disableGradientSmoothing = false;
		bool killingModeEnabled = false;

		bool recordReconstructionToVideo = false;

		bool saveAfterInitialProcessing = false;
		bool loadBeforeProcessing = false;

		bool startInStepByStep = false;

		bool recordCanonicalSceneAsSlices = false;
		bool recordLiveSceneAsSlices = false;
		bool record1DSlices = false;
		bool record2DSlices = false;
		bool record3DSlices = false;
		unsigned int _3DSliceRadius = 10;
		unsigned int _3DSliceExtraThicknessMargin = 0;
		bool record3DSceneAndWarps = false;
		bool plotEnergies = false;

		Plane planeFor2Dand3DSlices = PLANE_XY;


		//@formatter:off
		arguments.add_options()
				("help,h", "Print help screen")
				( "config,cfg", po::value<std::string>(),
				        "Configuration file in JSON format, e.g.  ./default_config.json "
			                      "WARNING: using this option will invalidate any other command line arguments.")
				("calib_file,c", po::value<std::string>(), "Full path to the calibration file, e.g.: ./Files/Teddy/calib.txt")

				("input_path,i", po::value<std::vector<std::string>>(), "Input files/paths. 0-3 total arguments. "
						"Usage scenarios:\n\n"
						"(0) No arguments: tries to get frames from attached device (OpenNI, "
	                        "RealSense, etc.). \n"
						"(1) One argument: tries to load OpenNI video at that location. \n"
						"(2) Two arguments (files): tries to load rgb & depth from two separate files when paths "
	                         "do NOT contain '%'.\n"
						"(3) Two arguments (masks): tries to "
	                        "load rgb & depth frames as single "
					        "images using the provided two "
		                    "arguments as file path masks "
				            "containing '%',\n"
						    "e.g.: ./Files/Teddy/Frames/%%04i.ppm ./Files/Teddy/Frames/%%04i.pgm\n"
						"(4) An extra mask argument (beyond 2) "
	                        "containing '%' will signify a file "
					        "mask to mask images."
						"(5) An extra path argument (beyond 2) "
	                    "NOT containing the '%' character "
					    "will signify a path to the IMU "
		                "file source.\n\n"
						"Currently, either (4) or (5) can only be combined with (3), but NOT both at the same time."
						"No other usage scenario takes them into account.\n"
				)
				("output,o", po::value<std::string>()->default_value("./Output"), "Output directory, e.g.: ./Output")

				("index", po::value<std::string>()->default_value("hash"), "Indexing method. May be one of [hash, array].")
				("device", po::value<std::string>()->default_value("CPU"), "Compute device. May be one of [CPU, CUDA]")

				("record_reconstruction_video", po::bool_switch(&recordReconstructionToVideo)->default_value(false),
				 "Whether to record the reconstruction rendering to video after each frame is processed.")
				("start_in_step_by_step_mode", po::bool_switch(&startInStepByStep)->default_value(false),
				 "Whether to start in step-by-step mode (dynamic fusion only).")



				("fix_camera", po::bool_switch(&fixCamera)->default_value(false),
				 "Whether or not to turn of the camera tracking (fix the virtual camera position)")



				/* Ranges for frame skipping or automated processing on launch */
				("process_N_frames,N", po::value<int>(), "Launch immediately and process the specified number of "
				 "frames (potentially, with recording, if corresponding commands are issued), and then stop.")
			    ("start_from_frame_ix,S", po::value<int>(), "Skip the first S frames / start at frame index S.\n")

				/* Automated loading / saving on particular frame */
				("save_after_initial_processing", po::bool_switch(&saveAfterInitialProcessing)->default_value(false),
				 "Save scene after the frames specified with process_N_frames were processed.")
				("load_before_processing", po::bool_switch(&loadBeforeProcessing)->default_value(false),
				 "Start by loading scene from disk before any processing takes place.")


				/* Visualization and logging for visual debugging of scene-tracking*/
				("focus_coordinates,f", po::value<std::vector<int>>()->multitoken(), "The coordinates of the voxel"
						" which to focus on for logging/debugging, as 3 integers separated by spaces, \"x y z\"."
	                    " When specified:\n"
						"    (1) Voxel-specific debug information will be printed about this voxel.\n"
	                    "    (2) The record-scene feature will work differently from standard behavior:"
					    "        it will, instead of recording the whole scene, record a small slice of voxels "
		                "        around this voxel."
				)
				("plot_energies",po::bool_switch(&plotEnergies)->default_value(false),
				 "Used in dynamic fusion. Plot graphs of energy contributions from all terms used during scene "
	             "tracking optimization.")
				("record_1d_slices",po::bool_switch(&record1DSlices)->default_value(false),
				 "Used in dynamic fusion. Plot graphs of canonical and live SDF (around the focus coordinate,"
	             " if provided), plot the live frame progression and warp vectors (for visual debugging).")
				("record_live_as_slices",po::bool_switch(&recordLiveSceneAsSlices)->default_value(false),
				 "Used in dynamic fusion. Slice up the whole live scene as images in each direction. ")
				("record_canonical_as_slices",po::bool_switch(&recordCanonicalSceneAsSlices)->default_value(false),
				 "Used in dynamic fusion. Slice up the whole canonical scene as images in each direction. ")
				("record_2d_slices",po::bool_switch(&record2DSlices)->default_value(false),
				 "Used in dynamic fusion. Render warps from each frame onto an image of the original live frame"
	             " (around the focus coordinate, if provided), as well as warped live frame"
			     " progression (for debugging).")
				("record_3d_slices",po::bool_switch(&record3DSlices)->default_value(false),
				 "Used in dynamic fusion. Visualize & record a 3D slice of the canonical scene with warp vectors"
	             " and the live scene as they evolve.")
				("3d_slice_radius",po::value<unsigned int>(&_3DSliceRadius)->default_value(10),
				 "(Dynamic fusion) half-width of the square of pixels (in plane) in the slice for 3d slice recording.")
				("3d_slice_margin",po::value<unsigned int>(&_3DSliceExtraThicknessMargin)->default_value(0),
				 "(Dynamic fusion) extra margin to include, in voxels, from the 3D slice center along the axis "
	             "perpendicular to the slice plane.")
				("slice_plane",po::value<ITMLib::Plane>(&planeFor2Dand3DSlices)->default_value(PLANE_XY),
				 "(Dynamic fusion) plane to use for recording of 2d slices.")
				("record_3d_scene_and_warps",po::bool_switch(&record3DSceneAndWarps)->default_value(false),
				 "Used in dynamic fusion. Record 3D scenes at each frame and complete warp progression at every iteration.")

				/*================================================================================*/
				/*=== ITMSceneMotionOptimizationParameters for scene tracking optimization (KillingFusion/SobolevFusion) ===*/
				/*================================================================================*/

				/* convergence parameters & learning rate*/
				("max_iterations", po::value<unsigned int>()->default_value(100),
				        "Maximum number of iterations in each frame of scene tracking optimization.")
				("vector_update_threshold", po::value<float>()->default_value(0.0001f),
					        "Unit: meters. Used in scene tracking optimization. Termination condition: optimization "
			     "stops when warp vector update lengths don't exceed this distance threshold.")
			    ("learning_rate", po::value<float>(),
					        "Used in scene tracking optimization. Gradient descent step magnitude / learning rate.")

				/* term weights / factors */
				("weight_data_term", po::value<float>()->default_value(1.0f),
				 "Used in scene tracking optimization when the data term is enabled."
				 " Greater values make the difference between canonical and live SDF grids induce greater warp updates.")
				("weight_smoothing_term", po::value<float>()->default_value(0.2f),
				 "Used in scene tracking optimization when the smoothness regularization term is enabled."
			         " Greater values penalize non-smooth scene deformations.")
				("rigidity_enforcement_factor", po::value<float>(),
				 "Used in scene tracking optimization when the Killing regularization term is enabled."
		         " Greater values penalize non-isometric scene deformations.")
				("weight_level_set_term", po::value<float>()->default_value(0.2f),
				 "Used in scene tracking optimization when the level set regularization term is enabled."
				 " Greater values penalize deformations resulting in non-SDF-like voxel grid.")

				/* Enable / disable dynamic fusion optimization terms / procedures */
				("disable_data_term", po::bool_switch(&disableDataTerm),
				 "Whether or not to disable the data term if using the DynamicFusion algorithm")
				("enable_level_set_term", po::bool_switch(&enableLevelSetTerm),
				 "Whether or not to disable the level set term if using the DynamicFusion algorithm")
				("disable_smoothing_term", po::bool_switch(&disableSmoothingTerm),
				 "Whether or not to disable the smoothness term if using the DynamicFusion algorithm")
				("enable_killing_term", po::bool_switch(&enableKillingTerm),
				 "Whether or not to enable the Killing term (isometric motion enforcement regularizer) if using the "
	             "dynamic scene fusion algorithm")
				("disable_gradient_smoothing", po::bool_switch(&disableGradientSmoothing),
				 "Whether or not to disable the Sobolev gradient smoothing if using the DynamicFusion algorithm\n")

				/* modes (parameter presets)*/
				("preset_mode", po::value<std::string>()->default_value("SobolevFusion"),
				 "Which default set of parameters to use for optimization. Individual parameters can still be overridden."
	             "For instance, passing --preset_mode KillingFusion will envoke the parameters from KillingFusion "
                 "(2017) article by Slavcheva et al., which would be equivalent to: \n"
	             "--disable_gradient_smoothing --enable_level_set_term --enable_killing_term "
			     "--rigidity_enforcement_factor 0.1 --weight_smoothness_term 0.5 --weight_level_set 0.2")
				;
		//@formatter:on


		positional_arguments.add("calib_file", 1);
		positional_arguments.add("input_path", 3);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).options(arguments).positional(positional_arguments).style(
				po::command_line_style::unix_style ^ po::command_line_style::allow_short).run(), vm);
		po::notify(vm);

		InputPaths inputPaths;

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

		if (vm["config"].empty()) {
			inputPaths = InputPaths::FromVariablesMap(vm, printHelp);
			Configuration::load_from_variable_map(vm);
		} else {
			std::string configPath = vm["config"].as<std::string>();
			inputPaths = InputPaths::FromJsonFile(configPath);
			Configuration::load_from_json_file(configPath);
		}

		printf("initialising ...\n");
		ImageSourceEngine* imageSource = nullptr;
		IMUSourceEngine* imuSource = nullptr;

		CreateDefaultImageSource(imageSource, imuSource, inputPaths);
		if (imageSource == nullptr) {
			std::cerr << "Failed to open any image stream." << std::endl;
			printHelp();
			return EXIT_FAILURE;
		}


// region ================================ GENERATE MAIN ENGINE ========================================================
		auto& settings = Configuration::get();
		Configuration::IndexingMethod chosenIndexingMethod = settings.indexing_method;
		ITMDynamicFusionLogger_Interface& logger = GetLogger(chosenIndexingMethod);
		ITMMainEngine* mainEngine = nullptr;


		switch (settings.library_mode) {
			case Configuration::LIBMODE_BASIC:
				switch (chosenIndexingMethod) {
					case Configuration::INDEX_HASH:
						mainEngine = new ITMBasicEngine<ITMVoxel, ITMVoxelBlockHash>(imageSource->getCalib(),
						                                                             imageSource->getRGBImageSize(),
						                                                             imageSource->getDepthImageSize());
						break;
					case Configuration::INDEX_ARRAY:
						mainEngine = new ITMBasicEngine<ITMVoxel, ITMPlainVoxelArray>(imageSource->getCalib(),
						                                                              imageSource->getRGBImageSize(),
						                                                              imageSource->getDepthImageSize());
						break;
				}
				break;
			case Configuration::LIBMODE_BASIC_SURFELS:
				mainEngine = new ITMBasicSurfelEngine<ITMSurfelT>(imageSource->getCalib(),
				                                                  imageSource->getRGBImageSize(),
				                                                  imageSource->getDepthImageSize());
				break;
			case Configuration::LIBMODE_LOOPCLOSURE:
				switch (chosenIndexingMethod) {
					case Configuration::INDEX_HASH:
						mainEngine = new ITMMultiEngine<ITMVoxel, ITMVoxelBlockHash>(imageSource->getCalib(),
						                                                             imageSource->getRGBImageSize(),
						                                                             imageSource->getDepthImageSize());
						break;
					case Configuration::INDEX_ARRAY:
						mainEngine = new ITMMultiEngine<ITMVoxel, ITMPlainVoxelArray>(imageSource->getCalib(),
						                                                              imageSource->getRGBImageSize(),
						                                                              imageSource->getDepthImageSize());
						break;
				}
				break;
			case Configuration::LIBMODE_DYNAMIC:
				switch (chosenIndexingMethod) {
					case Configuration::INDEX_HASH:
						mainEngine = new ITMDynamicEngine<ITMVoxel, ITMWarp, ITMVoxelBlockHash>(
								imageSource->getCalib(), imageSource->getRGBImageSize(),
								imageSource->getDepthImageSize());
						break;
					case Configuration::INDEX_ARRAY:
						mainEngine = new ITMDynamicEngine<ITMVoxel, ITMWarp, ITMPlainVoxelArray>(
								imageSource->getCalib(), imageSource->getRGBImageSize(),
								imageSource->getDepthImageSize());
						break;
				}
				break;
			default:
				throw std::runtime_error("Unsupported library mode!");
		}

		if (fixCamera) {
			std::cout << "fix_camera flag passed, automatically locking camera if possible "
			             "(attempting to disable tracking)." << std::endl;
			mainEngine->turnOffTracking();
		}
// endregion ===========================================================================================================
// region =========================== SET LOGGER / VISUALIZERS WITH CLI ARGUMENTS ======================================
		//NB: Logger's focus coordinates set above together with main engine settings, if provided
		if (plotEnergies) logger.TurnPlottingEnergiesOn();
		if (record3DSceneAndWarps) logger.TurnRecording3DSceneAndWarpProgressionOn();
		if (recordCanonicalSceneAsSlices) logger.TurnRecordingCanonicalSceneAs2DSlicesOn();
		if (recordLiveSceneAsSlices) logger.TurnRecordingLiveSceneAs2DSlicesOn();
		if (record1DSlices) logger.TurnRecordingScene1DSlicesWithUpdatesOn();
		if (record2DSlices) logger.TurnRecordingScene2DSlicesWithUpdatesOn();
		if (record3DSlices) logger.TurnRecordingScene3DSlicesWithUpdatesOn();
		if (settings.telemetry_settings.focus_coordinates_specified) {
			logger.SetFocusCoordinates(settings.telemetry_settings.focus_coordinates);
		}
		logger.SetOutputDirectory(settings.telemetry_settings.output_path);

		logger.SetPlaneFor2Dand3DSlices(planeFor2Dand3DSlices);
		logger.Set3DSliceInPlaneRadius(_3DSliceRadius);
		logger.Set3DSliceOutOfPlaneRadius(_3DSliceExtraThicknessMargin);

// endregion
// region =========================== SET UI ENGINE SETTINGS WITH CLI ARGUMENTS ========================================
		int processNFramesOnLaunch = 0;
		if (!vm["process_N_frames"].empty()) {
			processNFramesOnLaunch = vm["process_N_frames"].as<int>();
		}

		int skipFirstNFrames = 0;
		if (!vm["start_from_frame_ix"].empty()) {
			skipFirstNFrames = vm["start_from_frame_ix"].as<int>();
		}

		//TODO (see top of file)
#ifndef WIN32
		XInitThreads();
#endif
		UIEngine_BPO::Instance().Initialise(argc, argv, imageSource, imuSource, mainEngine,
		                                    settings.telemetry_settings.output_path.c_str(), settings.device_type,
		                                    processNFramesOnLaunch, skipFirstNFrames, recordReconstructionToVideo,
		                                    startInStepByStep, saveAfterInitialProcessing, loadBeforeProcessing,
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

