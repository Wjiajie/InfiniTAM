//  ================================================================
//  Created by Gregory Kramida on 12/27/19.
//  Copyright (c)  2019 Gregory Kramida
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

#include "ProgramOptions.h"

void PopulateOptionsDescription(po::options_description& arguments, RunOptions& runOptions, LoggingOptions& loggingOptions){
	//@formatter:off
	arguments.add_options()
			("help,h", "Print help screen")
			("halp,h", "Funkaaay")
			( "config,cfg", po::value<std::string>(),
			  "Configuration file in JSON format, e.g.  ./default_config_cuda.json "
			  "WARNING: using this option will invalidate any other command line arguments.")
			("calibration_file,c", po::value<std::string>(), "Full path to the calibration file, e.g.: ./Files/Teddy/calib.txt")

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

			("record_reconstruction_video", po::bool_switch(&runOptions.recordReconstructionToVideo)->default_value(false),
			 "Whether to record the reconstruction rendering to video after each frame is processed.")
			("start_in_step_by_step_mode", po::bool_switch(&runOptions.startInStepByStep)->default_value(false),
			 "Whether to start in step-by-step mode (dynamic fusion only).")



			("fix_camera", po::bool_switch(&runOptions.fixCamera)->default_value(false),
			 "Whether or not to turn of the camera tracking (fix the virtual camera position)")



			/* Ranges for frame skipping or automated processing on launch */
			("process_N_frames,N", po::value<int>(), "Launch immediately and process the specified number of "
			                                         "frames (potentially, with recording, if corresponding commands are issued), and then stop.")
			("start_from_frame_ix,S", po::value<int>(), "Skip the first S frames / start at frame index S.\n")

			/* Automated loading / saving on particular frame */
			("save_after_initial_processing", po::bool_switch(&runOptions.saveAfterInitialProcessing)->default_value(false),
			 "Save scene after the frames specified with process_N_frames were processed.")
			("load_before_processing", po::bool_switch(&runOptions.loadVolumeBeforeProcessing)->default_value(false),
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
			("plot_energies",po::bool_switch(&loggingOptions.plotEnergies)->default_value(false),
			 "Used in dynamic fusion. Plot graphs of energy contributions from all terms used during scene "
			 "tracking optimization.")
			("record_1d_slices",po::bool_switch(&loggingOptions.record1DSlices)->default_value(false),
			 "Used in dynamic fusion. Plot graphs of canonical and live SDF (around the focus coordinate,"
			 " if provided), plot the live frame progression and warp vectors (for visual debugging).")
			("record_live_as_slices",po::bool_switch(&loggingOptions.recordLiveSceneAsSlices)->default_value(false),
			 "Used in dynamic fusion. Slice up the whole live scene as images in each direction. ")
			("record_canonical_as_slices",po::bool_switch(&loggingOptions.recordCanonicalSceneAsSlices)->default_value(false),
			 "Used in dynamic fusion. Slice up the whole canonical scene as images in each direction. ")
			("record_2d_slices",po::bool_switch(&loggingOptions.record2DSlices)->default_value(false),
			 "Used in dynamic fusion. Render warps from each frame onto an image of the original live frame"
			 " (around the focus coordinate, if provided), as well as warped live frame"
			 " progression (for debugging).")
			("record_3d_slices",po::bool_switch(&loggingOptions.record3DSlices)->default_value(false),
			 "Used in dynamic fusion. Visualize & record a 3D slice of the canonical scene with warp vectors"
			 " and the live scene as they evolve.")
			("3d_slice_radius",po::value<unsigned int>(&loggingOptions._3DSliceRadius)->default_value(10),
			 "(Dynamic fusion) half-width of the square of pixels (in plane) in the slice for 3d slice recording.")
			("3d_slice_margin",po::value<unsigned int>(&loggingOptions._3DSliceExtraThicknessMargin)->default_value(0),
			 "(Dynamic fusion) extra margin to include, in voxels, from the 3D slice center along the axis "
			 "perpendicular to the slice plane.")
			("slice_plane",po::value<ITMLib::Plane>(&loggingOptions.planeFor2Dand3DSlices)->default_value(ITMLib::PLANE_XY),
			 "(Dynamic fusion) plane to use for recording of 2d slices.")
			("record_3d_scene_and_warps",po::bool_switch(&loggingOptions.record3DSceneAndWarps)->default_value(false),
			 "Used in dynamic fusion. Record 3D scenes at each frame and complete warp progression at every iteration.")

			/*================================================================================*/
			/*=== SurfaceTrackerOptimizationParameters for scene tracking optimization (KillingFusion/SobolevFusion) ===*/
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
			("disable_data_term", po::bool_switch(),
			 "Whether or not to disable the data term if using the DynamicFusion algorithm")
			("enable_level_set_term", po::bool_switch(),
			 "Whether or not to disable the level set term if using the DynamicFusion algorithm")
			("disable_smoothing_term", po::bool_switch(),
			 "Whether or not to disable the smoothness term if using the DynamicFusion algorithm")
			("enable_killing_term", po::bool_switch(),
			 "Whether or not to enable the Killing term (isometric motion enforcement regularizer) if using the "
			 "dynamic scene fusion algorithm")
			("disable_gradient_smoothing", po::bool_switch(),
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
}