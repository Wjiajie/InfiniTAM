//  ================================================================
//  Created by Gregory Kramida on 11/08/19.
//  Copyright (c) 2019 Gregory Kramida
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
#include "Configuration.h"
using namespace ITMLib;



Configuration::Configuration(const po::variables_map& vm) {

}

Configuration::Configuration()
:   //mu(m), maxW, voxel size(m), clipping min, clipping max, stopIntegratingAtMaxW
	scene_parameters(0.04f, 100, 0.004f, 0.2f, 3.0f, false),//corresponds to KillingFusion article //_DEBUG
	//scene_parameters(0.02f, 100, 0.005f, 0.2f, 3.0f, false),//standard InfiniTAM values
	surfel_scene_parameters(0.5f, 0.6f, static_cast<float>(20 * M_PI / 180), 0.01f, 0.004f, 3.5f, 25.0f, 4, 1.0f, 5.0f, 20, 10000000, true, true),
	slavcheva_parameters(SlavchevaSurfaceTracker::ConfigurationMode::SOBOLEV_FUSION,scene_parameters.voxelSize / scene_parameters.mu),
	slavcheva_switches(SlavchevaSurfaceTracker::ConfigurationMode::SOBOLEV_FUSION)
{
	// skips every other point when using the colour renderer for creating a point cloud
	skipPoints = true;

	// create all the things required for marching cubes and mesh extraction
	// - uses additional memory (lots!)
	createMeshingEngine = true;

#ifndef COMPILE_WITHOUT_CUDA
	deviceType = MEMORYDEVICE_CUDA;
#else
#ifdef COMPILE_WITH_METAL
	deviceType = MEMORYDEVICE_METAL;
#else
	deviceType = MEMORYDEVICE_CPU;
#endif
#endif

	/// how swapping works: disabled, fully enabled (still with dragons) and delete what's not visible - not supported in loop closure version
	swappingMode = SWAPPINGMODE_DISABLED;

	/// enables or disables approximate raycast
	useApproximateRaycast = false;

	/// enable or disable threshold depth filtering
	useThresholdFilter = false;

	/// enable or disable bilateral depth filtering
	useBilateralFilter = false;

	/// what to do on tracker failure: ignore, relocalise or stop integration - not supported in loop closure version
	behaviourOnFailure = FAILUREMODE_IGNORE;

	/// switch between various library modes - basic, with loop closure, etc.
	//libMode = LIBMODE_BASIC;
	//libMode = LIBMODE_LOOPCLOSURE;
	//libMode = LIBMODE_BASIC_SURFELS;
	libMode = LIBMODE_DYNAMIC;//_DEBUG

	//// Default ICP tracking
	//trackerConfig = "type=icp,levels=rrrbb,minstep=1e-3,"
	//				"outlierC=0.01,outlierF=0.002,"
	//				"numiterC=10,numiterF=2,failureDec=5.0"; // 5 for normal, 20 for loop closure

	// Depth-only extended tracker:
	trackerConfig = "type=extended,levels=rrbb,useDepth=1,minstep=1e-4,"
					  "outlierSpaceC=0.1,outlierSpaceF=0.004,"
					  "numiterC=20,numiterF=50,tukeyCutOff=8,"
					  "framesToSkip=20,framesToWeight=50,failureDec=20.0";

	//// For hybrid intensity+depth tracking:
	//trackerConfig = "type=extended,levels=bbb,useDepth=1,useColour=1,"
	//				  "colourWeight=0.3,minstep=1e-4,"
	//				  "outlierColourC=0.175,outlierColourF=0.005,"
	//				  "outlierSpaceC=0.1,outlierSpaceF=0.004,"
	//				  "numiterC=20,numiterF=50,tukeyCutOff=8,"
	//				  "framesToSkip=20,framesToWeight=50,failureDec=20.0";

	// Colour only tracking, using rendered colours
	//trackerConfig = "type=rgb,levels=rrbb";

	//trackerConfig = "type=imuicp,levels=tb,minstep=1e-3,outlierC=0.01,outlierF=0.005,numiterC=4,numiterF=2";
	//trackerConfig = "type=extendedimu,levels=ttb,minstep=5e-4,outlierSpaceC=0.1,outlierSpaceF=0.004,numiterC=20,numiterF=5,tukeyCutOff=8,framesToSkip=20,framesToWeight=50,failureDec=20.0";

	// Surfel tracking
	if(libMode == LIBMODE_BASIC_SURFELS)
	{
		trackerConfig = "extended,levels=rrbb,minstep=1e-4,outlierSpaceC=0.1,outlierSpaceF=0.004,numiterC=20,numiterF=20,tukeyCutOff=8,framesToSkip=0,framesToWeight=1,failureDec=20.0";
	}
	// Killing-constraint tracking
	if(libMode == LIBMODE_DYNAMIC)
	{
//		trackerConfig = "type=killing,levels=rrbb,useDepth=1,minstep=1e-4,"
//				"outlierSpaceC=0.1,outlierSpaceF=0.004,"
//				"numiterC=20,numiterF=50,tukeyCutOff=8,"
//				"framesToSkip=20,framesToWeight=50,failureDec=20.0";
		//// For hybrid intensity+depth tracking:
		trackerConfig = "type=killing,levels=bbb,useDepth=1,useColour=1,"
						  "colourWeight=0.3,minstep=1e-4,"
						  "outlierColourC=0.175,outlierColourF=0.005,"
						  "outlierSpaceC=0.1,outlierSpaceF=0.004,"
						  "numiterC=20,numiterF=50,tukeyCutOff=8,"
						  "framesToSkip=20,framesToWeight=50,failureDec=20.0";
	}

	//TODO: the following 3 groups should be kept in 4 separate structs in here, perpetually; these structs should be passed as pointers to the classes that use them (instead of being copied to structs in those classes)

	// Dynamic fusion debugging/logging
	analysisSettings.focus_coordinates_specified = false;
	//By default, write to a State folder within the current directory
	analysisSettings.output_path = "./State/";

	restrictZtrackingForDebugging = false;

	// Dynamic fusion optimization termination parameters
	surface_tracking_max_optimization_iteration_count = 200;
	surface_tracking_optimization_vector_update_threshold_meters = 0.0001f;// in meters, default from KillingFusion
}

bool Configuration::FocusCoordinatesAreSpecified() const {
	return analysisSettings.focus_coordinates_specified;
}

MemoryDeviceType Configuration::GetMemoryType() const
{
	return deviceType == MEMORYDEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
}



Configuration::AnalysisSettings::AnalysisSettings() :
output_path("output/"),
focus_coordinates_specified(false),
focus_coordinates(Vector3i(0))
{}

static Vector3i vector3i_from_variable_map(const po::variables_map& vm, std::string argument){
	Vector3i vec;
	std::vector<int> int_vector = vm[argument].as<std::vector<int>>();
	if (int_vector.size() != 3) {
		DIEWITHEXCEPTION_REPORTLOCATION("Could not parse argument as exactly 3 integers, \"x y z\"");
	}
	memcpy(vec.values, int_vector.data(), sizeof(int) * 3);
	return vec;
}

Configuration::AnalysisSettings::AnalysisSettings(const po::variables_map& vm):
output_path(vm["output"].empty() ? AnalysisSettings().output_path : vm["output"].as<std::string>().c_str()),
focus_coordinates_specified(!vm["focus_coordinates"].empty()),
focus_coordinates(vm["focus_coordinates"].empty() ? AnalysisSettings().focus_coordinates :
vector3i_from_variable_map(vm, "focus_coordinates"))
{}
