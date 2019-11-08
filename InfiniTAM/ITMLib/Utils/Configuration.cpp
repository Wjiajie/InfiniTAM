// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "Configuration.h"
using namespace ITMLib;



void Configuration::SetFromVariableMap(const po::variables_map& vm) {

}

Configuration::Configuration()
:   //mu(m), maxW, voxel size(m), clipping min, clipping max, stopIntegratingAtMaxW
	sceneParams(0.04f, 100, 0.004f, 0.2f, 3.0f, false),//corresponds to KillingFusion article //_DEBUG
	//sceneParams(0.02f, 100, 0.005f, 0.2f, 3.0f, false),//standard InfiniTAM values
	surfelSceneParams(0.5f, 0.6f, static_cast<float>(20 * M_PI / 180), 0.01f, 0.004f, 3.5f, 25.0f, 4, 1.0f, 5.0f, 20, 10000000, true, true)
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

	//_DEBUG
	deviceType = MEMORYDEVICE_CPU;

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
	analysisSettings.focusCoordinatesSpecified = false;
	//By default, write to a State folder within the current directory
	analysisSettings.outputPath = "./State/";

	restrictZtrackingForDebugging = false;


	// Dynamic fusion switches
	enableDataTerm = true;
	enableLevelSetTerm = true;
	enableSmoothingTerm = true;
	enableKillingConstraintInSmoothingTerm = false;
	enableGradientSmoothing = true;

	// Dynamic fusion optimization termination parameters
	sceneTrackingMaxOptimizationIterationCount = 200;
	sceneTrackingOptimizationVectorUpdateThresholdMeters = 0.0001f;// in meters, default from KillingFusion

	// Dynamic fusion weights / factors used during scene tracking
	sceneTrackingGradientDescentLearningRate = 0.1f; // default from KillingFusion & SobolevFusion
	sceneTrackingRigidityEnforcementFactor = 0.1f; // default from KillingFusion
	sceneTrackingWeightDataTerm = 1.0f; // not used in Killing/Sobolev Fusion (implicitly adjusted using other weights & thresholds)
	sceneTrackingWeightSmoothingTerm = 0.2f; // default from SobolevFusion, 0.5f used in KillingFusion
	sceneTrackingWeightLevelSetTerm = 0.2f; // default from KillingFusion
	sceneTrackingLevelSetTermEpsilon = 1e-5f; // default from KillingFusion //TODO: scaling factor of 0.01, since their units are mm and ours are m?
}

bool Configuration::FocusCoordinatesAreSpecified() const {
	return analysisSettings.focusCoordinatesSpecified;
}

MemoryDeviceType Configuration::GetMemoryType() const
{
	return deviceType == MEMORYDEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
}

Vector3i Configuration::GetFocusCoordinates() const {
	return analysisSettings.focusCoordinates;
}

void Configuration::SetFocusCoordinates(const Vector3i& coordiantes) {
	analysisSettings.focusCoordinatesSpecified = true;
	analysisSettings.focusCoordinates = coordiantes;
}

void Configuration::SetFocusCoordinates(int x, int y, int z) {
	this->SetFocusCoordinates(Vector3i(x, y, z));
}



