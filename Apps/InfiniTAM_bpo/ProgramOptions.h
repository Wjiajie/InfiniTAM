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

#pragma once

#define BOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE
//boost
#include <boost/program_options.hpp>

//ITMLib
#include "../../ITMLib/Utils/Visualization/ITMVisualizationCommon.h"

namespace po = boost::program_options;

struct RunOptions{
	bool fixCamera = false;
	bool recordReconstructionToVideo = false;
	bool saveAfterInitialProcessing = false;
	bool loadVolumeBeforeProcessing = false;
	bool startInStepByStep = false;
};

struct LoggingOptions{
	bool recordCanonicalSceneAsSlices = false;
	bool recordLiveSceneAsSlices = false;
	bool record1DSlices = false;
	bool record2DSlices = false;
	bool record3DSlices = false;
	unsigned int _3DSliceRadius = 10;
	unsigned int _3DSliceExtraThicknessMargin = 0;
	bool record3DSceneAndWarps = false;
	bool plotEnergies = false;
	ITMLib::Plane planeFor2Dand3DSlices = ITMLib::PLANE_XY;
};

void PopulateOptionsDescription(po::options_description& arguments, RunOptions& runOptions, LoggingOptions& loggingOptions);