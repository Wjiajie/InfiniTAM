//  ================================================================
//  Created by Gregory Kramida on 10/23/17.
//  Copyright (c) 2017-2025 Gregory Kramida
//  Licensed under the Apache Li
// cense, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================

#define BOOST_TEST_MODULE SceneConstruction
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//stdlib
#include <random>
#include <vector>


//boost
#include <boost/test/unit_test.hpp>

//ITMlib
#include "../ITMLib/ITMLibDefines.h"
#include "../ITMLib/Objects/Scene/ITMVoxelVolume.h"
#include "../ITMLib/Objects/Scene/ITMRepresentationAccess.h"
#include "../ITMLib/Objects/Camera/ITMCalibIO.h"

#include "../ITMLib/Utils/ITMLibSettings.h"
#include "../ITMLib/Utils/Analytics/ITMSceneStatisticsCalculator.h"
#include "../ITMLib/Utils/Analytics/ITMVoxelVolumeComparison_CPU.h"

#include "../ITMLib/Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../ITMLib/Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../ITMLib/Engines/Manipulation/CUDA/ITMSceneManipulationEngine_CUDA.h"
#include "../ITMLib/Engines/ViewBuilding/ITMViewBuilderFactory.h"
#include "../ITMLib/Engines/SceneFileIO/ITMSceneFileIOEngine.h"
#include "../ITMLib/Engines/Reconstruction/Interface/ITMDynamicSceneReconstructionEngine.h"
#include "../ITMLib/Engines/Reconstruction/ITMDynamicSceneReconstructionEngineFactory.h"

#include "../InputSource/ImageSourceEngine.h"
#include "../ORUtils/FileUtils.h"

//local
#include "TestUtils.h"


using namespace ITMLib;

