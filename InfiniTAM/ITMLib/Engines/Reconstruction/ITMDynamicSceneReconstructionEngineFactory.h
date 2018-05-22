//  ================================================================
//  Created by Gregory Kramida on 5/22/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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

#include "../../Utils/ITMLibSettings.h"

#include "CPU/ITMDynamicSceneReconstructionEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
//#include "CUDA/ITMDynamicSceneReconstructionEngine_CUDA.h" TODO
#endif
#ifdef COMPILE_WITH_METAL
#error "NOT CURRENTLY SUPPORTED"
#endif

namespace ITMLib
{

/**
 * \brief This struct provides functions that can be used to construct scene reconstruction engines.
 */
struct ITMDynamicSceneReconstructionEngineFactory
{
	//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

	/**
	 * \brief Makes a scene reconstruction engine.
	 *
	 * \param deviceType  The device on which the scene reconstruction engine should operate.
	 */
	template <typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
	static ITMDynamicSceneReconstructionEngine<TVoxelCanonical, TVoxelLive,TIndex>* MakeSceneReconstructionEngine(ITMLibSettings::DeviceType deviceType)
	{
		ITMDynamicSceneReconstructionEngine<TVoxelCanonical, TVoxelLive,TIndex> *sceneRecoEngine = NULL;

		switch(deviceType)
		{
			case ITMLibSettings::DEVICE_CPU:
				sceneRecoEngine = new ITMDynamicSceneReconstructionEngine_CPU<TVoxelCanonical, TVoxelLive,TIndex>;
				break;
			case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
				DIEWITHEXCEPTION_REPORTLOCATION("Not yet implemented");
				//sceneRecoEngine = new ITMSceneReconstructionEngine_CUDA<TVoxelCanonical, TVoxelLive,TIndex>;
#endif
				break;
			case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
				sceneRecoEngine = new ITMSceneReconstructionEngine_Metal<TVoxelCanonical,TIndex>;
#endif
				break;
		}

		return sceneRecoEngine;
	}
};

}
