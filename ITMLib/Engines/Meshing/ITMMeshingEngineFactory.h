// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "CPU/ITMMeshingEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/ITMMeshingEngine_CUDA.h"
#endif

namespace ITMLib
{

	/**
	 * \brief This struct provides functions that can be used to construct meshing engines.
	 */
	struct ITMMeshingEngineFactory
	{
		//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

		/**
		 * \brief Makes a meshing engine.
		 *
		 * \param deviceType  The device on which the meshing engine should operate.
		 */
		template <typename TVoxel, typename TIndex>
		static ITMMeshingEngine<TVoxel, TIndex> *MakeMeshingEngine(MemoryDeviceType deviceType, const TIndex& index)
		{
			ITMMeshingEngine<TVoxel, TIndex> *meshingEngine = NULL;

			switch (deviceType)
			{
			case MEMORYDEVICE_CPU:
				meshingEngine = new ITMMeshingEngine_CPU<TVoxel, TIndex>(index);
				break;
			case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
				meshingEngine = new ITMMeshingEngine_CUDA<TVoxel, TIndex>(index);
#endif
				break;
			case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
				meshingEngine = new ITMMeshingEngine_CPU<TVoxelCanonical, TIndex>(index);
#endif
				break;
			}

			return meshingEngine;
		}
	};
}