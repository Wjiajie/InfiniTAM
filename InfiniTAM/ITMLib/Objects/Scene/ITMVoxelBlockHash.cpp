//  ================================================================
//  Created by Gregory Kramida on 11/8/19.
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

#include "ITMVoxelBlockHash.h"
#include "../../ITMLibDefines.h"
#include "../../Engines/Indexing/VBH/CPU/ITMIndexingEngine_CPU_VoxelBlockHash.h"
#include "../../Engines/Indexing/VBH/CUDA/ITMIndexingEngine_CUDA_VoxelBlockHash.h"

namespace ITMLib {
ITMHashEntry ITMVoxelBlockHash::GetHashEntryAt_CPU(const Vector3s& pos) const {
	const ITMHashEntry* entries = this->GetEntries();
	switch (memoryType) {
		case MEMORYDEVICE_CPU:
			return ITMIndexingEngine<ITMVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CPU>::Instance()
					.FindHashEntry(*this,pos);
#ifndef COMPILE_WITHOUT_CUDA
		case MEMORYDEVICE_CUDA:
			return ITMIndexingEngine<ITMVoxel, ITMVoxelBlockHash, MEMORYDEVICE_CUDA>::Instance()
					.FindHashEntry(*this,pos);
#endif
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported device type.");
			return ITMHashEntry();
	}
}
}// namespace ITMLib
