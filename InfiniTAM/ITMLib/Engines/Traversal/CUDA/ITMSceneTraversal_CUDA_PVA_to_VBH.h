//  ================================================================
//  Created by Gregory Kramida on 10/4/19.
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
#pragma once


//local
#include "../Shared/ITMSceneTraversal_Shared.h"
#include "../Interface/ITMSceneTraversal.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "../../../Utils/Analytics/ITMIsAltered.h"
#include "../../../Objects/Scene/ITMRepresentationAccess.h"

namespace ITMLib {


template<typename TVoxelPrimary, typename TVoxelSecondary>
class ITMDualSceneTraversalEngine<TVoxelPrimary, TVoxelSecondary, ITMPlainVoxelArray, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CUDA> {
public:
	/**
	 * \brief Routine allowing some kind of comparison function call on voxel pairs from the two scenes where both
	 * voxels share the same spatial location.
	 * \details voxels that are not modified / have default value (see isModified for how that works) in the primary
	 * voxel volume are ignored if the voxel hash block at this location at the secondary voxel volume has not been
	 * allocated.
	 * \tparam TFunctor type of the function object (see parameter description)
	 * \param primaryVolume the primary volume -- indexed using plain voxel array (PVA)
	 * \param secondaryVolume the secondary volume -- indexed using voxel block hash table (VBH)
	 * \param functor a function object accepting two voxels by reference as arguments and returning true/false
	 * \return true if the matching functor returns "true" for all allocated voxels, false otherwise.
	 */
	template<typename TFunctor>
	inline static bool
	DualVoxelTraversal_AllTrue(
			ITMVoxelVolume<TVoxelPrimary, ITMPlainVoxelArray>* primaryVolume,
			ITMVoxelVolume<TVoxelSecondary, ITMVoxelBlockHash>* secondaryVolume,
			TFunctor& functor) {
		DIEWITHEXCEPTION_REPORTLOCATION("NOT IMPLEMENTED");
	}

};
}//namespace ITMLib
