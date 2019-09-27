//  ================================================================
//  Created by Gregory Kramida on 7/24/18.
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

#include "../../../Objects/Scene/ITMPlainVoxelArray.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "../Interface/ITMSceneManipulationEngine.h"
#include "../../../Utils/ITMHashBlockProperties.h"


namespace ITMLib {

template<typename TVoxel, typename TIndex>
class ITMSceneManipulationEngine_CUDA : public ITMSceneManipulationEngine<TVoxel, TIndex> {
};


template<typename TVoxel>
class ITMSceneManipulationEngine_CUDA<TVoxel, ITMPlainVoxelArray> :
		public ITMSceneManipulationEngine<TVoxel, ITMPlainVoxelArray> {
public:

	//can be used as a singleton, but doesn't HAVE TO be
	static ITMSceneManipulationEngine_CUDA& Inst() {
		static ITMSceneManipulationEngine_CUDA<TVoxel, ITMPlainVoxelArray> instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	void ResetScene(ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* scene) override ;
	bool SetVoxel(ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* scene, Vector3i at, TVoxel voxel) override ;
	TVoxel ReadVoxel(ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* scene, Vector3i at);
	TVoxel ReadVoxel(ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* scene, Vector3i at,
	                 ITMPlainVoxelArray::IndexCache& cache) override;
	bool IsPointInBounds(ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* scene, const Vector3i& at);
	void OffsetWarps(ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* scene, Vector3f offset) override;
	bool CopySceneSlice(
			ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* destination,
			ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* source,
			Vector6i bounds, const Vector3i& offset) override;
	bool CopyScene(ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* destination,
	               ITMVoxelVolume <TVoxel, ITMPlainVoxelArray>* source,
	               const Vector3i& offset = Vector3i(0)) override;
};


template<typename TVoxel>
class ITMSceneManipulationEngine_CUDA<TVoxel, ITMVoxelBlockHash> :
		public ITMSceneManipulationEngine<TVoxel, ITMVoxelBlockHash> {
private:
	void* allocationTempData_device;
	void* allocationTempData_host;
	unsigned char* entriesAllocType_device;
	Vector3s* blockCoords_device;

public:
	ITMSceneManipulationEngine_CUDA();
	~ITMSceneManipulationEngine_CUDA();
	//can be used as a singleton, but doesn't HAVE TO be
	static ITMSceneManipulationEngine_CUDA& Inst() {
		static ITMSceneManipulationEngine_CUDA<TVoxel, ITMVoxelBlockHash> instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}
	void ResetScene(ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* scene);
	bool SetVoxel(ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* scene, Vector3i at, TVoxel voxel) override;
	TVoxel ReadVoxel(ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* scene, Vector3i at) override;
	TVoxel
	ReadVoxel(ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* scene, Vector3i at,
	          ITMVoxelBlockHash::IndexCache& cache) override;

	void OffsetWarps(ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* scene, Vector3f offset) override;
	bool CopySceneSlice(ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* destination, ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* source,
	                    Vector6i bounds, const Vector3i& offset = Vector3i(0)) override;

	bool CopyScene(ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* destination,
	               ITMVoxelVolume <TVoxel, ITMVoxelBlockHash>* source,
	               const Vector3i& offset = Vector3i(0)) override;

};


}//namespace ITMLib