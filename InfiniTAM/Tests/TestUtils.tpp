//  ================================================================
//  Created by Gregory Kramida on 11/3/17.
//  Copyright (c) 2017-2025 Gregory Kramida
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
#include "TestUtils.h"
#include "../ITMLib/Utils/ITMLibSettings.h"
#include "../ITMLib/Engines/Manipulation/CPU/ITMSceneManipulationEngine_CPU.h"
#include "../ITMLib/Engines/Reconstruction/ITMSceneReconstructionEngineFactory.h"

using namespace ITMLib;

template<class TVoxel, class TIndex>
void GenerateTestScene01(ITMVoxelVolume<TVoxel, TIndex>* scene) {
	ITMSceneManipulationEngine_CPU<TVoxel, TIndex>::ResetScene(scene);
	const int narrowBandThicknessVoxels = 10;
	int xOffset = 8;
	int surfaceSizeVoxelsZ = 16;
	int surfaceSizeVoxelsY = 64;

	for (int iVoxelAcrossBand = 0; iVoxelAcrossBand < narrowBandThicknessVoxels + 1; iVoxelAcrossBand++) {
		float sdfMagnitude = 0.0F + iVoxelAcrossBand * (1.0F / narrowBandThicknessVoxels);
		int xPos = xOffset + iVoxelAcrossBand;
		int xNeg = xOffset - iVoxelAcrossBand;
		TVoxel voxelPos, voxelNeg;
		voxelPos.sdf = sdfMagnitude;
		voxelNeg.sdf = -sdfMagnitude;
		simulateVoxelAlteration(voxelNeg);
		simulateVoxelAlteration(voxelPos);

		for (int z = 0; z < surfaceSizeVoxelsZ; z++) {
			for (int y = 0; y < surfaceSizeVoxelsY; y++) {
				ITMSceneManipulationEngine_CPU<TVoxel,TIndex>::SetVoxel(scene, Vector3i(xPos, y, z), voxelPos);
				ITMSceneManipulationEngine_CPU<TVoxel,TIndex>::SetVoxel(scene, Vector3i(xNeg, y, z), voxelNeg);
			}
		}
	}

}


template<bool hasSemanticInformation, typename TVoxel>
struct SimulateVoxelAlterationFunctor;

template<typename TVoxel>
struct SimulateVoxelAlterationFunctor<true, TVoxel> {
	inline static
	void run(TVoxel& voxel) {
		if (voxel.sdf > -1.0f && voxel.sdf < 1.0f) {
			voxel.flags = VoxelFlags::VOXEL_NONTRUNCATED;
		} else {
			voxel.flags = VoxelFlags::VOXEL_TRUNCATED;
		}
	}
};

template<typename TVoxel>
struct SimulateVoxelAlterationFunctor<false, TVoxel> {
	inline static
	void run(TVoxel& voxel) {
		voxel.w_depth = 1;
	}
};

template<typename TVoxel>
void simulateVoxelAlteration(TVoxel& voxel){
	SimulateVoxelAlterationFunctor<ITMVoxel::hasSemanticInformation, ITMVoxel>::run(voxel);
}