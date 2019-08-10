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
		bool isTruncated = (1.0f - sdfMagnitude) < FLT_EPSILON;
		if(isTruncated){
			voxelPos.flags = ITMLib::VOXEL_TRUNCATED;
			voxelNeg.flags =ITMLib::VOXEL_TRUNCATED;
		}else{
			voxelPos.flags = ITMLib::VOXEL_NONTRUNCATED;
			voxelNeg.flags = ITMLib::VOXEL_NONTRUNCATED;
		}

		for (int z = 0; z < surfaceSizeVoxelsZ; z++) {
			for (int y = 0; y < surfaceSizeVoxelsY; y++) {
				ITMSceneManipulationEngine_CPU<TVoxel,TIndex>::SetVoxel(scene, Vector3i(xPos, y, z), voxelPos);
				ITMSceneManipulationEngine_CPU<TVoxel,TIndex>::SetVoxel(scene, Vector3i(xNeg, y, z), voxelNeg);
			}
		}
	}

}