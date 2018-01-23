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
#include "../ITMLib/Objects/Scene/ITMSceneManipulation.h"
#include "../ITMLib/Engines/Reconstruction/ITMSceneReconstructionEngineFactory.h"

using namespace ITMLib;

template<class TVoxel, class TIndex>
void GenerateTestScene01(ITMScene<TVoxel, TIndex>& scene) {

	ITMSceneReconstructionEngine<TVoxel, TIndex>* reconstructionEngine =
			ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxel, TIndex>(
					ITMLibSettings::DEVICE_CPU);

	reconstructionEngine->ResetScene(&scene);
	const int narrowBandThicknessVoxels = 10;
	int xOffset = 8;
	int surfaceSizeVoxels = 16; //in both y and z directions here

	for (int iVoxelAcrossBand = 0; iVoxelAcrossBand < narrowBandThicknessVoxels + 1; iVoxelAcrossBand++) {

		float sdfValue = 0.0F + iVoxelAcrossBand * (1.0F / narrowBandThicknessVoxels);
		int xPos = xOffset + iVoxelAcrossBand;
		int xNeg = xOffset - iVoxelAcrossBand;
		TVoxel voxelPos, voxelNeg;
		voxelPos.sdf = sdfValue;
		voxelNeg.sdf = -sdfValue;
		for (int z = 0; z < surfaceSizeVoxels; z++) {
			for (int y = 0; y < surfaceSizeVoxels; y++) {
				SetVoxel_CPU(scene, Vector3i(xPos, y, z), voxelPos);
				SetVoxel_CPU(scene, Vector3i(xNeg, y, z), voxelNeg);
			}
		}
	}

}