//  ================================================================
//  Created by Gregory Kramida on 10/18/17.
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
#include <cmath>
#include "ITMSceneMotionTracker_CPU.h"
#include "../Shared/ITMSceneMotionTracker_Shared.h"

using namespace ITMLib;

template<typename TVoxel, typename TIndex>
float
ITMSceneMotionTracker_CPU<TVoxel, TIndex>::UpdateWarpField(ITMScene<TVoxel, TIndex>* canonicalScene,
                                                           ITMScene<TVoxel, TIndex>* liveScene) {

	TVoxel* canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();
	typename TIndex::IndexCache canonicalCache;

	const TVoxel* liveVoxels = liveScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* liveHashTable = liveScene->index.GetEntries();
	typename TIndex::IndexCache liveCache;

	int noTotalEntries = canonicalScene->index.noTotalEntries;
	float maxVectorUpdate = 0.0f;

	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		Vector3i canonicalHashEntryPosition;
		const ITMHashEntry& currentCanonicalHashEntry = canonicalHashTable[entryId];

		if (currentCanonicalHashEntry.ptr < 0) continue;

		//position of the current entry in 3D space
		canonicalHashEntryPosition = currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		TVoxel* localVoxelBlock = &(canonicalVoxels[currentCanonicalHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3i originalPosition = canonicalHashEntryPosition + Vector3i(x, y, z);

					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxel canonicalVoxel = localVoxelBlock[locId];

					//=================================== PRELIMINARIES ================================================
					//Jacobean and Hessian of the live scene sampled at warped location + deltas,
					//as well as local Jacobean and Hessian of the warp field itself
					float warpedSdf;
					bool useColor;
					Vector3f warpedColor;
					Vector3f warpedSdfJacobean;
					Vector3f warpedColorJacobean;
					Matrix3f warpedSdfHessian;

					if (canonicalVoxel.sdf > 0.25f) {
						useColor = false;
						ComputePerPointWarpedLiveJacobianAndHessian<TVoxel, TIndex, typename TIndex::IndexCache>
								(originalPosition, canonicalVoxel.warp_t,
								 canonicalVoxels, canonicalHashTable, canonicalCache,
								 liveVoxels, liveHashTable, liveCache,
								 warpedSdf, warpedSdfJacobean, warpedSdfHessian);
					} else {
						useColor = true;
						ComputePerPointWarpedLiveJacobianAndHessian<TVoxel, TIndex, typename TIndex::IndexCache>
								(originalPosition, canonicalVoxel.warp_t,
								 canonicalVoxels, canonicalHashTable, canonicalCache,
								 liveVoxels, liveHashTable, liveCache,
								 warpedSdf, warpedColor, warpedSdfJacobean, warpedColorJacobean, warpedSdfHessian);
					}


					Matrix3f warpJacobian;
					Matrix3f warpHessian[3];
					ComputePerPointWarpJacobianAndHessian<TVoxel, TIndex, typename TIndex::IndexCache>(
							canonicalVoxel.warp_t, originalPosition, canonicalVoxels, canonicalHashTable,
							canonicalCache, warpJacobian, warpHessian);

					//=================================== DATA TERM ====================================================
					//Compute data term error / energy
					float energySdf = warpedSdf - canonicalVoxel.sdf;

					Vector3f deltaEData = warpedSdfJacobean * energySdf;
					if (useColor) {
						float energyColor = ITMSceneMotionTracker<TVoxel, TIndex>::weightColorDataTerm *
						                    squareDistance(warpedColor, canonicalVoxel.clr.toFloat());
						deltaEData += warpedColorJacobean * energyColor;
					}

					//=================================== LEVEL SET TERM ===============================================
					Vector3f deltaELevelSet =
							(warpedSdfJacobean - Vector3f(1.0)) * (warpedSdfHessian * warpedSdfJacobean) /
							(warpedSdfJacobean + Vector3f(ITMSceneMotionTracker<TVoxel, TIndex>::epsilon));

					//=================================== KILLING TERM =================================================


					const float gamma = ITMSceneMotionTracker<TVoxel, TIndex>::killingTermDampingFactor;
					// |u_x, u_y, u_z|       |m00, m10, m20|
					// |v_x, v_y, v_z|       |m01, m11, m21|
					// |w_x, w_y, w_z|       |m02, m12, m22|
					Vector3f stackedVector0((1.0f + gamma) * warpJacobian.m00,
					                        warpJacobian.m10 + gamma * warpJacobian.m01,
					                        warpJacobian.m20 + gamma * warpJacobian.m02);
					Vector3f stackedVector1(warpJacobian.m01 + gamma * warpJacobian.m10,
					                        (1.0f + gamma) * warpJacobian.m11,
					                        warpJacobian.m21 + gamma * warpJacobian.m12);
					Vector3f stackedVector2(warpJacobian.m02 * gamma * warpJacobian.m20,
					                        warpJacobian.m12 + gamma * warpJacobian.m21,
					                        (1.0f + gamma) * warpJacobian.m22);

					Vector3f deltaEKilling = 2.0f *
					                         warpHessian[0] * stackedVector0 +
					                         warpHessian[1] * stackedVector1 +
					                         warpHessian[2] * stackedVector2;

					//=================================== FINAL UPDATE =================================================

					Vector3f deltaE = deltaEData + deltaELevelSet + deltaEKilling;
					Vector3f warpUpdate = ITMSceneMotionTracker<TVoxel, TIndex>::gradientDescentUpdateFactor * deltaE;
					float vecLength = length(warpUpdate);
					if (maxVectorUpdate < vecLength) {
						maxVectorUpdate = vecLength;
					}

					localVoxelBlock[locId].warp_t = canonicalVoxel.warp_t - warpUpdate;
				}
			}
		}
	}

	DIEWITHEXCEPTION("Scene tracking iteration not yet implemented");
	return 0;
}

