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

	const TVoxel* canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
	const TVoxel* liveVoxels = liveScene->localVBA.GetVoxelBlocks();

	const ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();
	const ITMHashEntry* liveHashTable = liveScene->index.GetEntries();

	int noTotalEntries = canonicalScene->index.noTotalEntries;

	float mu = canonicalScene->sceneParams->mu;

	typename TIndex::IndexCache cacheCanonical;
	typename TIndex::IndexCache cacheLive;


//#define USE_CENTRAL_DIFFERENCE


	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		Vector3i canonicalHashEntryPosition;
		const ITMHashEntry& currentCanonicalHashEntry = canonicalHashTable[entryId];

		if (currentCanonicalHashEntry.ptr < 0) continue;

		//position of the current entry in 3D space
		canonicalHashEntryPosition = currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3i originalPosition = canonicalHashEntryPosition + Vector3i(x, y, z);

					int foundVoxel;
					TVoxel canonicalVoxel =
							readVoxel(canonicalVoxels, canonicalHashTable, originalPosition, foundVoxel,
							          cacheCanonical);

					if (foundVoxel) {

						//TODO: compute without color if canonical sdf < 0.25f
						//=========================== PRELIMINARIES: FINITE DIFFERENCES ================================
						//find value at direct location of voxel projected with the warp vector
						Vector3f projectedPosition = originalPosition.toFloat() + canonicalVoxel.warp_t;
						float warpedSdf;
						Vector3f warpedColor;
						warpedSdf = interpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition, cacheLive,
						                                   warpedColor);

						//=== shifted warped sdf locations
						Vector3f warpedSdfForward;
						Vector3f warpedColorForward1[3];

						warpedSdfForward = ComputeWarpedTrilinearStep<TVoxel, TIndex, typename TIndex::IndexCache>(
								originalPosition, projectedPosition, 1,
								canonicalVoxels, canonicalHashTable, cacheCanonical,
								liveVoxels, liveHashTable, cacheLive,
								warpedColorForward1);

#ifdef USE_CENTRAL_DIFFERENCE
						Vector3f warpedSdfBackward1;
						Vector3f warpedColorBackward1[3];

						warpedSdfBackward1 = ComputeWarpedTrilinearStep<TVoxel, TIndex, typename TIndex::IndexCache>(
								originalPosition, projectedPosition, -1, canonicalVoxels, canonicalHashTable,
								cacheCanonical, liveVoxels, liveHashTable, cacheLive, warpedColorBackward1);
#else
						Vector3f warpedSdfForward2;
						Vector3f warpedColorForward2[3];

						warpedSdfForward = ComputeWarpedTrilinearStep<TVoxel, TIndex, typename TIndex::IndexCache>(
								originalPosition, projectedPosition, 2,
								canonicalVoxels, canonicalHashTable, cacheCanonical,
								liveVoxels, liveHashTable, cacheLive,
								warpedColorForward2);
#endif

						Vector3f warpedSdfCorners;
						Vector3f warpedColorCorners[3];
						Vector3i positionShift;

						positionShift = Vector3i(1, 1, 0); //(x+1, y+1, z) corner
						warpedSdfCorners.x =
								ComputeWarpedTrilinear<TVoxel, TIndex, typename TIndex::IndexCache>(
										originalPosition, projectedPosition, positionShift,
										canonicalVoxels, canonicalHashTable, cacheCanonical,
										liveVoxels, liveHashTable, cacheLive,
										warpedColorCorners[0]);

						positionShift = Vector3i(0, 1, 1); //(x, y+1, z+1) corner
						warpedSdfCorners.y =
								ComputeWarpedTrilinear<TVoxel, TIndex, typename TIndex::IndexCache>(
										originalPosition, projectedPosition, positionShift,
										canonicalVoxels, canonicalHashTable, cacheCanonical,
										liveVoxels, liveHashTable, cacheLive,
										warpedColorCorners[1]);

						positionShift = Vector3i(1, 0, 1); //(x+1, y, z+1) corner
						warpedSdfCorners.z =
								ComputeWarpedTrilinear<TVoxel, TIndex, typename TIndex::IndexCache>(
										originalPosition, projectedPosition, positionShift,
										canonicalVoxels, canonicalHashTable, cacheCanonical,
										liveVoxels, liveHashTable, cacheLive,
										warpedColorCorners[2]);

						//===
						//=== approximation for gradients of warped live scene,
						//=== i.e. partial 1-st order derivatives in 3 directions
						//===
						Vector3f sdfDerivatives_x_y_z = warpedSdfForward - Vector3f(warpedSdf);
						Vector3f colorDerivatives_x_y_z = Vector3f(
								squareDistance(warpedColorForward1[0], warpedColor),
								squareDistance(warpedColorForward1[1], warpedColor),
								squareDistance(warpedColorForward1[2], warpedColor));

						//===
						//=== Approximation for Hessian of warped live scene, i.e. partial 2-nd order derivatives in 3 directions
						//===
						//let warped live scene be denoted as phi(psi)
						//first, compute [delta_xx(phi(psi)), delta_yy(phi(psi)), delta_zz(phi(psi))]^T
#ifdef USE_CENTRAL_DIFFERENCE
						Vector3f sdfDerivatives_xx_yy_zz =
								warpedSdfForward - Vector3f(2.0f * warpedSdf) + warpedSdfBackward1 ;
						Vector3f colorDerivatives_xx_yy_zz = squareFiniteCentralDifferenceColor(
								warpedColorForward1, warpedColor, warpedColorBackward1);
#else
						Vector3f sdfDerivatives_xx_yy_zz =
								warpedSdfForward2 - 2 * warpedSdfForward + Vector3f(warpedSdf);
						Vector3f colorDerivatives_xx_yy_zz = squareFiniteForward2DifferenceColor(
								warpedColor, warpedColorForward1, warpedColorForward2);
#endif
						//=== corner-voxel auxiliary derivatives for later 2nd derivative approximations
						//Along the x axis, case 1: (0,1,0)->(1,1,0)
						//Along the y axis, case 1: (0,0,1)->(0,1,1)
						//Along the z axis, case 1: (1,0,0)->(1,0,1)

						Vector3f cornerSdfDerivatives1 = Vector3f(
								warpedSdfCorners.x - warpedSdfForward.y,
								warpedSdfCorners.y - warpedSdfForward.z,
								warpedSdfCorners.z - warpedSdfForward.x);
						Vector3f cornerColorDerivatives1 = Vector3f(
								squareDistance(warpedColorCorners[0], warpedColorForward1[1]),
								squareDistance(warpedColorCorners[1], warpedColorForward1[2]),
								squareDistance(warpedColorCorners[2], warpedColorForward1[0]));

						//Along the x axis, case 2: (0,0,1)->(1,0,1)
						//Along the y axis, case 2: (1,0,0)->(1,1,0)
						//Along the z axis, case 2: (0,1,0)->(0,1,1)
						Vector3f cornerSdfDerivatives2 = Vector3f(
								warpedSdfCorners.z - warpedSdfForward.x,
								warpedSdfCorners.x - warpedSdfForward.z,
								warpedSdfCorners.y - warpedSdfForward.y);
						Vector3f cornerColorDerivatives2 = Vector3f(
								squareDistance(warpedColorCorners[2], warpedColorForward1[0]),
								squareDistance(warpedColorCorners[0], warpedColorForward1[2]),
								squareDistance(warpedColorCorners[1], warpedColorForward1[1]));

						//===Compute the 2nd partial derivatives for different direction sequences
						Vector3f sdfDerivatives_xy_yz_zx = cornerSdfDerivatives1 - sdfDerivatives_x_y_z;
						Vector3f clrDerivatives_xy_yz_zx = cornerColorDerivatives1 - colorDerivatives_x_y_z;
						Vector3f sdfDerivatives_xz_yx_zy = cornerSdfDerivatives2 - sdfDerivatives_x_y_z;
						Vector3f clrDerivatives_xz_yx_zy = cornerColorDerivatives2 - colorDerivatives_x_y_z;

						// @formatter:off
						Matrix3f sdfHessianMatrix(sdfDerivatives_xx_yy_zz.x, sdfDerivatives_xy_yz_zx.x, sdfDerivatives_xz_yx_zy.x,
						                          sdfDerivatives_xz_yx_zy.y, sdfDerivatives_xx_yy_zz.y, sdfDerivatives_xy_yz_zx.y,
						                          sdfDerivatives_xy_yz_zx.z, sdfDerivatives_xz_yx_zy.z, sdfDerivatives_xx_yy_zz.z);

						Matrix3f clrHessianMatrix(colorDerivatives_xx_yy_zz.x, clrDerivatives_xy_yz_zx.x,   clrDerivatives_xz_yx_zy.x,
						                          clrDerivatives_xz_yx_zy.y,   colorDerivatives_xx_yy_zz.y, clrDerivatives_xy_yz_zx.y,
						                          clrDerivatives_xy_yz_zx.z,   clrDerivatives_xz_yx_zy.z,   colorDerivatives_xx_yy_zz.z);
						// @formatter:on
						//=================================== DATA TERM ================================================
						//Compute data term error / energy
						float energySdf = warpedSdf - canonicalVoxel.sdf;
						float energyColor = ITMSceneMotionTracker<TVoxel, TIndex>::weightColorDataTerm *
						                    squareDistance(warpedColor, canonicalVoxel.clr.toFloat());

						//The following are the multiplying the base data error by the components of the
						//of the delta(appyWarp(LiveSDF,Warp)) vector, equation 10
						//of the supplementary materials for KillingFusion, top line, to compute the data term's
						//contribution to the warp field update.

						Vector3f deltaEDataSdf = sdfDerivatives_x_y_z * energySdf;
						Vector3f deltaEDataColor = colorDerivatives_x_y_z * energyColor;
						Vector3f deltaEData = deltaEDataSdf + deltaEDataColor;

						//=================================== LEVEL SET TERM ===========================================
						Vector3f deltaELevelSetSdf =
								(sdfDerivatives_x_y_z - Vector3f(1.0)) * (sdfHessianMatrix * sdfDerivatives_x_y_z) /
								(sdfDerivatives_x_y_z + Vector3f(ITMSceneMotionTracker<TVoxel, TIndex>::epsilon));

						Vector3f deltaELevelSet = deltaELevelSetSdf;



					}
				}
			}
		}
	}

	DIEWITHEXCEPTION("Scene tracking iteration not yet implemented");
	return 0;
}

template<class TVoxel, class TIndex>
void ITMSceneMotionTracker_CPU<TVoxel, TIndex>::DeformScene(ITMScene<TVoxel, TIndex>* sceneOld,
                                                            ITMScene<TVoxel, TIndex>* sceneNew) {

	DIEWITHEXCEPTION("Scene tracking iteration not yet implemented");
}
