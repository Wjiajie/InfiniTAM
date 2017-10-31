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

#define _DEBUG
#ifdef _DEBUG
	double aveCanonicaSdf = 0.0f;
	int canonicalSdfCount = 0;
	double aveLiveSdf = 0.0f;
	double totalDataEnergy = 0.0f;
#endif

	const float epsilon = ITMSceneMotionTracker<TVoxel, TIndex>::epsilon;


#ifdef WITH_OPENMP
	//#pragma omp parallel for
#endif
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

					TVoxel& canonicalVoxel = localVoxelBlock[locId];

					//_DEBUG
					if(std::abs(canonicalVoxel.sdf - 1.0f) < epsilon) continue;

					//=================================== PRELIMINARIES ================================================
					//Jacobean and Hessian of the live scene sampled at warped location + deltas,
					//as well as local Jacobean and Hessian of the warp field itself
					float liveSdf;

					//_DEBUG
					Vector3f projectedPosition = originalPosition.toFloat() + canonicalVoxel.warp_t;
					liveSdf = interpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition, liveCache);
					if(std::abs(liveSdf - 1.0f) < epsilon) continue;


					bool useColor;
					float canonicalSdf = TVoxel::valueToFloat(canonicalVoxel.sdf);
					Vector3f liveColor;
					Vector3f liveSdfJacobean;
					//_DEBUG
					Vector3f liveSdfJacobeanAlt1;
					Vector3f liveColorJacobean;
					Matrix3f liveSdfHessian;

					if (canonicalSdf > 0.25f) {
						useColor = false;
						ComputePerPointWarpedLiveJacobianAndHessian<TVoxel, TIndex, typename TIndex::IndexCache>
								(originalPosition, canonicalVoxel.warp_t,
								 canonicalVoxels, canonicalHashTable, canonicalCache,
								 liveVoxels, liveHashTable, liveCache,
								 liveSdf, liveSdfJacobean, liveSdfHessian);
					} else {
						useColor = true;
						ComputePerPointWarpedLiveJacobianAndHessian<TVoxel, TIndex, typename TIndex::IndexCache>
								(originalPosition, canonicalVoxel.warp_t,
								 canonicalVoxels, canonicalHashTable, canonicalCache,
								 liveVoxels, liveHashTable, liveCache,
								 liveSdf, liveColor, liveSdfJacobean, liveColorJacobean, liveSdfHessian);
					}
					ComputePerPointWarpedLiveJacobianAlt2<TVoxel, TIndex, typename TIndex::IndexCache>
							(originalPosition, canonicalVoxel.warp_t,
							 canonicalVoxels, canonicalHashTable, canonicalCache,
							 liveVoxels, liveHashTable, liveCache,
							 liveSdf, liveSdfJacobeanAlt1);

					//_DEBUG
					//std::cout << "J1: " << std::endl << liveSdfJacobean << std::endl <<"J2: " << std::endl << liveSdfJacobeanAlt1 << std::endl <<std::endl;


//					if(entryId == 7861 && locId == 206){
//						int vmIndex;
//						//TVoxel voxel = readVoxel(canonicalVoxels, canonicalHashTable, originalPosition + Vector3i(0,1,0), vmIndex, canonicalCache);
//						//TVoxel voxel2 = readVoxel(canonicalVoxels, canonicalHashTable, originalPosition + Vector3i(-1,0,0), vmIndex, canonicalCache);
//						std::cout << "hi" << std::endl;
//					}
					Matrix3f warpJacobean;
					Matrix3f warpHessian[3] = {Matrix3f(), Matrix3f(), Matrix3f()};
					ComputePerPointWarpJacobianAndHessian<TVoxel, TIndex, typename TIndex::IndexCache>(
							canonicalVoxel.warp_t, originalPosition, canonicalVoxels, canonicalHashTable,
							canonicalCache, warpJacobean, warpHessian);

					//=================================== DATA TERM ====================================================
					//Compute data term error / energy
					float diffSdf = liveSdf - canonicalSdf;

					//_DEBUG
					Vector3f deltaEData = liveSdfJacobeanAlt1 * diffSdf;
					//Vector3f deltaEData = liveSdfJacobean * diffSdf;
					if (useColor) {
						float diffColor = ITMSceneMotionTracker<TVoxel, TIndex>::weightColorDataTerm *
						                    squareDistance(liveColor, TO_FLOAT3(canonicalVoxel.clr) / 255.f);
						deltaEData += liveColorJacobean * diffColor;
					}

					//=================================== LEVEL SET TERM ===============================================
					float sdfJacobianNorm = length(liveSdfJacobean);
					Vector3f deltaELevelSet =
							(sdfJacobianNorm - 1.0) * (liveSdfHessian * liveSdfJacobean) /
							(sdfJacobianNorm + ITMSceneMotionTracker<TVoxel, TIndex>::epsilon);

					//=================================== KILLING TERM =================================================


					const float gamma = ITMSceneMotionTracker<TVoxel, TIndex>::killingTermDampingFactor;
					float onePlusGamma = 1 + gamma;
					// |u_x, u_y, u_z|       |m00, m10, m20|
					// |v_x, v_y, v_z|       |m01, m11, m21|
					// |w_x, w_y, w_z|       |m02, m12, m22|
					Matrix3f& J = warpJacobean;
											//(1+gamma) * u_x
					Vector3f stackedVector0((onePlusGamma) * J.m00,
					                        //u_y + gamma * v_x
					                         J.m10 + gamma * J.m01,
					                        //u_z + gamma * w_x
					                         J.m20 + gamma * J.m02);

											//v_x + gamma * u_y
					Vector3f stackedVector1(J.m01 + gamma * J.m10,
					                        //(1+gamma) * v_y
					                        (onePlusGamma) * J.m11,
					                        //v_z + gamma * w_y
					                        J.m21 + gamma * J.m12);
											//w_x * gamma * u_z
					Vector3f stackedVector2(J.m02 + gamma * J.m20,
					                        //w_y + gamma * v_z
					                        J.m12 + gamma * J.m21,
					                        //(1+gamma) * w_z
					                        (onePlusGamma) * J.m22);

					Vector3f deltaEKilling = 2.0f *
					                         (warpHessian[0] * stackedVector0 +
					                          warpHessian[1] * stackedVector1 +
					                          warpHessian[2] * stackedVector2);

					//_DEBUG
					//alternative/naive struct lookup way to compute deltaEKilling:
					// JACOBEAN
					// |u_x, u_y, u_z|       |m00, m10, m20|
					// |v_x, v_y, v_z|       |m01, m11, m21|
					// |w_x, w_y, w_z|       |m02, m12, m22|
					// HESSIAN U
					// |0, 3, 6|     |m00, m10, m20|      |u_xx, u_xy, u_xz|
					// |1, 4, 7|     |m01, m11, m21|      |u_xy, u_yy, u_yz|
					// |2, 5, 8|     |m02, m12, m22|      |u_xz, u_yz, u_zz|
					// HESSIAN V
					// |0, 3, 6|     |m00, m10, m20|      |v_xx, v_xy, v_xz|
					// |1, 4, 7|     |m01, m11, m21|      |v_xy, v_yy, v_yz|
					// |2, 5, 8|     |m02, m12, m22|      |v_xz, v_yz, v_zz|
					// HESSIAN W
					// |0, 3, 6|     |m00, m10, m20|      |w_xx, w_xy, w_xz|
					// |1, 4, 7|     |m01, m11, m21|      |w_xy, w_yy, w_yz|
					// |2, 5, 8|     |m02, m12, m22|      |w_xz, w_yz, w_zz|
					Matrix3f& H_u = warpHessian[0];
					Matrix3f& H_v = warpHessian[1];
					Matrix3f& H_w = warpHessian[2];
					//_DEBUG
					Vector3f deltaEKillingAlt = 2.0f * Vector3f(
							//(1 + gamma) *u_x   *u_xx    + (u_y  + gamma*v_x)   * u_xy    + (u_z + gamma*w_x)    *u_xx
							onePlusGamma * J.m00 *H_u.m00 + (J.m10 + gamma*J.m01)* H_u.m10 + (J.m20 + gamma*J.m02)*H_u.m20 +
							//(v_x + gamma*u_y)  *v_xx	  + (1 + gamma)*v_y      * v_xy    + (v_z + gamma*w_y)    *v_xz
							(J.m01 + gamma*J.m10)*H_v.m00 + onePlusGamma  *J.m11 * H_v.m10 + (J.m21 + gamma*J.m12)*H_v.m20 +
							//(w_x + gamma*u_z)  *w_xx    + (w_y  + gamma* v_z)  * w_xy    + (1 + gamma)*w_z      *w_xz
 							(J.m02 + gamma*J.m20)*H_w.m00 + (J.m12 + gamma*J.m21)* H_w.m10 + onePlusGamma  *J.m22 *H_w.m20,

							//(1 + gamma) *u_x   *u_xy    + (u_y  + gamma*v_x)   * u_yy    + (u_z + gamma*w_x)    *u_yz
							onePlusGamma*J.m00   *H_u.m01 + (J.m10 + gamma*J.m01)* H_u.m11 + (J.m20 + gamma*J.m02)*H_u.m21 +
							//(v_x + gamma*u_y)  *v_xy	  + (1 + gamma)*v_y      * v_yy    + (v_z + gamma*w_y)    *v_yz
							(J.m01 + gamma*J.m10)*H_v.m01 + onePlusGamma*J.m11   * H_v.m11 + (J.m21 + gamma*J.m12)*H_v.m21 +
							//(w_x + gamma*u_z)  *w_xy    + (w_y  + gamma* v_z)  * w_yy    + (1 + gamma)*w_z      *w_yz
							(J.m02 + gamma*J.m20)*H_w.m01 + (J.m12 + gamma*J.m21)* H_w.m11 + onePlusGamma*J.m22   *H_w.m21,

							//(1 + gamma) *u_x   *u_xz    + (u_y  + gamma*v_x)   * u_yz    + (u_z + gamma*w_x)    *u_zz
							onePlusGamma*J.m00   *H_u.m02 + (J.m10 + gamma*J.m01)* H_u.m12 + (J.m20 + gamma*J.m02)*H_u.m22 +
							//(v_x + gamma*u_y)  *v_xz	  + (1 + gamma)*v_y      * v_yz    + (v_z + gamma*w_y)    *v_zz
							(J.m01 + gamma*J.m10)*H_v.m02 + onePlusGamma*J.m11   * H_v.m12 + (J.m21 + gamma*J.m12)*H_v.m22 +
							(J.m02 + gamma*J.m20)*H_w.m02 + (J.m12 + gamma*J.m21)* H_w.m12 + onePlusGamma*J.m22   *H_w.m22

					);

					//_DEBUG
					Vector3f inspectDeltaEKilling0 = warpHessian[0] * stackedVector0;
					Vector3f inspectDeltaEKilling1 = warpHessian[1] * stackedVector1;
					Vector3f inspectDeltaEKilling2 = warpHessian[2] * stackedVector2;
								//(1 + gamma) *u_x   *u_xx    + (u_y  + gamma*v_x)   * u_xy    + (u_z + gamma*w_x)   *u_xx
					float v00 = onePlusGamma * J.m00*H_u.m00 + (J.m10 + gamma*J.m01)*H_u.m10 + (J.m20 + gamma*J.m02)*H_u.m20;
								//(v_x + gamma*u_y)  *v_xx	  + (1 + gamma)*v_y      * v_xy    + (v_z + gamma*w_y)    *v_xz
					float v01 = (J.m01 + gamma*J.m10)*H_v.m00 + onePlusGamma  *J.m11 * H_v.m10 + (J.m21 + gamma*J.m12)*H_v.m20;

								//(w_x + gamma*u_z)  *w_xx    + (w_y  + gamma* v_z)  * w_xy    + (1 + gamma)*w_z      *w_xz
					float v02 = (J.m02 + gamma*J.m20)*H_w.m00 + (J.m12 + gamma*J.m21)* H_w.m10 + onePlusGamma  *J.m22 *H_w.m20;

								//(1 + gamma) *u_x   *u_xy    + (u_y  + gamma*v_x)   * u_yy    + (u_z + gamma*w_x)   *u_yz
					//float v001 = onePlusGamma * J.m00*H_u.m01 + (J.m10 + gamma*J.m01)*H_u.m11 + (J.m20 + gamma*J.m02)*H_u.m21;
					float v10 = onePlusGamma * J.m00*H_u.m10 + (J.m10 + gamma*J.m01)*H_u.m11 + (J.m20 + gamma*J.m02)*H_u.m21;
								//(1 + gamma) *u_x   *u_xz    + (u_y  + gamma*v_x)   * u_yz    + (u_z + gamma*w_x)   *u_zz
					float v20 = onePlusGamma * J.m00*H_u.m02 + (J.m10 + gamma*J.m01)*H_u.m12 + (J.m20 + gamma*J.m02)*H_u.m22;
					float v0 = v00 + v01 + v02;
					float iv0 = inspectDeltaEKilling0.x + inspectDeltaEKilling1.x + inspectDeltaEKilling2.x;


					//=================================== FINAL UPDATE =================================================
					const float weightKilling =  ITMSceneMotionTracker<TVoxel, TIndex>::weightKillingTerm;
					const float weightLevelSet = ITMSceneMotionTracker<TVoxel, TIndex>::weightLevelSetTerm;
					Vector3f deltaE = deltaEData;// + weightKilling * deltaEKillingAlt;
					// + weightLevelSet * deltaELevelSet + weightKilling * deltaEKilling;
					Vector3f warpUpdate = ITMSceneMotionTracker<TVoxel, TIndex>::gradientDescentUpdateFactor * deltaE;
					float vecLength = length(warpUpdate);
					if (maxVectorUpdate < vecLength) {
						maxVectorUpdate = vecLength;
					}

					canonicalVoxel.warp_t -= warpUpdate;

					//debug stats
					aveCanonicaSdf += canonicalSdf;
					aveLiveSdf += liveSdf;
					canonicalSdfCount += 1;
					totalDataEnergy += (diffSdf*diffSdf);
					//80280 252 4, 7, 3 -1.53724, -1.53811, 0
					//_DEBUG
//					if(std::isnan(deltaE.x) || std::isnan(deltaE.y) || std::isnan(deltaE.z)){
//						std::cout << entryId << " " << locId << " " << Vector3i(x,y,z) << " " << deltaE << std::endl;
//						DIEWITHEXCEPTION("NAN encountered");
//					}
					if(vecLength > 0.1){
						//std::cout << std::endl << " deltaEKilling: " << std::endl << deltaEKilling << std::endl << "deltaEKillingAlt: " <<std::endl << deltaEKillingAlt << std::endl;
						//std::cout << std::endl << " inspectDeltaEKilling0.x: " << inspectDeltaEKilling0.x << " deltaEKillingAlt v00: " << v00 << std::endl;
						//std::cout << std::endl << " inspectDeltaEKilling1.x: " << inspectDeltaEKilling1.x << " deltaEKillingAlt v01: " << v01 << std::endl;
						std::cout << std::endl << " inspectDeltaEKilling2.x: " << inspectDeltaEKilling2.x << " or " << dot(H_w.getRow(0),stackedVector2) << " deltaEKillingAlt v02: " << v02 << std::endl;
						//std::cout << std::endl << " iv0: " << iv0 << " v0: " << v0 << std::endl;
					}
					//voxel is 5mm, so 10 voxels is 5 cm
					if(deltaE.x > 10.0f || deltaE.y > 10.0f || deltaE.z > 10.0f){
						std::cout << std::endl << entryId << " " << locId << std::endl;
					}
					//std::cout << entryId << " " << locId << " " << Vector3i(x,y,z) << " " << deltaE << std::endl;
//					if(entryId == 7861 && locId == 206){
//						//std::cout << deltaE << warpUpdate;
//						//std::cout << canonicalVoxel.warp_t;
//						std::cout << std::endl << " deltaEData: " << std::endl << deltaEData << std::endl << "deltaEKilling: " <<std::endl << deltaEKilling << std::endl << "Update: " << std::endl << warpUpdate  << std::endl  << std::endl;
//					}
				}
			}
		}
	}
	aveCanonicaSdf /= canonicalSdfCount;
	aveLiveSdf /= canonicalSdfCount;
	std::cout << " Data term energy: " << totalDataEnergy;
	//std::cout << " Ave canonical SDF: " << aveCanonicaSdf << " Ave live SDF: " << aveLiveSdf << " 'proper' voxel count: " << canonicalSdfCount;
	return maxVectorUpdate;
}

template<typename TVoxel, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxel,TIndex>::FuseFrame(ITMScene<TVoxel, TIndex>* canonicalScene,
                                                         ITMScene<TVoxel, TIndex>* liveScene) {
	TVoxel* canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();
	typename TIndex::IndexCache canonicalCache;

	const TVoxel* liveVoxels = liveScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* liveHashTable = liveScene->index.GetEntries();
	typename TIndex::IndexCache liveCache;

	int maxW = canonicalScene->sceneParams->maxW;

	int noTotalEntries = canonicalScene->index.noTotalEntries;

#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
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
					TVoxel& canonicalVoxel = localVoxelBlock[locId];
					int oldWDepth, oldWColor;
					float oldSdf;
					oldSdf = canonicalVoxel.sdf;
					Vector3f oldColor = TO_FLOAT3(canonicalVoxel.clr) / 255.0f;
					oldWDepth = canonicalVoxel.w_depth;
					oldWColor = canonicalVoxel.w_color;

					Vector3f projectedPosition = originalPosition.toFloat() + canonicalVoxel.warp_t;

					Vector3f liveColor;
					int liveWDepth, liveWColor;
					float liveConfidence;
					float liveSdf = interpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition, liveCache,
					                                       liveColor, liveWDepth, liveWColor, liveConfidence);

					float newSdf = oldWDepth * oldSdf + liveWDepth * liveSdf;
					float newWDepth = oldWDepth + liveWDepth;
					newSdf /= newWDepth;
					newWDepth = MIN(newWDepth, maxW);

					Vector3f newColor = oldWColor * oldColor + liveWColor * liveColor;
					float newWColor = oldWColor + liveWColor;
					newColor /= newWColor;
					newWColor = MIN(newWDepth, maxW);

					canonicalVoxel.sdf = TVoxel::floatToValue(newSdf);
					canonicalVoxel.w_depth = (uchar)newWDepth;
					canonicalVoxel.clr = TO_UCHAR3(newColor * 255.0f);
					canonicalVoxel.w_color = (uchar)newWColor;
					canonicalVoxel.confidence += liveConfidence;
				}
			}
		}
	}

}

