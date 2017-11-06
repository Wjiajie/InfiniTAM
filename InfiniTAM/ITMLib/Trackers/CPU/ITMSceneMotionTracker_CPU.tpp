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
#include <iomanip>
#include "ITMSceneMotionTracker_CPU.h"
#include "../Shared/ITMSceneMotionTracker_Shared.h"
#include "../../ITMLibDefines.h"

using namespace ITMLib;


template<typename TVoxel, typename TIndex>
float
ITMSceneMotionTracker_CPU<TVoxel, TIndex>::UpdateWarpField(ITMScene<TVoxel, TIndex>* canonicalScene,
                                                           ITMScene<ITMVoxelAux, TIndex>* liveScene) {

	TVoxel* canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();
	typename TIndex::IndexCache canonicalCache;

	const ITMVoxelAux* liveVoxels = liveScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* liveHashTable = liveScene->index.GetEntries();
	typename TIndex::IndexCache liveCache;

	int noTotalEntries = canonicalScene->index.noTotalEntries;
	float maxVectorUpdate = 0.0f;

#define _DEBUG
#ifdef _DEBUG
	double aveCanonicaSdf = 0.0;
	int consideredVoxelCount = 0;
	double aveLiveSdf = 0.0;
	double aveSdfDiff = 0.0;
	Vector3f trackedDataUpdate;
	Vector3f maxKillingUpdate;
	float maxKillingUpdateLength = 0.0;
	double totalDataEnergy = 0.0;
	double totalLevelSetEnergy = 0.0;
	double totalSmoothnessEnergy = 0.0;
	double totalKillingEnergy = 0.0;
	double totalEnergy = 0.0;
	double aveWarpDist = 0.0;
#endif

	const float epsilon = ITMSceneMotionTracker<TVoxel, TIndex>::epsilon;


	//compute the update, don't apply yet (computation depends on previous warp for neighbors,
	// no practical way to keep those buffered with the hash in mind)
#ifdef WITH_OPENMP
	//#pragma omp parallel for firstprivate(canonicalCache, liveCache) reduction(+:aveCanonicaSdf, consideredVoxelCount, aveLiveSdf, totalDataEnergy, totalLevelSetEnergy, totalSmoothnessEnergy, totalKillingEnergy, aveWarpDist, aveSdfDiff)
#endif
	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		Vector3i canonicalHashEntryPosition;
		const ITMHashEntry& currentCanonicalHashEntry = canonicalHashTable[entryId];

		if (currentCanonicalHashEntry.ptr < 0) continue;

		//position of the current entry in 3D space

		canonicalHashEntryPosition = currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		//_DEBUG
		//std::cout << std::endl << "HASH POS: " << canonicalHashEntryPosition << std::endl;
		TVoxel* localVoxelBlock = &(canonicalVoxels[currentCanonicalHashEntry.ptr * (SDF_BLOCK_SIZE3)]);


		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3i originalPosition = canonicalHashEntryPosition + Vector3i(x, y, z);

					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

					TVoxel& canonicalVoxel = localVoxelBlock[locId];

					//=================================== PRELIMINARIES ================================================
					//Jacobian and Hessian of the live scene sampled at warped location + deltas,
					//as well as local Jacobian and Hessian of the warp field itself
					float liveSdf;

					//_DEBUG
					Vector3f projectedPosition = originalPosition.toFloat() + canonicalVoxel.warp_t;
					liveSdf = interpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition, liveCache);
					//_DEBUG
					if (1.0f - std::abs(liveSdf) < 10e-10) {
						float liveSdfAlt = interpolateTrilinearlyAlt(liveVoxels, liveHashTable, projectedPosition,
						                                             liveCache);
						//if(std::abs(liveSdfAlt - liveSdf) > epsilon){
						//std::cout << "Mismatching interpolation: " << liveSdf << " v.s. " << liveSdfAlt << std::endl;
						//}
					}


					//_DEBUG
					//almost no restriction
					//if (1.0f - std::abs(canonicalVoxel.sdf) < epsilon && 1.0f - std::abs(liveSdf) < epsilon) continue;
					//if (1.0f - std::abs(canonicalVoxel.sdf) < epsilon) continue;
					//if (1.0f - std::abs(liveSdf) < epsilon) continue;

					//most restrictive
					if (1.0f - std::abs(canonicalVoxel.sdf) < epsilon || 1.0f - fabs(liveSdf) < epsilon) continue;

					bool useColor;
					float canonicalSdf = TVoxel::valueToFloat(canonicalVoxel.sdf);
					Vector3f liveColor;
					Vector3f liveSdfJacobian;
					Vector3f liveColorJacobian;
					Matrix3f liveSdfHessian;

					if (canonicalSdf > ITMSceneMotionTracker<TVoxel, TIndex>::colorSdfThreshold) {
						useColor = false;
						ComputePerPointWarpedLiveJacobianAndHessian<TVoxel, TIndex, typename TIndex::IndexCache>
								(originalPosition, canonicalVoxel.warp_t,
								 canonicalVoxels, canonicalHashTable, canonicalCache,
								 liveVoxels, liveHashTable, liveCache,
								 liveSdf, liveSdfJacobian, liveSdfHessian);
					} else {
						useColor = true;
						ComputePerPointWarpedLiveJacobianAndHessian<TVoxel, TIndex, typename TIndex::IndexCache>
								(originalPosition, canonicalVoxel.warp_t,
								 canonicalVoxels, canonicalHashTable, canonicalCache,
								 liveVoxels, liveHashTable, liveCache,
								 liveSdf, liveColor, liveSdfJacobian, liveColorJacobian, liveSdfHessian);
					}
					Matrix3f warpJacobian;
					Matrix3f warpHessian[3] = {Matrix3f(), Matrix3f(), Matrix3f()};
					ComputePerPointWarpJacobianAndHessian<TVoxel, TIndex, typename TIndex::IndexCache>(
							canonicalVoxel.warp_t, originalPosition, canonicalVoxels, canonicalHashTable,
							canonicalCache, warpJacobian, warpHessian);
					//_DEBUG
//					ComputePerPointWarpJacobianAndHessianAlt<TVoxel, TIndex, typename TIndex::IndexCache>(
//							canonicalVoxel.warp_t, originalPosition, canonicalVoxels, canonicalHashTable,
//							canonicalCache, warpJacobian, warpHessian);

					//=================================== DATA TERM ====================================================
					//Compute data term error / energy
					float diffSdf = liveSdf - canonicalSdf;

					//_DEBUG
					Vector3f deltaEData = liveSdfJacobian * diffSdf;
					//Vector3f deltaEData = liveSdfJacobian * diffSdf;
					if (useColor) {
						float diffColor = ITMSceneMotionTracker<TVoxel, TIndex>::weightColorDataTerm *
						                  squareDistance(liveColor, TO_FLOAT3(canonicalVoxel.clr) / 255.f);
						deltaEData += liveColorJacobian * diffColor;
					}

					//=================================== LEVEL SET TERM ===============================================
					float sdfJacobianNorm = length(liveSdfJacobian);
					float sdfJacobianNormMinusOne = sdfJacobianNorm - 1.0f;
					Vector3f deltaELevelSet =
							sdfJacobianNormMinusOne * (liveSdfHessian * liveSdfJacobian) /
							(sdfJacobianNorm + ITMSceneMotionTracker<TVoxel, TIndex>::epsilon);

					//=================================== KILLING TERM =================================================
					const float gamma = ITMSceneMotionTracker<TVoxel, TIndex>::rigidityEnforcementFactor;
					float onePlusGamma = 1.0f + gamma;
					// |u_x, u_y, u_z|       |m00, m10, m20|
					// |v_x, v_y, v_z|       |m01, m11, m21|
					// |w_x, w_y, w_z|       |m02, m12, m22|
					Matrix3f& J = warpJacobian;
					// @formatter:off
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
					// @formatter:on
					Vector3f deltaEKilling = 2.0f *
					                         (warpHessian[0] * stackedVector0 +
					                          warpHessian[1] * stackedVector1 +
					                          warpHessian[2] * stackedVector2);
					//_DEBUG
//					Vector3f stackedSmVector0 = J.getRow(0);
//					Vector3f stackedSmVector1 = J.getRow(1);
//					Vector3f stackedSmVector2 = J.getRow(2);
//					Vector3f deltaESmooth = 2.0f*(warpHessian[0] * stackedSmVector0 +
//					                              warpHessian[1] * stackedSmVector1 +
//					                              warpHessian[2] * stackedSmVector2);
					//_DEBUG
//					if(sum(deltaESmooth - deltaEKilling) > 1.0e-10f){
//						std::cout << "ERROR!" << std::endl;
//					}

					//_DEBUG
					// KillingTerm Energy
					Matrix3f warpJacobianTranspose = warpJacobian.t();
					float localSmoothnessEnergy = dot(warpJacobian.getColumn(0), warpJacobian.getColumn(0)) +
					                              dot(warpJacobian.getColumn(1), warpJacobian.getColumn(1)) +
					                              dot(warpJacobian.getColumn(2), warpJacobian.getColumn(2));
					float localKillingEnergy = localSmoothnessEnergy +
					                           gamma *
					                           (dot(warpJacobianTranspose.getColumn(0), warpJacobian.getColumn(0)) +
					                            dot(warpJacobianTranspose.getColumn(1), warpJacobian.getColumn(1)) +
					                            dot(warpJacobianTranspose.getColumn(2), warpJacobian.getColumn(2)));

					//=================================== FINAL UPDATE =================================================
					const float weightKilling = ITMSceneMotionTracker<TVoxel, TIndex>::weightKillingTerm;
					//_DEBUG
					//const float weightKilling = 0.5;
					const float weightLevelSet = ITMSceneMotionTracker<TVoxel, TIndex>::weightLevelSetTerm;
					const float learningRate = ITMSceneMotionTracker<TVoxel, TIndex>::gradientDescentLearningRate;
					//_DEBUG
					Vector3f deltaE = deltaEData;
					//Vector3f deltaE = deltaEData + weightKilling * deltaEKilling;
					//Vector3f deltaE = deltaEData + weightLevelSet * deltaELevelSet;
					//Vector3f deltaE = -deltaEData + weightLevelSet * deltaELevelSet + weightKilling * deltaEKilling;
					Vector3f warpUpdate = learningRate * deltaE;
					float vecLength = length(warpUpdate);

					//_DEBUG
					float killingLength = length(deltaEKilling);

					//need thread lock here to ensure atomic updates to maxVectorUpdate
#ifdef WITH_OPENMP
#pragma omp critical(maxVectorUpdate)
#endif
					{
						if (maxVectorUpdate < vecLength) {
							maxVectorUpdate = vecLength;
						}
						if (maxKillingUpdateLength < killingLength) {
							maxKillingUpdateLength = killingLength;
							maxKillingUpdate = weightKilling * deltaEKilling;
							trackedDataUpdate = deltaEData;
						}
					};

					canonicalVoxel.warp_t_update = TO_SHORT_FLOOR3((warpUpdate * FLOAT_TO_SHORT_CONVERSION_FACTOR));

					//debug stats
					aveCanonicaSdf += canonicalSdf;
					aveLiveSdf += liveSdf;
					consideredVoxelCount += 1;
					totalDataEnergy += (diffSdf * diffSdf);
					totalLevelSetEnergy += weightLevelSet * 0.5 * (sdfJacobianNormMinusOne * sdfJacobianNormMinusOne);
					totalKillingEnergy += weightKilling * localKillingEnergy;
					totalSmoothnessEnergy += weightKilling * localSmoothnessEnergy;
					aveWarpDist += length(canonicalVoxel.warp_t);
					aveSdfDiff += diffSdf;

					//80280 252 4, 7, 3 -1.53724, -1.53811, 0
					//_DEBUG
//					if(std::isnan(deltaE.x) || std::isnan(deltaE.y) || std::isnan(deltaE.z)){
//						std::cout << entryId << " " << locId << " " << Vector3i(x,y,z) << " " << deltaE << std::endl;
//						DIEWITHEXCEPTION("NAN encountered");
//					}
					//voxel is 5mm, so 10 voxels is 5 cm
					//if (deltaE.x > 10.0f || deltaE.y > 10.0f || deltaE.z > 10.0f) {
					//std::cout << entryId << " " << locId << std::endl;
					//}
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
	//Apply the update

	//_DEBUG
	//Warp Update Length Histogram
	// <20%, 40%, 60%, 80%, 100%
	const int histBinCount = 10;
	int bins[histBinCount] = {0};

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		const ITMHashEntry& currentCanonicalHashEntry = canonicalHashTable[entryId];

		if (currentCanonicalHashEntry.ptr < 0) continue;
		TVoxel* localVoxelBlock = &(canonicalVoxels[currentCanonicalHashEntry.ptr * (SDF_BLOCK_SIZE3)]);


		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxel& canonicalVoxel = localVoxelBlock[locId];
					//_DEBUG
					Vector3f update = TO_FLOAT3(canonicalVoxel.warp_t_update) / FLOAT_TO_SHORT_CONVERSION_FACTOR;
					float updateLength = length(update);
					int binIdx = 0;
					if(maxVectorUpdate > 0){
						binIdx = std::min(histBinCount - 1, (int) (updateLength * histBinCount / maxVectorUpdate));
					}
					bins[binIdx]++;
					canonicalVoxel.warp_t -= update;
					//END _DEBUG
					//canonicalVoxel.warp_t -= TO_FLOAT3(canonicalVoxel.warp_t_update) / FLOAT_TO_SHORT_CONVERSION_FACTOR;

				}
			}
		}
	}
	//_DEBUG
	aveCanonicaSdf /= consideredVoxelCount;
	aveLiveSdf /= consideredVoxelCount;
	aveWarpDist /= consideredVoxelCount;
	aveSdfDiff /= consideredVoxelCount;
	totalEnergy = totalDataEnergy + totalLevelSetEnergy + totalKillingEnergy;
	const std::string red("\033[0;31m");
	const std::string green("\033[0;32m");
	const std::string yellow("\033[0;33m");
	const std::string cyan("\033[0;36m");
	const std::string reset("\033[0m");
//	std::cout << " Max Killing update: " << maxKillingUpdate << " Corresp. data update: " << trackedDataUpdate << std::endl;
	std::cout << " [ENERGY] Data term: " << totalDataEnergy
	          << " Level set term: " << totalLevelSetEnergy << cyan
	          << " Smoothness term: " << totalSmoothnessEnergy << yellow
	          << " Killing term: " << totalKillingEnergy << reset
	          << " Total: " << totalEnergy << green
	          << " No Killing: " << totalDataEnergy + totalLevelSetEnergy << reset
	          << " No Level Set: " << totalDataEnergy + totalKillingEnergy;
	std::cout << std::endl << " Ave canonical SDF: " << aveCanonicaSdf << " Ave live SDF: " << aveLiveSdf
	          << " Ave sdf difference: " << aveSdfDiff << " Considered voxel count: " << consideredVoxelCount
	          << " Ave warp distance: " << aveWarpDist;
	//_DEBUG
//	std::cout << std::endl;
//	for(int iBin =0 ; iBin < histBinCount; iBin++){
//		std::cout << std::setfill(' ') << std::setw(7) << bins[iBin] << "  ";
//	}
	return maxVectorUpdate;
}

template<typename TVoxel, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxel, TIndex>::FuseFrame(ITMScene<TVoxel, TIndex>* canonicalScene,
                                                          ITMScene<ITMVoxelAux, TIndex>* liveScene) {
	TVoxel* canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();
	typename TIndex::IndexCache canonicalCache;

	const ITMVoxelAux* liveVoxels = liveScene->localVBA.GetVoxelBlocks();
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
					canonicalVoxel.w_depth = (uchar) newWDepth;
					canonicalVoxel.clr = TO_UCHAR3(newColor * 255.0f);
					canonicalVoxel.w_color = (uchar) newWColor;
					canonicalVoxel.confidence += liveConfidence;
				}
			}
		}
	}

}

template<typename TVoxel, typename TIndex>
ITMSceneMotionTracker_CPU<TVoxel, TIndex>::ITMSceneMotionTracker_CPU(const ITMSceneParams& params)
		: ITMSceneMotionTracker<TVoxel, TIndex>(params) {}
