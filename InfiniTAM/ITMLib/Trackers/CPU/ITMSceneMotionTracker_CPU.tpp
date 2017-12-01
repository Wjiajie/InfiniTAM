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
//#define PRINT_TIME_STATS
#define PRINT_SINGLE_VOXEL_RESULT
#define PRINT_MAX_WARP
#define PRINT_ENERGY_STATS
#define PRINT_ADDITIONAL_STATS
#define PRINT_DEBUG_HISTOGRAM
#define WARP_BOUNDARY_SPECIAL_TREATMENT
//#define OPENMP_WARP_UPDATE_COMPUTE_DISABLE

//stdlib
#include <cmath>
#include <iomanip>

//_DEBUG -- OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <unordered_set>
#include <chrono>


//local
#include "ITMSceneMotionTracker_CPU.h"
#include "../Shared/ITMSceneMotionTracker_Shared.h"
#include "../../ITMLibDefines.h"

//START _DEBUG
#define TIC(var)\
    auto start_##var = std::chrono::steady_clock::now();

#define TOC(var)\
    auto end_##var = std::chrono::steady_clock::now();\
    auto diff_##var = end_##var - start_##var;\
    var += std::chrono::duration <double, std::milli> (diff_##var).count();
//end _DEBUG

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

	//_DEBUG
#define _DEBUG
#ifdef _DEBUG
#ifdef PRINT_ADDITIONAL_STATS
	double aveCanonicaSdf = 0.0;
	double aveLiveSdf = 0.0;
	double aveSdfDiff = 0.0;
	int consideredVoxelCount = 0;
	double aveWarpDist = 0.0;
	double aveWarpDistBoundary = 0.0;
	int boundaryVoxelCount = 0;
#endif
	float maxWarpLength = 0.0;
#ifdef PRINT_ENERGY_STATS
	double totalDataEnergy = 0.0;
	double totalLevelSetEnergy = 0.0;
	double totalSmoothnessEnergy = 0.0;
	double totalKillingEnergy = 0.0;
	double totalEnergy = 0.0;
#endif

	const std::string red("\033[0;31m");
	const std::string green("\033[0;32m");
	const std::string yellow("\033[0;33m");
	const std::string cyan("\033[0;36m");
	const std::string reset("\033[0m");

	//debug image stuff
	const int imgRangeStartX = -262;
	const int imgRangeEndX = 138;
	const int imgRangeStartY = -98;
	const int imgRangeEndY = 338;
	const int imgZSlice = 559;

	const int imgVoxelRangeX = imgRangeEndX - imgRangeStartX;
	const int imgVoxelRangeY = imgRangeEndY - imgRangeStartY;

	const float imgToVoxelScale = 4.0;

	const int imgPixelRangeX = static_cast<int>(imgToVoxelScale * imgVoxelRangeX);
	const int imgPixelRangeY = static_cast<int>(imgToVoxelScale * imgVoxelRangeY);

	cv::Mat source = cv::Mat::ones(imgPixelRangeX, imgPixelRangeY, CV_32F);
	cv::Mat target = cv::Mat::ones(imgPixelRangeX, imgPixelRangeY, CV_32F);
	cv::Mat warped = cv::Mat::ones(imgPixelRangeX, imgPixelRangeY, CV_32F);


#endif

	const float epsilon = ITMSceneMotionTracker<TVoxel, TIndex>::epsilon;

	TIC(timeWarpUpdateCompute);

	//compute the update, don't apply yet (computation depends on previous warp for neighbors,
	// no practical way to keep those buffered with the hash in mind)
#ifdef WITH_OPENMP
#ifndef OPENMP_WARP_UPDATE_COMPUTE_DISABLE

#if defined(PRINT_ADDITIONAL_STATS) && defined(PRINT_ENERGY_STATS)
#pragma omp parallel for firstprivate(canonicalCache, liveCache) reduction(+:aveCanonicaSdf, consideredVoxelCount, aveLiveSdf, aveWarpDist, aveSdfDiff, boundaryVoxelCount, aveWarpDistBoundary, totalDataEnergy, totalLevelSetEnergy, totalSmoothnessEnergy, totalKillingEnergy)
#else
#if defined(PRINT_ADDITIONAL_STATS)
#pragma omp parallel for firstprivate(canonicalCache, liveCache) reduction(+:aveCanonicaSdf, consideredVoxelCount, aveLiveSdf, aveWarpDist, aveSdfDiff, boundaryVoxelCount, aveWarpDistBoundary)
#elif defined(PRINT_ENERGY_STATS)
#pragma omp parallel for firstprivate(canonicalCache, liveCache) reduction(+:totalDataEnergy, totalLevelSetEnergy, totalSmoothnessEnergy, totalKillingEnergy)
#else
#pragma omp parallel for firstprivate(canonicalCache, liveCache)
#endif
#endif //both sets of debug vars

#endif// ndef OPENMP_WARP_UPDATE_COMPUTE_DISABLE
#endif// WITH_OPENMP
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
					Vector3f projectedPosition = originalPosition.toFloat() + canonicalVoxel.warp_t;
					liveSdf = interpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition, liveCache);
					float canonicalSdf = TVoxel::valueToFloat(canonicalVoxel.sdf);

					//_DEBUG
					//almost no restriction -- Mira's case
					bool emptyInCanonical = 1.0f - std::abs(canonicalSdf) < epsilon;
					bool emptyInLive = 1.0f - std::abs(liveSdf) < epsilon;
					if (emptyInCanonical && emptyInLive) continue;

					if (emptyInCanonical) {
						canonicalSdf = std::copysign(canonicalSdf, liveSdf);
					} else if (emptyInLive) {
						liveSdf = std::copysign(liveSdf, canonicalSdf);
					}

					bool useColor;

					Vector3f liveColor, liveSdfJacobian, liveColorJacobian;
					Matrix3f liveSdfHessian;
					//_DEBUG
					bool boundary, printResult = false;
#ifdef PRINT_SINGLE_VOXEL_RESULT
					if (originalPosition == (ITMSceneMotionTracker<TVoxel, TIndex>::testPos1)) {
						printResult = true;
						std::cout << std::endl << "Source SDF vs. target SDF: " << canonicalSdf
								  << "-->" << liveSdf << std::endl << "Warp: " << canonicalVoxel.warp_t << std::endl;
					}
#endif
#ifdef PRINT_TIME_STATS
					TIC(timeDataJandHCompute);
#endif
					if (std::abs(canonicalSdf) > ITMSceneMotionTracker<TVoxel, TIndex>::colorSdfThreshold) {
						useColor = false;
#ifndef PRINT_SINGLE_VOXEL_RESULT
						ComputePerPointWarpedLiveJacobianAndHessian<TVoxel, TIndex, typename TIndex::IndexCache>
								(originalPosition, canonicalVoxel.warp_t,
								 canonicalVoxels, canonicalHashTable, canonicalCache,
								 liveVoxels, liveHashTable, liveCache,
								 liveSdf, liveSdfJacobian, liveSdfHessian);
#else
						//_DEBUG
						ComputePerPointWarpedLiveJacobianAndHessianAlt<TVoxel, TIndex, typename TIndex::IndexCache>
								(originalPosition, canonicalVoxel.warp_t,
								 canonicalVoxels, canonicalHashTable, canonicalCache,
								 liveVoxels, liveHashTable, liveCache,
								 liveSdf, liveSdfJacobian, liveSdfHessian, printResult);
#endif
					} else {
						useColor = true;
						ComputePerPointWarpedLiveJacobianAndHessian<TVoxel, TIndex, typename TIndex::IndexCache>
								(originalPosition, canonicalVoxel.warp_t,
								 canonicalVoxels, canonicalHashTable, canonicalCache,
								 liveVoxels, liveHashTable, liveCache,
								 liveSdf, liveColor, liveSdfJacobian, liveColorJacobian, liveSdfHessian);
					}
#ifdef PRINT_TIME_STATS
					TOC(timeDataJandHCompute);
					TIC(timeWarpJandHCompute);
#endif
					Matrix3f warpJacobian;
					Matrix3f warpHessian[3];// = {Matrix3f(), Matrix3f(), Matrix3f()};
#ifndef WARP_BOUNDARY_SPECIAL_TREATMENT
					ComputePerPointWarpJacobianAndHessian<TVoxel, TIndex, typename TIndex::IndexCache>(
							canonicalVoxel.warp_t, originalPosition, canonicalVoxels, canonicalHashTable,
							canonicalCache, warpJacobian, warpHessian);
					//_DEBUG --only one of the above/below statements should remain in the end, prob. the one below
#else
					ComputePerPointWarpJacobianAndHessianBoundaries2<TVoxel, TIndex, typename TIndex::IndexCache>(
							canonicalVoxel.warp_t, originalPosition, canonicalVoxels, canonicalHashTable,
							canonicalCache, warpJacobian, warpHessian, boundary, printResult);
#ifdef PRINT_ADDITIONAL_STATS
					if (boundary) boundaryVoxelCount++;
#endif
#endif
#ifdef PRINT_TIME_STATS
					TOC(timeWarpJandHCompute);
					TIC(timeUpdateTermCompute);
#endif
					//=================================== DATA TERM ====================================================
					//Compute data term error / energy
					float diffSdf = liveSdf - canonicalSdf;

					Vector3f deltaEData = liveSdfJacobian * diffSdf;
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
#ifdef COMPUTE_KILLING_OLD

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
					Vector3f deltaEKillingOld = -2.0f *
											 (warpHessian[0] * stackedVector0 +
											  warpHessian[1] * stackedVector1 +
											  warpHessian[2] * stackedVector2);

#endif
					// |0, 3, 6|     |m00, m10, m20|      |u_xx, u_xy, u_xz|
					// |1, 4, 7|     |m01, m11, m21|      |u_xy, u_yy, u_yz|
					// |2, 5, 8|     |m02, m12, m22|      |u_xz, u_yz, u_zz|
					Matrix3f& H_u = warpHessian[0];
					Matrix3f& H_v = warpHessian[1];
					Matrix3f& H_w = warpHessian[2];
					float KillingDeltaEu, KillingDeltaEv, KillingDeltaEw;

//					if(boundary){
//						KillingDeltaEu = -2.0f*(H_u.xx + H_u.yy + H_u.zz);
//						KillingDeltaEv = -2.0f*(H_v.yy + H_v.zz + H_v.xx);
//						KillingDeltaEw = -2.0f*(H_w.zz + H_w.xx + H_w.yy);
//					}else{
						KillingDeltaEu = -2.0f*((onePlusGamma)*H_u.xx + (H_u.yy) + (H_u.zz) + gamma*H_v.xy + gamma*H_w.xz);
						KillingDeltaEv = -2.0f*((onePlusGamma)*H_v.yy + (H_v.zz) + (H_v.xx) + gamma*H_u.xy + gamma*H_w.yz);
						KillingDeltaEw = -2.0f*((onePlusGamma)*H_w.zz + (H_w.xx) + (H_w.yy) + gamma*H_v.yz + gamma*H_u.xz);
//					}

					Vector3f deltaEKilling = Vector3f(KillingDeltaEu,
					                                  KillingDeltaEv,
					                                  KillingDeltaEw);
#ifdef PRINT_TIME_STATS
					TOC(timeUpdateTermCompute);
#endif
					//_DEBUG
#ifdef PRINT_ENERGY_STATS
					//=================================== ENERGY =======================================================
					// KillingTerm Energy
					Matrix3f warpJacobianTranspose = warpJacobian.t();

					float localSmoothnessEnergy = dot(warpJacobian.getColumn(0), warpJacobian.getColumn(0)) +
					                              dot(warpJacobian.getColumn(1), warpJacobian.getColumn(1)) +
					                              dot(warpJacobian.getColumn(2), warpJacobian.getColumn(2));

//					float localKillingEnergy = boundary? localSmoothnessEnergy : localSmoothnessEnergy +
//					                           gamma *
//					                           (dot(warpJacobianTranspose.getColumn(0), warpJacobian.getColumn(0)) +
//					                            dot(warpJacobianTranspose.getColumn(1), warpJacobian.getColumn(1)) +
//					                            dot(warpJacobianTranspose.getColumn(2), warpJacobian.getColumn(2)));
					float localKillingEnergy = localSmoothnessEnergy +
					                           gamma *
					                           (dot(warpJacobianTranspose.getColumn(0), warpJacobian.getColumn(0)) +
					                            dot(warpJacobianTranspose.getColumn(1), warpJacobian.getColumn(1)) +
					                            dot(warpJacobianTranspose.getColumn(2), warpJacobian.getColumn(2)));
#endif

					//=================================== FINAL UPDATE =================================================
					const float weightKilling = ITMSceneMotionTracker<TVoxel, TIndex>::weightKillingTerm;
					//_DEBUG
					//const float weightKilling = 0.5;
					const float weightLevelSet = ITMSceneMotionTracker<TVoxel, TIndex>::weightLevelSetTerm;
					const float learningRate = ITMSceneMotionTracker<TVoxel, TIndex>::gradientDescentLearningRate;
					//_DEBUG
					//Vector3f deltaE = deltaEData;
					//Vector3f deltaE = weightKilling * deltaEKilling;
					//Vector3f deltaE = deltaEData + weightKilling * deltaEKillingOld;
					//Vector3f deltaE = deltaEData + weightLevelSet * deltaELevelSet;
					Vector3f deltaE = deltaEData + weightLevelSet * deltaELevelSet + weightKilling * deltaEKilling;


					Vector3f warpUpdate = learningRate * deltaE;
					float vecLength = length(warpUpdate);
					//_DEBUG
					float warpLength = ORUtils::length(canonicalVoxel.warp_t - warpUpdate);

					//need thread lock here to ensure atomic updates to maxVectorUpdate
#ifdef WITH_OPENMP
#pragma omp critical(maxVectorUpdate)
#endif
					{
						if (maxVectorUpdate < vecLength) {
							maxVectorUpdate = vecLength;
						}
						if (maxWarpLength < warpLength) {
							maxWarpLength = warpLength;
						}
					};

					canonicalVoxel.warp_t_update = warpUpdate;
					//TODO: this doesn't work for values > 1.0 -Greg (GitHub: Algomorph)
					//canonicalVoxel.warp_t_update = TO_SHORT_FLOOR3((warpUpdate * FLOAT_TO_SHORT_CONVERSION_FACTOR));
#ifdef PRINT_SINGLE_VOXEL_RESULT
					if (printResult) {
						std::cout << "Data update: " << deltaEData;
						std::cout << " Level set update: " << deltaELevelSet;
						std::cout << " Killing update: " << deltaEKilling;
						std::cout << " Warp update: " << warpUpdate << std::endl << std::endl;
					}
#endif

					//debug stats
#ifdef PRINT_ADDITIONAL_STATS
					aveCanonicaSdf += canonicalSdf;
					aveLiveSdf += liveSdf;
					aveSdfDiff += diffSdf;
					aveWarpDist += ORUtils::length(canonicalVoxel.warp_t);
					if (boundary) {
						aveWarpDistBoundary += ORUtils::length(canonicalVoxel.warp_t);
					}
					consideredVoxelCount += 1;

#endif
#ifdef PRINT_ENERGY_STATS
					totalDataEnergy += 0.5 * (diffSdf * diffSdf);
					totalLevelSetEnergy += weightLevelSet * 0.5 * (sdfJacobianNormMinusOne * sdfJacobianNormMinusOne);
					totalKillingEnergy += weightKilling * localKillingEnergy;
					totalSmoothnessEnergy += weightKilling * localSmoothnessEnergy;
#endif

				}
			}
		}
	}
#ifdef PRINT_TIME_STATS
	TOC(timeWarpUpdateCompute);
#endif

	//Apply the update
#ifdef PRINT_DEBUG_HISTOGRAM
	//_DEBUG
	//Warp Update Length Histogram
	// <20%, 40%, 60%, 80%, 100%
	const int histBinCount = 10;
	int bins[histBinCount] = {0};
#endif
	TIC(timeWarpUpdateApply);
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
#ifdef PRINT_DEBUG_HISTOGRAM
					//_DEBUG
					//Vector3f update = TO_FLOAT3(canonicalVoxel.warp_t_update) / FLOAT_TO_SHORT_CONVERSION_FACTOR;
					Vector3f update = canonicalVoxel.warp_t_update;
					canonicalVoxel.warp_t -= update;
					float warpLength = ORUtils::length(canonicalVoxel.warp_t);
					int binIdx = 0;
#ifdef PRINT_MAX_WARP
					if(warpLength == maxWarpLength){
						std::cout << " Max warp pos: " << currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE + Vector3i(x,y,z);
					}
#endif
					if (maxVectorUpdate > 0) {
						binIdx = std::min(histBinCount - 1, (int) (warpLength * histBinCount / maxWarpLength));
					}
					bins[binIdx]++;
					//END _DEBUG -- restore one of next two lines, depending on whether the field as short conv. working/not
					//canonicalVoxel.warp_t -= TO_FLOAT3(canonicalVoxel.warp_t_update) / FLOAT_TO_SHORT_CONVERSION_FACTOR;
					//
#else
					canonicalVoxel.warp_t -= canonicalVoxel.warp_t_update;
#ifdef PRINT_MAX_WARP
					//_DEBUG
					float warpLength = ORUtils::length(canonicalVoxel.warp_t);
					if(warpLength == maxWarpLength){
						std::cout << " Max warp pos: " << currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE + Vector3i(x,y,z);
					}
#endif //PRINT_MAX_WARP
#endif //PRINT_DEBUG_HISTOGRAM

				}
			}
		}
	}

	TOC(timeWarpUpdateApply);

#ifdef PRINT_ENERGY_STATS
	totalEnergy = totalDataEnergy + totalLevelSetEnergy + totalKillingEnergy;

	std::cout << " [ENERGY] Data term: " << totalDataEnergy
	          << " Level set term: " << totalLevelSetEnergy << cyan
	          << " Smoothness term: " << totalSmoothnessEnergy << yellow
	          << " Killing term: " << totalKillingEnergy << green
	          << " Total: " << totalEnergy << reset;
//	          << " No Killing: " << totalDataEnergy + totalLevelSetEnergy << reset
//	          << " No Level Set: " << totalDataEnergy + totalKillingEnergy;
	std::cout << std::endl;
#endif
#ifdef PRINT_ADDITIONAL_STATS
	//_DEBUG
	aveCanonicaSdf /= consideredVoxelCount;
	aveLiveSdf /= consideredVoxelCount;
	aveWarpDist /= consideredVoxelCount;
	aveSdfDiff /= consideredVoxelCount;
	if (boundaryVoxelCount > 0) {
		aveWarpDistBoundary /= boundaryVoxelCount;
	}
	          std::cout << " Ave canonical SDF: " << aveCanonicaSdf
	          << " Ave live SDF: " << aveLiveSdf
	          << " Ave SDF diff: " << aveSdfDiff
	          << " Used voxel count: " << consideredVoxelCount
	          << " Ave warp distance: " << aveWarpDist;

	if (boundaryVoxelCount > 0) {
		std::cout << " Boundary voxel count: " << boundaryVoxelCount
		          << " Boundary ave w. dist.: " << aveWarpDistBoundary;
	}
	std::cout << std::endl;
#endif
	//start _DEBUG
#ifdef PRINT_MAX_WARP
	std::cout << "Max warp: " << maxWarpLength;
#endif
#ifdef PRINT_TIME_STATS
	std::cout << " Update compute time: " << timeWarpUpdateCompute
	          << " Update apply time: " << timeWarpUpdateApply
			  << " Data J&H time: " << timeDataJandHCompute
			  << " Warp J&H time: " << timeWarpJandHCompute
			  << " Update term time: " << timeUpdateTermCompute;
#endif
#ifdef PRINT_DEBUG_HISTOGRAM
	for(int iBin =0 ; iBin < histBinCount; iBin++){
		std::cout << std::setfill(' ') << std::setw(7) << bins[iBin] << "  ";
	}
#endif
	//end _DEBUG
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
		: ITMSceneMotionTracker<TVoxel, TIndex>(params) {

	cv::Mat source = cv::Mat::ones(imgPixelRangeX, imgPixelRangeY, CV_32F);
	cv::Mat target = cv::Mat::ones(imgPixelRangeX, imgPixelRangeY, CV_32F);
	cv::Mat warped = cv::Mat::ones(imgPixelRangeX, imgPixelRangeY, CV_32F);
}


template<typename TVoxel, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxel, TIndex>::MarkWarpedSceneImage(ITMScene<TVoxel, TIndex>* scene, cv::Mat& image,
                                                                     Vector3i position) {
	bool vmIndex;
	TVoxel voxel = readVoxel(scene->localVBA.GetVoxelBlocks(), scene->index.GetEntries(), position, vmIndex);
	Vector3f projectedPosition = position.toFloat() + voxel.warp_t;
	Vector3i projectedPositionFloored = projectedPosition.toIntFloor();
	if (!isVoxelInImgRange(projectedPositionFloored.x, projectedPositionFloored.y, position.z-1) &&
		!isVoxelInImgRange(projectedPositionFloored.x, projectedPositionFloored.y, position.z) &&
		!isVoxelInImgRange(projectedPositionFloored.x, projectedPositionFloored.y, position.z+1)) return;

	Vector2i imgCoords = getVoxelImgCoords(projectedPosition.x, projectedPosition.y);
	const int voxelOnImageSize = static_cast<int>(imgToVoxelScale);
	float sdfRepr;
	//sdfRepr = std::abs(voxel.sdf);
	sdfRepr = 1.0f;// - sdfRepr*.6f;

	//fill a pixel block with the source scene value
	for (int row = imgCoords.y; row < imgCoords.y + voxelOnImageSize / 2; row++) {
		for (int col = imgCoords.x; col < imgCoords.x + voxelOnImageSize / 2; col++) {
//#pragma omp critical(PixelUpdate)
			image.at<uchar>(row, col) = static_cast<uchar>(sdfRepr * 255.0f);
		}
	}
};

//START _DEBUG
template<typename TVoxel, typename TIndex>
template<typename TTVoxel>
cv::Mat ITMSceneMotionTracker_CPU<TVoxel, TIndex>::DrawWarpedSceneImageTemplated(ITMScene<TTVoxel, TIndex>* scene) {
#ifdef IMAGE_BLACK_BACKGROUND
	//use if default voxel SDF value is -1.0
	cv::Mat img = cv::Mat::zeros(imgPixelRangeX, imgPixelRangeY, CV_32F);
#else
	//use if default voxel SDF value is 1.0
	cv::Mat img = cv::Mat::ones(imgPixelRangeX, imgPixelRangeY, CV_32F);
#endif

	TTVoxel* voxelBlocks = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* canonicalHashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noTotalEntries;
	typename TIndex::IndexCache canonicalCache;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		Vector3i currentBlockPositionVoxels;
		const ITMHashEntry& currentHashEntry = canonicalHashTable[entryId];

		if (currentHashEntry.ptr < 0) continue;

		//position of the current entry in 3D space
		currentBlockPositionVoxels = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		if (!isVoxelBlockInImgRangeTolerance(currentBlockPositionVoxels, 5)) continue;

		TTVoxel* localVoxelBlock = &(voxelBlocks[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3i originalPosition = currentBlockPositionVoxels + Vector3i(x, y, z);
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TTVoxel& voxel = localVoxelBlock[locId];
					Vector3f projectedPosition = originalPosition.toFloat() + voxel.warp_t;
					Vector3i projectedPositionFloored = projectedPosition.toIntFloor();
					if (!isVoxelInImgRange(projectedPositionFloored.x, projectedPositionFloored.y,
					                       originalPosition.z))
						continue;
					//if (originalPosition == ITMSceneMotionTracker<TVoxel, TIndex>::testPos1) continue;

					Vector2i imgCoords = getVoxelImgCoords(projectedPosition.x, projectedPosition.y);
					const int voxelOnImageSize = static_cast<int>(imgToVoxelScale);
					float sdfRepr;
					sdfRepr = absFillingStrategy ? std::abs(voxel.sdf) : (voxel.sdf + 1.0) / 2.0;
//					const bool invert = absFillingStrategy;
//					if (invert) {
//						sdfRepr = 1.0f - sdfRepr * .6f;
//					} else {
//						sdfRepr = 0.4f + sdfRepr * 0.6f;
//					}

					//fill a pixel block with the source scene value
					for (int row = imgCoords.y; row < imgCoords.y + voxelOnImageSize / 2; row++) {
						for (int col = imgCoords.x; col < imgCoords.x + voxelOnImageSize / 2; col++) {

//#pragma omp critical(PixelUpdate)
							img.at<float>(row, col) = sdfRepr;
						}
					}
				}
			}
		}
	}

	return img;
}
//END _DEBUG

//START _DEBUG
template<typename TVoxel, typename TIndex>
template<typename TTVoxel>
cv::Mat ITMSceneMotionTracker_CPU<TVoxel, TIndex>::DrawSceneImage(ITMScene<TTVoxel, TIndex>* scene) {
#ifdef IMAGE_BLACK_BACKGROUND
	//use if default voxel SDF value is -1.0
	cv::Mat img = cv::Mat::zeros(imgPixelRangeX, imgPixelRangeY, CV_32F);
#else
	//use if default voxel SDF value is 1.0
	cv::Mat img = cv::Mat::ones(imgPixelRangeX, imgPixelRangeY, CV_32F);
#endif
	TTVoxel* voxelBlocks = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* canonicalHashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noTotalEntries;
	typename TIndex::IndexCache canonicalCache;

	std::unordered_set<float> valueSet = {};

	int numPixelsFilled = 0;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		Vector3i currentBlockPositionVoxels;
		const ITMHashEntry& currentHashEntry = canonicalHashTable[entryId];

		if (currentHashEntry.ptr < 0) continue;

		//position of the current entry in 3D space
		currentBlockPositionVoxels = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		if (!isVoxelBlockInImgRange(currentBlockPositionVoxels)) continue;

		TTVoxel* localVoxelBlock = &(voxelBlocks[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3i originalPosition = currentBlockPositionVoxels + Vector3i(x, y, z);
					if (!isVoxelInImgRange(originalPosition.x, originalPosition.y, originalPosition.z)) continue;

					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TTVoxel& voxel = localVoxelBlock[locId];
					Vector2i imgCoords = getVoxelImgCoords(originalPosition.x, originalPosition.y);
					const int voxelOnImageSize = static_cast<int>(imgToVoxelScale);
					for (int row = imgCoords.y; row < imgCoords.y + voxelOnImageSize; row++) {
						for (int col = imgCoords.x; col < imgCoords.x + voxelOnImageSize; col++) {
							float sdfRepr = absFillingStrategy ? std::abs(voxel.sdf) : (voxel.sdf + 1.0) / 2.0;
							img.at<float>(row, col) = sdfRepr;
#pragma omp critical(PixelUniquesUpdate)
							valueSet.insert(sdfRepr);
							//std::cout<< sdfRepr << std::endl;
							numPixelsFilled++;
						}
					}
				}
			}
		}
	}
	std::cout << "Filled " << numPixelsFilled << " pixels with " << valueSet.size() << " unique values" << std::endl;
	return img;
}

template<typename TVoxel, typename TIndex>
bool ITMSceneMotionTracker_CPU<TVoxel, TIndex>::isVoxelInImgRange(int x, int y, int z) {
	return (z == imgZSlice && x >= imgRangeStartX && x < imgRangeEndX && y > imgRangeStartY && y < imgRangeEndY);
}

template<typename TVoxel, typename TIndex>
Vector2i ITMSceneMotionTracker_CPU<TVoxel, TIndex>::getVoxelImgCoords(int x, int y) {
	return Vector2i(static_cast<int>(imgToVoxelScale * (x - imgRangeStartX)),
	                imgPixelRangeY - static_cast<int>(imgToVoxelScale * (y - imgRangeStartY)));
}

template<typename TVoxel, typename TIndex>
Vector2i ITMSceneMotionTracker_CPU<TVoxel, TIndex>::getVoxelImgCoords(float x, float y) {
	return Vector2i(static_cast<int>(imgToVoxelScale * (x - imgRangeStartX)),
	                imgPixelRangeY - static_cast<int>(imgToVoxelScale * (y - imgRangeStartY)));
}

template<typename TVoxel, typename TIndex>
bool ITMSceneMotionTracker_CPU<TVoxel, TIndex>::isVoxelBlockInImgRange(Vector3i blockVoxelCoords) {
	Vector3i& bvc0 = blockVoxelCoords;
	Vector3i bvc1 = blockVoxelCoords + Vector3i(SDF_BLOCK_SIZE);
	return !(imgZSlice >= bvc1.z || imgZSlice < bvc0.z) &&
	       !(imgRangeStartX >= bvc1.x || imgRangeEndX < bvc0.x) &&
	       !(imgRangeStartY >= bvc1.y || imgRangeEndY < bvc0.y);
}

template<typename TVoxel, typename TIndex>
bool
ITMSceneMotionTracker_CPU<TVoxel, TIndex>::isVoxelBlockInImgRangeTolerance(Vector3i blockVoxelCoords, int tolerance) {
	Vector3i& bvc0 = blockVoxelCoords;
	Vector3i bvc1 = blockVoxelCoords + Vector3i(SDF_BLOCK_SIZE);
	return !(imgZSlice >= bvc1.z || imgZSlice < bvc0.z) &&
	       !(imgRangeStartX - tolerance >= bvc1.x || imgRangeEndX + tolerance < bvc0.x) &&
	       !(imgRangeStartY - tolerance >= bvc1.y || imgRangeEndY + tolerance < bvc0.y);
}
//END _DEBUG

