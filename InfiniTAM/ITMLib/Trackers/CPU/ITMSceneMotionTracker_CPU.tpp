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

#define _DEBUG
#ifdef _DEBUG
//#define PRINT_TIME_STATS
//#define PRINT_SINGLE_VOXEL_RESULT
#define PRINT_MAX_WARP
#define PRINT_ENERGY_STATS
#define PRINT_ADDITIONAL_STATS
#define PRINT_DEBUG_HISTOGRAM

//#define OPENMP_WARP_UPDATE_COMPUTE_DISABLE
#endif //_DEBUG

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

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
float
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::UpdateWarpField(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
		ITMScene<TVoxelLive, TIndex>* liveScene) {

	TVoxelCanonical* canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();
	typename TIndex::IndexCache canonicalCache;

	const TVoxelLive* liveVoxels = liveScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* liveHashTable = liveScene->index.GetEntries();
	typename TIndex::IndexCache liveCache;

	int noTotalEntries = canonicalScene->index.noTotalEntries;
	float maxWarpUpdateLength = 0.0f;

	//_DEBUG
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

#ifdef PRINT_ENERGY_STATS
	double totalDataEnergy = 0.0;
	double totalLevelSetEnergy = 0.0;
	double totalSmoothnessEnergy = 0.0;
	double totalKillingEnergy = 0.0;
	double totalEnergy = 0.0;
#endif

	float maxWarpLength = 0.0;
	const std::string red("\033[0;31m");
	const std::string green("\033[0;32m");
	const std::string yellow("\033[0;33m");
	const std::string cyan("\033[0;36m");
	const std::string reset("\033[0m");
#endif

	const float epsilon = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::epsilon;

	TIC(timeWarpUpdateCompute);

	//compute the update, don't apply yet (computation depends on previous warp for neighbors,
	//no practical way to keep those buffered with the hash in mind)
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
		TVoxelCanonical* localVoxelBlock = &(canonicalVoxels[currentCanonicalHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3i originalPosition = canonicalHashEntryPosition + Vector3i(x, y, z);
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxelCanonical& canonicalVoxel = localVoxelBlock[locId];

					//=================================== PRELIMINARIES ================================================
					//Jacobian and Hessian of the live scene sampled at warped location + deltas,
					//as well as local Jacobian and Hessian of the warp field itself
					float liveSdf;
					Vector3f projectedPosition = originalPosition.toFloat() + canonicalVoxel.warp_t;
					bool liveSdfFound;
					liveSdf = interpolateTrilinearly(liveVoxels, liveHashTable, projectedPosition, liveCache,
					                                 liveSdfFound);
					float canonicalSdf = TVoxelCanonical::valueToFloat(canonicalVoxel.sdf);

					//_DEBUG
					//almost no restriction -- Mira's case
					bool emptyInCanonical = 1.0f - std::abs(canonicalSdf) < epsilon;
					bool emptyInLive = 1.0f - std::abs(liveSdf) < epsilon;
					if (emptyInCanonical && emptyInLive) continue;

					//_DEBUG
//					if (emptyInLive && !liveSdfFound) {
//						//liveSdf = std::copysign(1.0f, canonicalSdf);
//					}

					bool useColor;

					Vector3f liveColor, liveSdfJacobian, liveColorJacobian;
					Matrix3f liveSdfHessian;
					//_DEBUG
					bool boundary, printResult = false;
#ifdef PRINT_SINGLE_VOXEL_RESULT
					if (originalPosition == (ITMSceneSliceRasterizer<TVoxelCanonical, TVoxelLive, TIndex>::testPos1)) {
						printResult = true;
						std::cout << std::endl << "Priniting voxel at " << originalPosition << ". ";
						std::cout <<  "Source SDF vs. target SDF: " << canonicalSdf
								  << "-->" << liveSdf << std::endl << "Warp: " << canonicalVoxel.warp_t;
						std::cout << " Live SDF found: " << liveSdfFound;
						std::cout << std::endl;
					}
#endif
#ifdef PRINT_TIME_STATS
					TIC(timeDataJandHCompute);
#endif
					if (std::abs(canonicalSdf) >
					    ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::colorSdfThreshold) {
						useColor = false;
						ComputePerPointWarpedLiveJacobianAndHessian<TVoxelCanonical, TVoxelLive, TIndex, typename TIndex::IndexCache>
								(originalPosition, canonicalVoxel.warp_t,
								 canonicalVoxels, canonicalHashTable, canonicalCache,
								 liveVoxels, liveHashTable, liveCache,
								 liveSdf, liveSdfJacobian, liveSdfHessian, printResult);
					} else {
						useColor = true;
						ComputePerPointWarpedLiveJacobianAndHessian<TVoxelCanonical, TVoxelLive, TIndex, typename TIndex::IndexCache>
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
					Matrix3f warpHessian[3];

					ComputePerPointWarpJacobianAndHessian<TVoxelCanonical, TIndex, typename TIndex::IndexCache>(
							canonicalVoxel.warp_t, originalPosition, canonicalVoxels, canonicalHashTable,
							canonicalCache, warpJacobian, warpHessian, boundary, printResult);
#ifdef PRINT_ADDITIONAL_STATS
					if (boundary) boundaryVoxelCount++;
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
						float diffColor =
								ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::weightColorDataTerm *
								squareDistance(liveColor, TO_FLOAT3(canonicalVoxel.clr) / 255.f);
						deltaEData += liveColorJacobian * diffColor;
					}

					//=================================== LEVEL SET TERM ===============================================
					float sdfJacobianNorm = length(liveSdfJacobian);
					float sdfJacobianNormMinusOne = sdfJacobianNorm - 1.0f;
					Vector3f deltaELevelSet =
							sdfJacobianNormMinusOne * (liveSdfHessian * liveSdfJacobian) /
							(sdfJacobianNorm + ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::epsilon);

					//=================================== KILLING TERM =================================================
					const float gamma = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::rigidityEnforcementFactor;
					float onePlusGamma = 1.0f + gamma;
					// |0, 3, 6|     |m00, m10, m20|      |u_xx, u_xy, u_xz|
					// |1, 4, 7|     |m01, m11, m21|      |u_xy, u_yy, u_yz|
					// |2, 5, 8|     |m02, m12, m22|      |u_xz, u_yz, u_zz|
					Matrix3f& H_u = warpHessian[0];
					Matrix3f& H_v = warpHessian[1];
					Matrix3f& H_w = warpHessian[2];
					float KillingDeltaEu, KillingDeltaEv, KillingDeltaEw;


					KillingDeltaEu =
							-2.0f * ((onePlusGamma) * H_u.xx + (H_u.yy) + (H_u.zz) + gamma * H_v.xy + gamma * H_w.xz);
					KillingDeltaEv =
							-2.0f * ((onePlusGamma) * H_v.yy + (H_v.zz) + (H_v.xx) + gamma * H_u.xy + gamma * H_w.yz);
					KillingDeltaEw =
							-2.0f * ((onePlusGamma) * H_w.zz + (H_w.xx) + (H_w.yy) + gamma * H_v.yz + gamma * H_u.xz);


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

					float localKillingEnergy = localSmoothnessEnergy +
					                           gamma *
					                           (dot(warpJacobianTranspose.getColumn(0), warpJacobian.getColumn(0)) +
					                            dot(warpJacobianTranspose.getColumn(1), warpJacobian.getColumn(1)) +
					                            dot(warpJacobianTranspose.getColumn(2), warpJacobian.getColumn(2)));
#endif

					//=================================== FINAL UPDATE =================================================
					const float weightKilling = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::weightKillingTerm;
					const float weightLevelSet = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::weightLevelSetTerm;
					const float learningRate = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::gradientDescentLearningRate;;
					Vector3f deltaE = deltaEData + weightLevelSet * deltaELevelSet + weightKilling * deltaEKilling;


					Vector3f warpUpdate = learningRate * deltaE;
					float warpUpdateLength = length(warpUpdate);//meters
					//_DEBUG
					float warpLength = ORUtils::length(canonicalVoxel.warp_t - warpUpdate);
					float warpUpdateToggle = ORUtils::length(canonicalVoxel.warp_t_update + warpUpdate);
					float warpUpdateDiff = ORUtils::length(canonicalVoxel.warp_t_update - warpUpdate);
					//TODO: this is a bad way to do convergence. Use something like Adam instead, maybe? --Greg
					if (warpUpdateToggle < 0.01 && warpUpdateDiff > 0.05) {
						warpUpdate *= 0.5;
					}

					//need thread lock here to ensure atomic updates to maxWarpUpdateLength
#ifdef WITH_OPENMP
#pragma omp critical(maxVectorUpdate)
#endif
					{
						if (maxWarpUpdateLength < warpUpdateLength) {
							maxWarpUpdateLength = warpUpdateLength;
						}
						if (maxWarpLength < warpLength) {
							maxWarpLength = warpLength;
						}
					};

					canonicalVoxel.warp_t_update = warpUpdate;
					//TODO: this (below) doesn't work for values > 1.0 -Greg (GitHub: Algomorph)
					//canonicalVoxel.warp_t_update = TO_SHORT_FLOOR3((warpUpdate * FLOAT_TO_SHORT_CONVERSION_FACTOR));

#ifdef PRINT_SINGLE_VOXEL_RESULT
					if (printResult) {
						std::cout << "Data update: " << deltaEData;
						std::cout << " Level set update: " << deltaELevelSet;
						std::cout << " Killing update: " << deltaEKilling;
						std::cout << std::endl;
						std::cout << "Warp update: " << warpUpdate;
						std::cout << " Warp update length: " << warpUpdateLength << std::endl << std::endl;
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
	int warpBins[histBinCount] = {0};
	int updateBins[histBinCount] = {0};
#endif
	TIC(timeWarpUpdateApply);
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		const ITMHashEntry& currentCanonicalHashEntry = canonicalHashTable[entryId];

		if (currentCanonicalHashEntry.ptr < 0) continue;
		TVoxelCanonical* localVoxelBlock = &(canonicalVoxels[currentCanonicalHashEntry.ptr * (SDF_BLOCK_SIZE3)]);


		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxelCanonical& canonicalVoxel = localVoxelBlock[locId];

					//_DEBUG TODO:
					//canonicalVoxel.warp_t -= TO_FLOAT3(canonicalVoxel.warp_t_update) / FLOAT_TO_SHORT_CONVERSION_FACTOR;
					//END _DEBUG --  short conv. working/not
					canonicalVoxel.warp_t -= canonicalVoxel.warp_t_update;
#if defined(PRINT_DEBUG_HISTOGRAM) || defined(PRINT_MAX_WARP)
					float warpLength = ORUtils::length(canonicalVoxel.warp_t);
					float warpUpdateLength = ORUtils::length(canonicalVoxel.warp_t_update);
#endif
#ifdef PRINT_MAX_WARP
					if (warpLength == maxWarpLength) {
						std::cout << " Max warp pos: "
						          << currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE + Vector3i(x, y, z);
					}

					if (warpUpdateLength == maxWarpUpdateLength) {
						std::cout << " Max update pos: "
						          << currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE + Vector3i(x, y, z);
					}
#endif //PRINT_MAX_WARP
#ifdef PRINT_DEBUG_HISTOGRAM
					int binIdx = 0;
					if (maxWarpLength > 0) {
						binIdx = std::min(histBinCount - 1, (int) (warpLength * histBinCount / maxWarpLength));
					}
					warpBins[binIdx]++;
					if (maxWarpUpdateLength > 0) {
						binIdx = std::min(histBinCount - 1,
						                  (int) (warpUpdateLength * histBinCount / maxWarpUpdateLength));
					}
					updateBins[binIdx]++;

#endif //PRINT_DEBUG_HISTOGRAM

				}
			}
		}
	}
#ifdef PRINT_TIME_STATS
	TOC(timeWarpUpdateApply);
#endif

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

#ifdef PRINT_TIME_STATS
	std::cout << " Update compute time: " << timeWarpUpdateCompute
			  << " Update apply time: " << timeWarpUpdateApply
			  << " Data J&H time: " << timeDataJandHCompute
			  << " Warp J&H time: " << timeWarpJandHCompute
			  << " Update term time: " << timeUpdateTermCompute;
#endif
#ifdef PRINT_DEBUG_HISTOGRAM
	std::cout << "Warp length histogram: ";
	for (int iBin = 0; iBin < histBinCount; iBin++) {
		std::cout << std::setfill(' ') << std::setw(7) << warpBins[iBin] << "  ";
	}
	std::cout << std::endl;
	std::cout << "Warp length histogram: ";
	for (int iBin = 0; iBin < histBinCount; iBin++) {
		std::cout << std::setfill(' ') << std::setw(7) << updateBins[iBin] << "  ";
	}
	std::cout << std::endl;
#endif
#ifdef PRINT_MAX_WARP
	std::cout << "Max warp: " << maxWarpLength;
#endif
	//end _DEBUG
	return maxWarpUpdateLength;

}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::FuseFrame(ITMScene<TVoxelCanonical,
		TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {
	TVoxelCanonical* canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();
	typename TIndex::IndexCache canonicalCache;

	const TVoxelLive* liveVoxels = liveScene->localVBA.GetVoxelBlocks();
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

		TVoxelCanonical* localVoxelBlock = &(canonicalVoxels[currentCanonicalHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector3i originalPosition = canonicalHashEntryPosition + Vector3i(x, y, z);
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxelCanonical& canonicalVoxel = localVoxelBlock[locId];
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
					                                       liveColor, liveWDepth, liveWColor);

					float newSdf = oldWDepth * oldSdf + liveWDepth * liveSdf;
					float newWDepth = oldWDepth + liveWDepth;
					newSdf /= newWDepth;
					newWDepth = MIN(newWDepth, maxW);

					Vector3f newColor = oldWColor * oldColor + liveWColor * liveColor;
					float newWColor = oldWColor + liveWColor;
					newColor /= newWColor;
					newWColor = MIN(newWDepth, maxW);

					canonicalVoxel.sdf = TVoxelCanonical::floatToValue(newSdf);
					canonicalVoxel.w_depth = (uchar) newWDepth;
					canonicalVoxel.clr = TO_UCHAR3(newColor * 255.0f);
					canonicalVoxel.w_color = (uchar) newWColor;
					canonicalVoxel.confidence += liveConfidence;
				}
			}
		}
	}

}

//========================================== CONSTRUCTORS AND DESTRUCTORS ================================================
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ITMSceneMotionTracker_CPU(const ITMSceneParams& params)
		: ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>(params),
		  entriesAllocFill(new ORUtils::MemoryBlock<bool>(TIndex::noTotalEntries, MEMORYDEVICE_CPU)),
		  canonicalEntriesAllocType(
				  new ORUtils::MemoryBlock<unsigned char>(TIndex::noTotalEntries, MEMORYDEVICE_CPU)),
		  liveEntriesAllocType(
				  new ORUtils::MemoryBlock<unsigned char>(TIndex::noTotalEntries, MEMORYDEVICE_CPU)),
		  blockCoords(new ORUtils::MemoryBlock<Vector3s>(TIndex::noTotalEntries, MEMORYDEVICE_CPU)) {
	uchar* entriesAllocType = this->canonicalEntriesAllocType->GetData(MEMORYDEVICE_CPU);
	memset(entriesAllocType, ITMLib::NO_CHANGE, static_cast<size_t>(TIndex::noTotalEntries));

}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::~ITMSceneMotionTracker_CPU() {
	delete entriesAllocFill;
	delete canonicalEntriesAllocType;
	delete liveEntriesAllocType;
	delete blockCoords;
}

//========================================= END CONSTRUCTORS AND DESTRUCTORS============================================
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::AllocateNewCanonicalHashBlocks(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {
	uchar* entriesAllocType = this->canonicalEntriesAllocType->GetData(MEMORYDEVICE_CPU);
	AllocateBoundaryHashBlocks<TVoxelCanonical>(canonicalScene, entriesAllocType);
	entriesAllocType = this->liveEntriesAllocType->GetData(MEMORYDEVICE_CPU);
	/*TODO: determine whether the below code really needs to be there.
	 If not, remove the templated version of the AllocateBoundaryHashBlocks (unneded complexity due to template)
	 -Greg (GitHub: Algomorph)*/
	//memset(entriesAllocType, ITMLib::NO_CHANGE, static_cast<size_t>(TIndex::noTotalEntries));
//	AllocateNewCanonicalHashBlocks<TVoxelLive>(liveScene,entriesAllocType);
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
template<typename TVoxel>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::AllocateBoundaryHashBlocks(
		ITMScene<TVoxel, TIndex>* scene, uchar* entriesAllocType) {

	int entryCount = scene->index.noTotalEntries;
	Vector3s* blockCoords = this->blockCoords->GetData(MEMORYDEVICE_CPU);
	bool* entriesAllocFill = this->entriesAllocFill->GetData(MEMORYDEVICE_CPU);
	memset(entriesAllocFill, false, static_cast<size_t>(entryCount));

	const short neighborhoodSize = 3;//must be an odd positive integer greater than 1.
	const short neighborhoodRangeStart = -neighborhoodSize / 2;
	const short neighborhoodRangeEnd = neighborhoodSize / 2 + 1;

	TVoxel* canonicalVoxels = scene->localVBA.GetVoxelBlocks();
	ITMHashEntry* canonicalHashTable = scene->index.GetEntries();
	typename TIndex::IndexCache canonicalCache;

	//adjust as necessary for alternative neighborhoodSize
	Vector3i testVoxelLocations[27] = {Vector3i(0, 0, 0), Vector3i(3, 0, 0), Vector3i(7, 0, 0),
	                                   Vector3i(0, 3, 0), Vector3i(3, 3, 0), Vector3i(7, 3, 0),
	                                   Vector3i(0, 7, 0), Vector3i(3, 7, 0), Vector3i(7, 7, 0),

	                                   Vector3i(0, 0, 3), Vector3i(3, 0, 3), Vector3i(7, 0, 3),
	                                   Vector3i(0, 3, 3), Vector3i(3, 3, 3), Vector3i(7, 3, 3),
	                                   Vector3i(0, 7, 3), Vector3i(4, 7, 3), Vector3i(7, 7, 3),

	                                   Vector3i(0, 0, 7), Vector3i(3, 0, 7), Vector3i(7, 0, 7),
	                                   Vector3i(0, 3, 7), Vector3i(3, 3, 7), Vector3i(7, 3, 7),
	                                   Vector3i(0, 7, 7), Vector3i(4, 7, 7), Vector3i(7, 7, 7)};
	TVoxel positiveHashEntry[SDF_BLOCK_SIZE3];
	for (int iVoxel = 0; iVoxel < SDF_BLOCK_SIZE3; iVoxel++) {
		positiveHashEntry[iVoxel] = TVoxel();
		positiveHashEntry[iVoxel].sdf = 1.0f;
	}

	//TODO: need to modify visible type here? Or maybe, in the fusion step? -Greg (GitHub: Algomorph)

	int countVoxelHashBlocksToAllocate = 0;
#ifdef WITH_OPENMP
#pragma omp parallel for reduction(+:countVoxelHashBlocksToAllocate)
#endif
	for (int entryId = 0; entryId < entryCount; entryId++) {
		const ITMHashEntry& currentCanonicalHashEntry = canonicalHashTable[entryId];
		if (currentCanonicalHashEntry.ptr < 0) continue;
		TVoxel* localVoxelBlock = &(canonicalVoxels[currentCanonicalHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		Vector3s hashBlockCoords = currentCanonicalHashEntry.pos;
		int iNeighbor = 0;
		for (short x = neighborhoodRangeStart; x < neighborhoodRangeEnd; x++) {
			for (short y = neighborhoodRangeStart; y < neighborhoodRangeEnd; y++) {
				for (short z = neighborhoodRangeStart; z < neighborhoodRangeEnd; z++) {
					//compute neighbor hash block position
					if (x == 0 && y == 0 && z == 0) continue;
					Vector3s currentBlockLocation = hashBlockCoords + Vector3s(x, y, z);
					SDF_BLOCK_SIZE;
					//compute index in hash table
					int hashIdx = hashIndex(currentBlockLocation);
					unsigned char entryAllocType = entriesAllocType[hashIdx];

					if (entryAllocType == ITMLib::NO_CHANGE) {
						ComputeHashBlockAllocType(entriesAllocType, blockCoords, hashIdx,
						                          currentBlockLocation, canonicalHashTable, canonicalCache);
						if (entriesAllocType[hashIdx] == ITMLib::NEEDS_ALLOC_IN_ORDERED_LIST
						    || entriesAllocType[hashIdx] == ITMLib::NEEDS_ALLOC_IN_EXCESS_LIST) {
							Vector3i& testVoxelLocation = testVoxelLocations[iNeighbor];
							int locId = testVoxelLocation.x + testVoxelLocation.y * SDF_BLOCK_SIZE +
							            testVoxelLocation.z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
							entriesAllocFill[hashIdx] = localVoxelBlock[locId].sdf + 1.0f > FLT_EPSILON;

							//_DEBUG
							countVoxelHashBlocksToAllocate++;
						}
					}
					iNeighbor++;
				}
			}
		}
	}
	std::cout << "Number of boundary hash blocks needing allocation: "
	          << countVoxelHashBlocksToAllocate << " out of " << entryCount << std::endl;

	int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
	int lastFreeExcessListId = scene->index.GetLastFreeExcessListId();
	int* voxelAllocationList = scene->localVBA.GetAllocationList();
	int* excessAllocationList = scene->index.GetExcessAllocationList();
	for (int iTargetHashBlock = 0; iTargetHashBlock < entryCount; iTargetHashBlock++) {
		unsigned char entryAllocType = entriesAllocType[iTargetHashBlock];
		switch (entryAllocType) {
			case ITMLib::NEEDS_ALLOC_IN_ORDERED_LIST:

				if (lastFreeVoxelBlockId >= 0) //there is room in the voxel block array
				{
					ITMHashEntry hashEntry;
					hashEntry.pos == blockCoords[iTargetHashBlock];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
					if (entriesAllocFill[iTargetHashBlock]) {
						TVoxel* localVoxelBlock = &(canonicalVoxels[hashEntry.ptr * (SDF_BLOCK_SIZE3)]);
						memcpy(localVoxelBlock, &positiveHashEntry, SDF_BLOCK_SIZE3 * sizeof(TVoxel));
					}

					canonicalHashTable[iTargetHashBlock] = hashEntry;
					entriesAllocType[iTargetHashBlock] = ITMLib::BOUNDARY_STATE;
					lastFreeVoxelBlockId--;
				}

				break;
			case NEEDS_ALLOC_IN_EXCESS_LIST:

				if (lastFreeVoxelBlockId >= 0 &&
				    lastFreeExcessListId >= 0) //there is room in the voxel block array and excess list
				{
					ITMHashEntry hashEntry;
					hashEntry.pos = blockCoords[iTargetHashBlock];
					hashEntry.ptr = voxelAllocationList[lastFreeVoxelBlockId];
					hashEntry.offset = 0;
					if (entriesAllocFill[iTargetHashBlock]) {
						TVoxel* localVoxelBlock = &(canonicalVoxels[hashEntry.ptr * (SDF_BLOCK_SIZE3)]);
						memcpy(localVoxelBlock, &positiveHashEntry, SDF_BLOCK_SIZE3 * sizeof(TVoxel));
					}
					int exlOffset = excessAllocationList[lastFreeExcessListId];
					canonicalHashTable[iTargetHashBlock].offset = exlOffset + 1; //connect to child
					canonicalHashTable[SDF_BUCKET_NUM + exlOffset] = hashEntry; //add child to the excess list
					entriesAllocType[iTargetHashBlock] = ITMLib::BOUNDARY_STATE;
					lastFreeVoxelBlockId--;
					lastFreeExcessListId--;
				}
				break;
		}
	}
	scene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
	scene->index.SetLastFreeExcessListId(lastFreeExcessListId);
}


//END _DEBUG

