//  ================================================================
//  Created by Gregory Kramida on 12/20/17.
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
#pragma once



//stdlib
#include <chrono>

//local
#include "ITMSceneMotionTracker_CPU.h"
#include "../Shared/ITMSceneMotionTracker_Shared.h"


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
//#define OPENMP_WARP_UPDATE_COMPUTE_DISABLE
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
	for (int hashEntryId = 0; hashEntryId < noTotalEntries; hashEntryId++) {
		Vector3i canonicalHashEntryPosition;
		const ITMHashEntry& currentCanonicalHashEntry = canonicalHashTable[hashEntryId];
		if (currentCanonicalHashEntry.ptr < 0) continue;
		//position of the current entry in 3D space
		canonicalHashEntryPosition = currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		//_DEBUG
		//std::cout << std::endl << "HASH POS: " << currentCanonicalHashEntry.pos << ": " << hashEntryId << std::endl;
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
					//almost no restriction -- Mira's case with addition of UNKNOWN flag checking
					bool markedUnknownInCanonical = canonicalVoxel.flags == ITMLib::UNKNOWN;
					bool emptyInCanonical = markedUnknownInCanonical || 1.0f - std::abs(canonicalSdf) < epsilon;
					bool emptyInLive = 1.0f - std::abs(liveSdf) < epsilon;

					if (emptyInCanonical && emptyInLive) continue;

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
					Vector3f deltaEData = Vector3f(0.0f);
					Vector3f deltaELevelSet = Vector3f(0.0f);
					float diffSdf = 0.0f;
					float sdfJacobianNormMinusOne = 0.0f;
					//for voxels in canonical that are unknown, we completely disregard the data and level set terms:
					//there is no information
					if(!markedUnknownInCanonical){
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
						//=================================== DATA TERM ====================================================
						//Compute data term error / energy
						diffSdf = liveSdf - canonicalSdf;

						deltaEData = liveSdfJacobian * diffSdf;
						if (useColor) {
							float diffColor =
									ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::weightColorDataTerm *
									squareDistance(liveColor, TO_FLOAT3(canonicalVoxel.clr) / 255.f);
							deltaEData += liveColorJacobian * diffColor;
						}

						//=================================== LEVEL SET TERM ===============================================
						float sdfJacobianNorm = length(liveSdfJacobian);
						sdfJacobianNormMinusOne = sdfJacobianNorm - 1.0f;
						deltaELevelSet =
								sdfJacobianNormMinusOne * (liveSdfHessian * liveSdfJacobian) /
								(sdfJacobianNorm + ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::epsilon);
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
					//BEGIN _DEBUG
					float warpLength = ORUtils::length(canonicalVoxel.warp_t - warpUpdate);
					float warpUpdateToggle = ORUtils::length(canonicalVoxel.warp_t_update + warpUpdate);
					float warpUpdateDiff = ORUtils::length(canonicalVoxel.warp_t_update - warpUpdate);
					//TODO: this is a bad way to do convergence. Use something like Adam instead, maybe? --Greg
					//TODO: figure out the exact conditions causing these oscillations, maybe nothing fancy is necessary here --Greg(GitHub: Algomorph)
					if (warpUpdateToggle < 0.01 && warpUpdateDiff > 0.05) {
						//We think that an oscillation has been detected
#ifdef LOG_HIGHLIGHTS
						int& currentFrame = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::currentFrameIx;
						const int& frameOfInterest = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::frameOfInterest;
						int& currentIteration = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::iteration;
						if(currentFrame == frameOfInterest){
							ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::sceneLogger
									.LogHighlight(hashEntryId,locId,currentFrame,currentIteration);
						}
#endif
#ifdef OLD_UGLY_WAY
						warpUpdate *= 0.5;//magic! -UNDO THE MAGIC FOR DEBUGGING OSCILLATIONS FURTHER
#endif
					}
					//END _DEBUG

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
#if defined(PRINT_DEBUG_HISTOGRAM) || defined(PRINT_MAX_WARP_AND_UPDATE)
					float warpLength = ORUtils::length(canonicalVoxel.warp_t);
					float warpUpdateLength = ORUtils::length(canonicalVoxel.warp_t_update);
#endif
#ifdef PRINT_MAX_WARP_AND_UPDATE
					if (maxWarpLength > 0.0f && warpLength == maxWarpLength) {
						std::cout << " Max warp pos: "
						          << currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE + Vector3i(x, y, z);
					}

					if (maxWarpUpdateLength > 0.0f && warpUpdateLength == maxWarpUpdateLength) {
						std::cout << " Max update pos: "
						          << currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE + Vector3i(x, y, z);
					}
#endif //PRINT_MAX_WARP_AND_UPDATE
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
#ifdef PRINT_MAX_WARP_AND_UPDATE
	std::cout << "Max warp: " << maxWarpLength << " Max update: " << maxWarpUpdateLength << std::endl;
#endif
	//end _DEBUG
	return maxWarpUpdateLength;

}

