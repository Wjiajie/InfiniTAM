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
#include <iomanip>

//local
#include "ITMSceneMotionTracker_CPU.h"
#include "../Shared/ITMSceneMotionTracker_Shared.h"
#include "../../Utils/ITMVoxelFlags.h"
//_DEBUG
#include "../Shared/ITMSceneMotionTracker_Debug.h"

#ifdef OLD_LEVEL_SET_TERM
#include "../Shared/ITMSceneMotionTracker_Deprecated.h"
#endif


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

	const bool hasFocusCoordinates = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::hasFocusCoordinates;
	Vector3i focusCoordinates = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::focusCoordinates;

	//_DEBUG
#ifdef _DEBUG
#ifdef PRINT_ADDITIONAL_STATS
	double aveCanonicaSdf = 0.0;
	double aveLiveSdf = 0.0;
	double aveSdfDiff = 0.0;
	int consideredVoxelCount = 0;
	int dataVoxelCount = 0;
	int levelSetVoxelCount = 0;
	double aveWarpDist = 0.0;

	int voxelOscillationCount = 0;
	int ignoredVoxelOscillationCount = 0;
#ifndef WITH_OPENMP
//_DEBUG
	std::vector<std::tuple<int, int>> voxelOscillations;
#endif // WITH_OPENMP
#endif // PRINT_ADDITIONAL_STATS

#ifdef PRINT_ENERGY_STATS
	double totalDataEnergy = 0.0;
	double totalLevelSetEnergy = 0.0;
	double totalSmoothnessEnergy = 0.0;
	double totalKillingEnergy = 0.0;
	double totalEnergy = 0.0;
#endif

	float maxWarpLength = 0.0;
#endif

	const float epsilon = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::epsilon;
	const int iteration = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::iteration;
	// fraction of narrow band (non-truncated region) half-width that a voxel spans
	const float unity = liveScene->sceneParams->voxelSize / liveScene->sceneParams->mu;

	//_DEBUG
	std::vector<Vector3i> _DEBUG_positions = {Vector3i(3, 36, 207 ), Vector3i(4, 36, 207 ), Vector3i(2, 37, 212 ), Vector3i(3, 37, 212 ), Vector3i(2, 37, 213 ), Vector3i(3, 37, 213 ), Vector3i(4, 37, 213 ), Vector3i(5, 37, 213 ), Vector3i(7, 0, 192 ), Vector3i(7, 0, 193 ), Vector3i(7, 0, 194 ), Vector3i(7, 0, 195)};


			//compute the update, don't apply yet (computation depends on previous warp for neighbors,
	//no practical way to keep those buffered with the hash & multithreading in mind)
#ifdef WITH_OPENMP
#ifndef OPENMP_WARP_UPDATE_COMPUTE_DISABLE

#if defined(PRINT_ADDITIONAL_STATS) && defined(PRINT_ENERGY_STATS)
#pragma omp parallel for firstprivate(canonicalCache, liveCache) reduction(+:aveCanonicaSdf, consideredVoxelCount, dataVoxelCount, levelSetVoxelCount, aveLiveSdf, aveWarpDist, aveSdfDiff, totalDataEnergy, totalLevelSetEnergy, totalSmoothnessEnergy, totalKillingEnergy, voxelOscillationCount, ignoredVoxelOscillationCount)
#else
#if defined(PRINT_ADDITIONAL_STATS)
#pragma omp parallel for firstprivate(canonicalCache, liveCache) reduction(+:aveCanonicaSdf, consideredVoxelCount, dataVoxelCount, levelSetVoxelCount, aveLiveSdf, aveWarpDist, aveSdfDiff, voxelOscillationCount, ignoredVoxelOscillationCount)
#elif defined(PRINT_ENERGY_STATS)
#pragma omp parallel for firstprivate(canonicalCache, liveCache) reduction(+:totalDataEnergy, totalLevelSetEnergy, totalSmoothnessEnergy, totalKillingEnergy)
#else
#pragma omp parallel for firstprivate(canonicalCache, liveCache)
#endif
#endif //both sets of debug vars

#endif// ndef OPENMP_WARP_UPDATE_COMPUTE_DISABLE
#else

#endif// WITH_OPENMP


	for (int hash = 0; hash < noTotalEntries; hash++) {
		Vector3i canonicalHashEntryPosition;
		const ITMHashEntry& currentCanonicalHashEntry = canonicalHashTable[hash];
		if (currentCanonicalHashEntry.ptr < 0) continue;
		//position of the current entry in 3D space
		canonicalHashEntryPosition = currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		TVoxelCanonical* localVoxelBlock = &(canonicalVoxels[currentCanonicalHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					//position of the voxel in the canonical frame
					Vector3i canonicalVoxelPosition = canonicalHashEntryPosition + Vector3i(x, y, z);
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxelCanonical& canonicalVoxel = localVoxelBlock[locId];
					//_DEBUG
//					if(canonicalVoxel.flags == VOXEL_SKIP_OPT){
//						continue;
//					}
					float canonicalSdf = TVoxelCanonical::valueToFloat(canonicalVoxel.sdf);

					//=================================== TRUNCATION REGION CHECKS =====================================
					float liveSdf;
					bool hitLiveNonTruncated;
					bool hitLiveKnownVoxels;
					Vector3f projectedPosition = canonicalVoxelPosition.toFloat() + canonicalVoxel.warp_t;

					liveSdf = _DEBUG_InterpolateTrilinearly_SetUnknownToVal_StruckChecks(
							liveVoxels, liveHashTable, canonicalSdf, projectedPosition, liveCache,
							hitLiveNonTruncated, hitLiveKnownVoxels);

					//almost no restriction -- Mira's case with addition of VOXEL_TRUNCATED flag checking
					bool truncatedInCanonical = canonicalVoxel.flags == ITMLib::VOXEL_TRUNCATED;
					//_DEBUG
//					if(truncatedInCanonical && !hitLiveNonTruncated && liveSdf != canonicalSdf){
//						canonicalVoxel.flags = VOXEL_SKIP_OPT;
//						continue;
//					}
//					if (iteration > 164 && ORUtils::length(canonicalVoxel.warp_t) > 3.0f){
//						std::cout << "Position: " << canonicalVoxelPosition << " C-to-L: " << canonicalSdf << " : " << liveSdf << std::endl;
//					}

//					if ((iteration == 0 || iteration == 165) &&
//							std::find(_DEBUG_positions.begin(),_DEBUG_positions.end(), canonicalVoxelPosition) != _DEBUG_positions.end()){
//						std::cout << "Position: " << canonicalVoxelPosition << " C-to-L: " << canonicalSdf << " : " << liveSdf << std::endl;
//					}
					bool knownInCanonical = canonicalSdf != TVoxelCanonical::SDF_initialValue();

					// the latter condition needs to be included since sometimes, even if some live voxels in the lookup
					// neighborhood are non-truncated, they ally may be a whole voxel away from the warped position,
					// which would then result in a live SDF lookup equivalent to that in a truncated region.
					bool truncatedInLive = !hitLiveNonTruncated || (1.0 - std::abs(liveSdf) < FLT_EPSILON2);

					// The data term is only relevant if we know the actual sdf values in both the canonical and the
					// live frame, since it's based on the comparison between them
					//_DEBUG
					//bool computeDataTerm = knownInCanonical && hitLiveKnownVoxels && enableDataTerm;
					bool computeDataTerm = knownInCanonical && !truncatedInCanonical && hitLiveNonTruncated && enableDataTerm;

					//_DEBUG
					//bool computeLevelSetTerm = !truncatedInCanonical && !truncatedInLive;
					//bool computeLevelSetTerm = (!knownInCanonical || !truncatedInCanonical) && !truncatedInLive;
					// level set term can be computed if we'bbre in non-truncated live space
					bool computeLevelSetTerm = !truncatedInLive && enableLevelSetTerm;
					bool computeKillingTerm = enableKillingTerm;
					Vector3f& warp = canonicalVoxel.warp_t;

					bool printVoxelResult = false;
					bool recordVoxelResult = false;
					if (hasFocusCoordinates && canonicalVoxelPosition == focusCoordinates) {
						liveSdf = _DEBUG_InterpolateTrilinearly_SetUnknownToVal_StruckChecks(
								liveVoxels, liveHashTable, canonicalSdf, projectedPosition, liveCache,
								hitLiveNonTruncated, hitLiveKnownVoxels);
						std::cout << std::endl << bright_cyan << "*** Printing voxel at " << canonicalVoxelPosition
						          << " *** " << reset << std::endl;
						std::cout << "Source SDF vs. target SDF: " << canonicalSdf << "-->" << liveSdf << std::endl
						          << "Warp: " << green << canonicalVoxel.warp_t << reset
						          << " length: " << green << ORUtils::length(canonicalVoxel.warp_t) << reset;
						std::cout << " Struck live narrow band: " << (hitLiveNonTruncated ? green : red)
						          << (hitLiveNonTruncated ? "true" : "false") << reset;
						std::cout << std::endl;
						printVoxelResult = true;
						if (ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::sceneLogger != nullptr) {
							recordVoxelResult = true;
						}
					}
					//================================ RETRIEVE NEIGHBOR'S WARPS =======================================
					const int neighborhoodSize = 9;
					Vector3f neighborWarps[neighborhoodSize];
					bool neighborAllocated[neighborhoodSize];
					bool neighborTruncated[neighborhoodSize];
					//    0        1        2          3         4         5           6         7         8
					//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
					findPoint2ndDerivativeNeighborhoodWarp(neighborWarps/*x9*/, neighborAllocated, neighborTruncated,
					                                       canonicalVoxelPosition, canonicalVoxels, canonicalHashTable,
					                                       canonicalCache);
					for (int iNeighbor = 0; iNeighbor < neighborhoodSize; iNeighbor++) {
						if (!neighborAllocated[iNeighbor]) {
							neighborWarps[iNeighbor] = warp;
						}
					}
					// =============================== DECLARATIONS & DEFAULTS FOR ALL TERMS ===========================
					Vector3f deltaEKilling(0.0f);
					Vector3f deltaEData(0.0f);
					Vector3f deltaELevelSet(0.0f);
					float localKillingEnergy = 0.0f;
					float localSmoothnessEnergy = 0.0f;

					float diffSdf = 0.0f, sdfJacobianNormMinusUnity = 0.0f;

					bool useColor = false;
					Matrix3f warpedSdfHessian;
					Vector3f liveColor, liveSdfJacobian, liveColorJacobian, liveSdf_Center_WarpForward, warpedSdfJacobian;

					if (computeDataTerm || computeLevelSetTerm) {
//						ComputeLiveSdf_Center_WarpForward_SetUnknown(canonicalVoxelPosition, warp, 1.0f, liveVoxels,
//						                                  liveHashTable, liveCache, liveSdf_Center_WarpForward);

//						ComputeLiveSdf_Center_WarpForward_SetTruncated(canonicalVoxelPosition, warp, liveSdf,
//						                                               liveVoxels,
//						                                               liveHashTable, liveCache,
//						                                               liveSdf_Center_WarpForward);

						ComputeLiveSdf_Center_WarpForward_SetUnknown(canonicalVoxelPosition, warp, liveSdf,
						                                               liveVoxels,
						                                               liveHashTable, liveCache,
						                                               liveSdf_Center_WarpForward);

						if (printVoxelResult) {
							std::cout << std::endl << "Live SDF at warp plus one for each direction: " << yellow
							          << liveSdf_Center_WarpForward << reset << std::endl;
						}
					}
					if (computeDataTerm) {
						//=================================== DATA TERM ================================================
#ifdef USE_COLOR
						if (std::abs(canonicalSdf) >
							ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::colorSdfThreshold) {
							useColor = false;
#else
						{

#endif

							ComputePerPointLiveJacobian(canonicalVoxelPosition, warp, canonicalSdf,
							                            liveVoxels, liveHashTable, liveCache,
							                            liveSdf, liveSdfJacobian,
							                            liveSdf_Center_WarpForward);


						}
#ifdef USE_COLOR
						else {
							useColor = true;
							ComputePerPointWarpedLiveJacobian
									(canonicalVoxelPosition, canonicalVoxel.warp_t,
									 liveVoxels, liveHashTable, liveCache,
									 liveSdf, liveColor, liveSdfJacobian,
									 liveSdf_Center_WarpForward, liveColorJacobian);
							if(printVoxelResult){
								PrintPerPointWarpedLiveJacobian(liveSdfJacobian, liveSdf_Center_WarpForward);
								std::cout << "Back-projected SDF Jacobian: " << warpedSdfJacobian << std::endl << std::endl;
							}
						}
#endif//USE_COLOR
						//Compute data term error / energy
						diffSdf = liveSdf - canonicalSdf;
						deltaEData = liveSdfJacobian * diffSdf;
#ifdef USE_COLOR
						if (useColor) {
							float diffColor =
									ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::weightColorDataTerm *
											squareDistance(liveColor, TO_FLOAT3(canonicalVoxel.clr) / 255.f);
							deltaEData += liveColorJacobian * diffColor;
						}
#endif//USE_COLOR
#ifdef PRINT_ADDITIONAL_STATS
						dataVoxelCount++;
						aveSdfDiff += std::abs(diffSdf);
#endif
					}// computeDataTerm
					else if (printVoxelResult) {
						//need to print a few blank lines to make sure the printing stays consistent height
						std::cout << std::endl << "Data term not computed." << std::endl
						          << "Known in canonical: " << (knownInCanonical ? "true" : "false") << std::endl
						          << "Hit known voxels in live: " << (hitLiveKnownVoxels ? "true" : "false")
						          << std::endl << std::endl;
					}
					if (computeLevelSetTerm) {
						//=================================== LEVEL SET TERM ===============================================

						Vector3f jacobianNormsAtWarpPlusOne;
						ComputeWarpedJacobianAndHessian(neighborWarps, canonicalVoxelPosition, canonicalSdf,
						                                liveSdf_Center_WarpForward, liveSdf, liveVoxels,
						                                liveHashTable, liveCache, warpedSdfJacobian,
						                                warpedSdfHessian);

						float sdfJacobianNorm = length(warpedSdfJacobian);
						sdfJacobianNormMinusUnity = sdfJacobianNorm - unity;

						//BEGIN _DEBUG
						liveSdfJacobian = liveSdf_Center_WarpForward - Vector3f(liveSdf);
						float liveSdfJacobianNorm = length(liveSdfJacobian);
						float liveJacobianNormMinusUnity = liveSdfJacobianNorm - unity;
						//END _DEBUG

						deltaELevelSet =
								sdfJacobianNormMinusUnity * (warpedSdfHessian * warpedSdfJacobian) /
								(sdfJacobianNorm + epsilon);
						if (printVoxelResult)
							_DEBUG_PrintLevelSetTermStuff(liveSdfJacobian,
							                              liveSdf_Center_WarpForward,
							                              warpedSdfJacobian,
							                              warpedSdfHessian);
#ifdef PRINT_ADDITIONAL_STATS
						levelSetVoxelCount++;
#endif
					} else {
						if (printVoxelResult) {
							//need to print a few blank lines to make sure the printing stays consistent height
							std::cout << std::endl << "Level set term not computed." << std::endl
							          << "Truncated in canonical: " << (truncatedInCanonical ? "true" : "false")
							          << std::endl
							          << "Considered truncated in live: " << (truncatedInLive ? "true" : "false")
							          << std::endl << std::endl;
						}
					}
					Matrix3f warpJacobian(0.0f);
					Matrix3f warpHessian[3] = {Matrix3f(0.0f),Matrix3f(0.0f),Matrix3f(0.0f)};
					if (computeKillingTerm) {
						//=================================== KILLING TERM =================================================


						ComputePerVoxelWarpJacobianAndHessian(canonicalVoxel.warp_t, canonicalVoxelPosition,
						                                      neighborWarps,
						                                      warpJacobian, warpHessian);
						if (printVoxelResult) {
							_DEBUG_PrintKillingTermStuff(neighborWarps, neighborAllocated, neighborTruncated,
							                             warpJacobian, warpHessian);
						}

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
								-2.0f *
								((onePlusGamma) * H_u.xx + (H_u.yy) + (H_u.zz) + gamma * H_v.xy + gamma * H_w.xz);
						KillingDeltaEv =
								-2.0f *
								((onePlusGamma) * H_v.yy + (H_v.zz) + (H_v.xx) + gamma * H_u.xy + gamma * H_w.yz);
						KillingDeltaEw =
								-2.0f *
								((onePlusGamma) * H_w.zz + (H_w.xx) + (H_w.yy) + gamma * H_v.yz + gamma * H_u.xz);


						deltaEKilling = Vector3f(KillingDeltaEu,
						                         KillingDeltaEv,
						                         KillingDeltaEw);
#ifdef PRINT_ENERGY_STATS
						//=================================== KILLING ENERGY =======================================================
						// KillingTerm Energy
						Matrix3f warpJacobianTranspose = warpJacobian.t();

						localSmoothnessEnergy = dot(warpJacobian.getColumn(0), warpJacobian.getColumn(0)) +
						                        dot(warpJacobian.getColumn(1), warpJacobian.getColumn(1)) +
						                        dot(warpJacobian.getColumn(2), warpJacobian.getColumn(2));

						localKillingEnergy = localSmoothnessEnergy +
						                     gamma *
						                     (dot(warpJacobianTranspose.getColumn(0), warpJacobian.getColumn(0)) +
						                      dot(warpJacobianTranspose.getColumn(1), warpJacobian.getColumn(1)) +
						                      dot(warpJacobianTranspose.getColumn(2), warpJacobian.getColumn(2)));
#endif
					}

					//=================================== FINAL UPDATE =================================================
					const float weightKilling = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::weightKillingTerm;
					const float weightLevelSet = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::weightLevelSetTerm;
					const float learningRate = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::gradientDescentLearningRate;

					Vector3f deltaE = deltaEData + weightLevelSet * deltaELevelSet + weightKilling * deltaEKilling;

					Vector3f warpUpdate = learningRate * deltaE;
					float warpUpdateLength = length(warpUpdate);//meters
					//BEGIN _DEBUG
					float warpLength = ORUtils::length(canonicalVoxel.warp_t - warpUpdate);

					double dataEnergy = 0.5 * (diffSdf * diffSdf);
					double levelSetEnergy =
							weightLevelSet * 0.5 * (sdfJacobianNormMinusUnity * sdfJacobianNormMinusUnity);
					double killingEnergy = weightKilling * localKillingEnergy;
					double smoothnessEnergy = weightKilling * localSmoothnessEnergy;
					double totalVoxelEnergy = dataEnergy + levelSetEnergy + killingEnergy * smoothnessEnergy;

					totalDataEnergy += dataEnergy;
					totalLevelSetEnergy += levelSetEnergy;
					totalKillingEnergy += killingEnergy;
					totalSmoothnessEnergy += smoothnessEnergy;
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
					if (printVoxelResult) {
						std::cout << "Data update: " << deltaEData * -1.f;
						std::cout << red << " Level set update: " << deltaELevelSet * -1.f;
						std::cout << yellow << " Killing update: " << deltaEKilling * -1.f;
						std::cout << std::endl;
						std::cout << green << "Warp update: " << warpUpdate * -1.f << reset;
						std::cout << " Warp update length: " << warpUpdateLength << std::endl << std::endl;

						if (recordVoxelResult) {
							const int& currentFrame = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::currentFrameIx;

							Vector3f canonicalWarp = canonicalVoxel.warp_t;
							Vector3f warpUpdateData = learningRate * deltaEData;
							Vector3f warpUpdateLevelSet = learningRate * weightLevelSet * deltaELevelSet;
							Vector3f warpUpdateKilling = learningRate * weightKilling * deltaEKilling;
							std::array<ITMNeighborVoxelIterationInfo, 9> neighbors;
							FindHighlightNeighborInfo(neighbors, canonicalVoxelPosition, hash, canonicalVoxels,
							                          canonicalHashTable, liveVoxels, liveHashTable, liveCache);
							ITMHighlightIterationInfo info = {hash, locId, currentFrame, iteration,
							                                  canonicalVoxelPosition,
							                                  canonicalWarp, canonicalSdf, liveSdf,
							                                  warpUpdate, warpUpdateData, warpUpdateLevelSet,
							                                  warpUpdateKilling, totalVoxelEnergy, dataEnergy,
							                                  levelSetEnergy, smoothnessEnergy, killingEnergy,
							                                  liveSdfJacobian, warpedSdfJacobian, warpedSdfHessian,
							                                  warpJacobian, warpHessian[0], warpHessian[1],
							                                  warpHessian[2],
							                                  neighbors, true};
							ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::sceneLogger->
									LogHighlight(hash, locId, currentFrame, info);
						}
					}

					//debug stats
#ifdef PRINT_ADDITIONAL_STATS
					aveCanonicaSdf += canonicalSdf;
					aveLiveSdf += liveSdf;

					aveWarpDist += ORUtils::length(canonicalVoxel.warp_t);

					consideredVoxelCount += 1;
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
#if defined(PRINT_MAX_WARP_AND_UPDATE) || defined(PRINT_DEBUG_HISTOGRAM) || defined(PRINT_ADDITIONAL_STATS)
	std::cout << bright_cyan << "*** General Iteration Statistics ***" << reset << std::endl;
#endif
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

					canonicalVoxel.warp_t -= canonicalVoxel.warp_t_update;
#if defined(PRINT_DEBUG_HISTOGRAM) || defined(PRINT_MAX_WARP_AND_UPDATE)
					float warpLength = ORUtils::length(canonicalVoxel.warp_t);
					float warpUpdateLength = ORUtils::length(canonicalVoxel.warp_t_update);
#endif
#ifdef PRINT_MAX_WARP_AND_UPDATE
					if (maxWarpLength > 0.0f && warpLength == maxWarpLength) {
						std::cout << " Max warp pos: " << red
						          << currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE + Vector3i(x, y, z)
						          << reset;
					}

					if (maxWarpUpdateLength > 0.0f && warpUpdateLength == maxWarpUpdateLength) {
						std::cout << " Max update pos: " << green
						          << (currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE + Vector3i(x, y, z))
						          << reset;
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
	std::cout << std::endl;
	std::cout << " [ENERGY] Data term: " << totalDataEnergy << red
	          << " Level set term: " << totalLevelSetEnergy << cyan
	          << " Smoothness term: " << totalSmoothnessEnergy << yellow
	          << " Killing term: " << totalKillingEnergy << green
	          << " Total: " << totalEnergy << reset;
//	          << " No Killing: " << totalDataEnergy + totalLevelSetEnergy << reset
//	          << " No Level Set: " << totalDataEnergy + totalKillingEnergy;
	std::cout << std::endl;
#ifdef WRITE_ENERGY_STATS_TO_FILE
	std::ofstream& energy_stat_file = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::energy_stat_file;
	energy_stat_file << totalDataEnergy << ", " << totalLevelSetEnergy << ", " << totalSmoothnessEnergy << ", "
	                 << totalKillingEnergy << ", " << totalEnergy << std::endl;
#endif
#endif
#ifdef PRINT_ADDITIONAL_STATS
	//_DEBUG
	aveCanonicaSdf /= consideredVoxelCount;
	aveLiveSdf /= consideredVoxelCount;
	aveWarpDist /= consideredVoxelCount;
	aveSdfDiff /= dataVoxelCount;
	std::cout //<< " Ave canonical SDF: " << aveCanonicaSdf
			//<< " Ave live SDF: " << aveLiveSdf
			<< " Ave SDF diff: " << aveSdfDiff
			<< " Used voxel count: " << consideredVoxelCount
			<< " Used for data term: " << dataVoxelCount
			<< " Used for LS term: " << levelSetVoxelCount
		//<< " Ave warp distance: " << aveWarpDist
			;
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
	std::cout << "  Warp length histogram: ";
	for (int iBin = 0; iBin < histBinCount; iBin++) {
		std::cout << std::setfill(' ') << std::setw(7) << warpBins[iBin] << "  ";
	}
	std::cout << std::endl;
	std::cout << "Update length histogram: ";
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

