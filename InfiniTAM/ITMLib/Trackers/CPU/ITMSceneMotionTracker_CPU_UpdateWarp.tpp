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
//#include "../Shared/ITMSceneMotionTracker_Shared_Old.h"
#include "../Shared/ITMSceneMotionTracker_Shared.h"
#include "../../Utils/ITMVoxelFlags.h"
//_DEBUG
#include "../Shared/ITMSceneMotionTracker_Debug.h"
#include "../../Objects/Scene/ITMSceneManipulation.h"


using namespace ITMLib;

// region ==================================== STATIC PRINTING / STATISTICS FUNCTIONS ==================================

inline static
void PrintEnergyStatistics(const bool& enableDataTerm,
                           const bool& enableLevelSetTerm,
                           const bool& enableSmoothnessTerm,
                           const bool& useIsometryEnforcementFactorInSmoothingTerm,
                           const float& gamma,
                           const double& totalDataEnergy,
                           const double& totalLevelSetEnergy,
                           const double& totalTikhonovEnergy,
                           const double& totalKillingEnergy,
                           const double& totalSmoothnessEnergy,
                           const double& totalEnergy) {
	std::cout << " [ENERGY]";
	if (enableDataTerm) {
		std::cout << blue << " Data term: " << totalDataEnergy;
	}
	if (enableLevelSetTerm) {
		std::cout << red << " Level set term: " << totalLevelSetEnergy;
	}
	if (enableSmoothnessTerm) {
		if (useIsometryEnforcementFactorInSmoothingTerm) {
			std::cout << yellow << " Tikhonov term: " << totalTikhonovEnergy;
			std::cout << yellow << " Killing term: " << totalKillingEnergy;
		}
		std::cout << cyan << " Smoothness term: " << totalSmoothnessEnergy;
	}
	std::cout << green << " Total: " << totalEnergy << reset << std::endl;
}

inline static
void CalculateAndPrintAdditionalStatistics(const bool& enableDataTerm,
                                           const bool& enableLevelSetTerm,
                                           const double& cumulativeCanonicalSdf,
                                           const double& cumulativeLiveSdf,
                                           const double& cumulativeWarpDist,
                                           const double& cumulativeSdfDiff,
                                           const unsigned int& consideredVoxelCount,
                                           const unsigned int& dataVoxelCount,
                                           const unsigned int& levelSetVoxelCount) {

	double averageCanonicalSdf = cumulativeCanonicalSdf / consideredVoxelCount;
	double averageLiveSdf = cumulativeLiveSdf / consideredVoxelCount;
	double averageWarpDist = cumulativeWarpDist / consideredVoxelCount;
	double averageSdfDiff = 0.0;

	if (enableDataTerm) {
		averageSdfDiff = cumulativeSdfDiff / dataVoxelCount;
	}

	std::cout << " Ave canonical SDF: " << averageCanonicalSdf
	          << " Ave live SDF: " << averageLiveSdf;
	if (enableDataTerm) {
		std::cout << " Ave SDF diff: " << averageSdfDiff;
	}
	std::cout << " Used voxel count: " << consideredVoxelCount
	          << " Data term v-count: " << dataVoxelCount;
	if (enableLevelSetTerm) {
		std::cout << " LS term v-count: " << levelSetVoxelCount;
	}
	std::cout << " Ave warp distance: " << averageWarpDist;
	std::cout << std::endl;
}

// endregion ===========================================================================================================

// region ========================== CALCULATE WARP GRADIENT & UPDATE ==================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
float
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::CalculateWarpUpdate(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
		ITMScene<TVoxelLive, TIndex>* liveScene) {
#if (defined(VERBOSE_DEBUG) || defined(_DEBUG)) && !defined(WITH_OPENMP)
	return CalculateWarpUpdate_SingleThreadedVerbose(canonicalScene, liveScene);
#else
	return CalculateWarpUpdate_Multithreaded(canonicalScene,liveScene);
#endif
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
float ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::CalculateWarpUpdate_SingleThreadedVerbose(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {

	// TODO: better function separation by regions where applicable. Chances are, private void functions will be inlined anyway at compile time.

	// region DECLARATIONS / INITIALIZATIONS

	// *** statistical aggregates
	double cumulativeCanonicalSdf = 0.0;
	double cumulativeLiveSdf = 0.0;
	double cumulativeSdfDiff = 0.0;
	double cumulativeWarpDist = 0.0;
	unsigned int consideredVoxelCount = 0;
	unsigned int dataVoxelCount = 0;
	unsigned int levelSetVoxelCount = 0;

	double totalDataEnergy = 0.0;
	double totalLevelSetEnergy = 0.0;
	double totalTikhonovEnergy = 0.0;
	double totalKillingEnergy = 0.0;
	double totalSmoothnessEnergy = 0.0;
	double totalEnergy = 0.0;

	// *** for less verbose parameter & debug variable access
	// debug variables
	const bool hasFocusCoordinates = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::hasFocusCoordinates;
	Vector3i focusCoordinates = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::focusCoordinates;
	std::ofstream& energy_stat_file = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::energy_stat_file;
	// params
	const float epsilon = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::epsilon;
	const int iteration = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::iteration;
	const float weightSmoothnessTerm = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::weightKillingTerm;
	const float weightLevelSetTerm = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::weightLevelSetTerm;
	const float gamma = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::rigidityEnforcementFactor;
	// fraction of narrow band (non-Truncated region) half-width that a voxel spans
	const float unity = liveScene->sceneParams->voxelSize / liveScene->sceneParams->mu;

	// *** traversal vars
	//TODO: make canonical ones const
	TVoxelCanonical* canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();
	typename TIndex::IndexCache canonicalCache;


	TVoxelLive* liveVoxels = liveScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* liveHashTable = liveScene->index.GetEntries();
	int noTotalEntries = liveScene->index.noTotalEntries;
	typename TIndex::IndexCache liveCache;
	//endregion

	// *** traverse all allocated hash blocks & voxels
	// compute the update, don't apply yet (computation depends on previous warp for neighbors,
	// no practical way to keep those buffered with the hash & multithreading in mind)
	for (int hash = 0; hash < noTotalEntries; hash++) {
		ITMHashEntry& currentLiveHashEntry = liveHashTable[hash];
		if (currentLiveHashEntry.ptr < 0) continue;
		ITMHashEntry& currentCanonicalHashEntry = canonicalHashTable[hash];

		// the rare case where we have different positions for live & canonical voxel block with the same index:
		// we have a hash bucket miss, find the canonical voxel with the matching coordinates
		if (currentCanonicalHashEntry.pos != currentLiveHashEntry.pos) {
			int canonicalHash = hash;
			if (!FindHashAtPosition(canonicalHash, currentLiveHashEntry.pos, canonicalHashTable)) {
				DIEWITHEXCEPTION_REPORTLOCATION("Could not find corresponding canonical block at postion!");
			}
			currentCanonicalHashEntry = canonicalHashTable[canonicalHash];
		}
		// position of the current entry in 3D space in voxel units
		Vector3i hashBlockPosition = currentLiveHashEntry.pos.toInt() * SDF_BLOCK_SIZE;

		TVoxelLive* localLiveVoxelBlock = &(liveVoxels[currentLiveHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		TVoxelCanonical* localCanonicalVoxelBlock = &(canonicalVoxels[currentCanonicalHashEntry.ptr *
		                                                              (SDF_BLOCK_SIZE3)]);
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

					TVoxelLive& liveVoxel = localLiveVoxelBlock[locId];
					if (liveVoxel.flags != ITMLib::VOXEL_NONTRUNCATED) {
						continue;
					}
					// region =============================== DECLARATIONS & DEFAULTS FOR ALL TERMS ====================
					Vector3i voxelPosition = hashBlockPosition + Vector3i(x, y, z);
					TVoxelCanonical& canonicalVoxel = localCanonicalVoxelBlock[locId];
					float canonicalSdf = TVoxelCanonical::valueToFloat(canonicalVoxel.sdf);
					float liveSdf = TVoxelLive::valueToFloat(liveVoxel.sdf);
					Vector3f& warp = canonicalVoxel.warp;

					Vector3f localSmoothnessEnergyGradient(0.0f);
					Vector3f localDataEnergyGradient(0.0f);
					Vector3f localLevelSetEnergyGradient(0.0f);
					float localDataEnergy = 0.0f, localLevelSetEnergy = 0.0f, localSmoothnessEnergy = 0.0f,
							localTikhonovEnergy = 0.0f, localKillingEnergy = 0.0f;
					float sdfDifferenceBetweenLiveAndCanonical = 0.0f, sdfJacobianNormMinusUnity = 0.0f;
					Matrix3f liveSdfHessian;
					Vector3f liveSdfJacobian, warpLaplacian;
					Matrix3f warpJacobian(0.0f);
					Matrix3f warpHessian[3] = {Matrix3f(0.0f), Matrix3f(0.0f), Matrix3f(0.0f)};
					// endregion

					// region =============================== DEBUG PRINTING / RECORDING ===============================

					bool printVoxelResult = false;
					bool recordVoxelResult = false;
					if (hasFocusCoordinates && voxelPosition == focusCoordinates) {
						std::cout << std::endl << bright_cyan << "*** Printing voxel at " << voxelPosition
						          << " *** " << reset << std::endl;
						std::cout << "Source SDF vs. target SDF: " << canonicalSdf << "-->" << liveSdf << std::endl
						          << "Warp: " << green << canonicalVoxel.warp << reset
						          << "length: " << green << ORUtils::length(canonicalVoxel.warp) << reset;
						std::cout << std::endl;
						printVoxelResult = true;
						if (ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::sceneLogger != nullptr) {
							recordVoxelResult = true;
						}
					}
					// endregion

					// region ============================== RETRIEVE NEIGHBOR'S WARPS =================================

					const int neighborhoodSize = 9;
					Vector3f neighborWarps[neighborhoodSize];
					bool neighborKnown[neighborhoodSize];
					bool neighborTruncated[neighborhoodSize];
					bool neighborAllocated[neighborhoodSize];
					//    0        1        2          3         4         5           6         7         8
					//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
					findPoint2ndDerivativeNeighborhoodWarp(neighborWarps/*x9*/, neighborKnown, neighborTruncated,
					                                       neighborAllocated, voxelPosition, canonicalVoxels,
					                                       canonicalHashTable, canonicalCache);
					//TODO: revise this to reflect new realities
					for (int iNeighbor = 0; iNeighbor < neighborhoodSize; iNeighbor++) {
						if (!neighborAllocated[iNeighbor]) {
							//assign current warp to neighbor warp if the neighbor is not allocated
							neighborWarps[iNeighbor] = warp;
						}
					}
					//endregion

					if (enableLevelSetTerm || enableDataTerm) {
						// compute the gradient of the live frame, ∇φ_n(Ψ)
						ComputeLiveJacobian_CentralDifferences(liveSdfJacobian, voxelPosition, liveVoxels,
						                                       liveHashTable, liveCache);
					}

					// region =============================== DATA TERM ================================================

					if (enableDataTerm) {

						// Compute data term error / energy
						sdfDifferenceBetweenLiveAndCanonical = liveSdf - canonicalSdf;
						// (φ_n(Ψ)−φ_{global}) ∇φ_n(Ψ)
						// φ_n(Ψ) = φ_n(x+u, y+v, z+w), where u = u(x,y,z), v = v(x,y,z), w = w(x,y,z)
						// φ_{global} = φ_{global}(x, y, z)
						localDataEnergyGradient = sdfDifferenceBetweenLiveAndCanonical * liveSdfJacobian;

						dataVoxelCount++;
						cumulativeSdfDiff += std::abs(sdfDifferenceBetweenLiveAndCanonical);
						localDataEnergy =
								0.5f * (sdfDifferenceBetweenLiveAndCanonical * sdfDifferenceBetweenLiveAndCanonical);
					}
					// endregion

					// region =============================== LEVEL SET TERM ===========================================

					if (enableLevelSetTerm) {

						ComputeSdfHessian(liveSdfHessian, voxelPosition, liveVoxels, liveHashTable, liveCache);

						float sdfJacobianNorm = ORUtils::length(liveSdfJacobian);
						sdfJacobianNormMinusUnity = sdfJacobianNorm - unity;

						localLevelSetEnergyGradient =
								sdfJacobianNormMinusUnity * (liveSdfHessian * liveSdfJacobian) /
								(sdfJacobianNorm + epsilon);

						levelSetVoxelCount++;
						localLevelSetEnergy =
								weightLevelSetTerm * 0.5f * (sdfJacobianNormMinusUnity * sdfJacobianNormMinusUnity);
					}
					// endregion =======================================================================================

					// region =============================== SMOOTHING TERM (TIKHONOV & KILLING) ======================

					if (enableSmoothingTerm) {
						if (useIsometryEnforcementFactorInSmoothingTerm) {
							ComputePerVoxelWarpJacobianAndHessian(canonicalVoxel.warp, neighborWarps,
							                                      warpJacobian, warpHessian);
							if (printVoxelResult) {
								_DEBUG_PrintKillingTermStuff(neighborWarps, neighborKnown, neighborTruncated,
								                             warpJacobian, warpHessian);
							}


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


							localSmoothnessEnergyGradient = Vector3f(KillingDeltaEu,
							                                         KillingDeltaEv,
							                                         KillingDeltaEw);
							//=================================== ENERGY ===============================================
							// KillingTerm Energy
							Matrix3f warpJacobianTranspose = warpJacobian.t();

							localTikhonovEnergy = dot(warpJacobian.getColumn(0), warpJacobian.getColumn(0)) +
							                      dot(warpJacobian.getColumn(1), warpJacobian.getColumn(1)) +
							                      dot(warpJacobian.getColumn(2), warpJacobian.getColumn(2));

							localKillingEnergy = gamma *
							                     (dot(warpJacobianTranspose.getColumn(0), warpJacobian.getColumn(0)) +
							                      dot(warpJacobianTranspose.getColumn(1), warpJacobian.getColumn(1)) +
							                      dot(warpJacobianTranspose.getColumn(2), warpJacobian.getColumn(2)));

							localSmoothnessEnergy = localTikhonovEnergy + localKillingEnergy;
						} else {
							ComputeWarpLaplacianAndJacobian(warpLaplacian, warpJacobian, warp, neighborWarps);
							//∇E_{reg}(Ψ) = −[∆U ∆V ∆W]' ,
							localSmoothnessEnergyGradient = -warpLaplacian;
							localTikhonovEnergy = dot(warpJacobian.getColumn(0), warpJacobian.getColumn(0)) +
							                      dot(warpJacobian.getColumn(1), warpJacobian.getColumn(1)) +
							                      dot(warpJacobian.getColumn(2), warpJacobian.getColumn(2));
						}
					}
					// endregion

					// region =============================== COMPUTE ENERGY GRADIENT ==================================

					Vector3f localEnergyGradient =
							localDataEnergyGradient +
							weightLevelSetTerm * localLevelSetEnergyGradient +
							weightSmoothnessTerm * localSmoothnessEnergyGradient;

					canonicalVoxel.gradient0 = localEnergyGradient;

					// endregion

					// region =============================== AGGREGATE VOXEL STATISTICS ===============================
					float energyGradeintLength = ORUtils::length(localEnergyGradient);//meters
					float warpLength = ORUtils::length(canonicalVoxel.warp);
					double localEnergy = localDataEnergy + weightLevelSetTerm * localLevelSetEnergy
					                     + weightSmoothnessTerm * localSmoothnessEnergy;


					totalDataEnergy += localDataEnergy;
					totalLevelSetEnergy += weightLevelSetTerm * localLevelSetEnergy;
					totalTikhonovEnergy += weightSmoothnessTerm * localTikhonovEnergy;
					totalKillingEnergy += weightSmoothnessTerm * localKillingEnergy;
					totalSmoothnessEnergy += weightSmoothnessTerm * localSmoothnessEnergy;

					cumulativeCanonicalSdf += canonicalSdf;
					cumulativeLiveSdf += liveSdf;
					cumulativeWarpDist += ORUtils::length(canonicalVoxel.warp);
					consideredVoxelCount += 1;

					if (maxWarpUpdateLength < energyGradeintLength) {
						maxWarpUpdateLength = energyGradeintLength;
					}
					if (maxWarpLength < warpLength) {
						maxWarpLength = warpLength;
					}
					// endregion

					//TODO: move to separate function?
					if (printVoxelResult) {
						std::cout << "Data gradient: " << localDataEnergyGradient * -1.f;
						std::cout << red << " Level set gradient: " << localLevelSetEnergyGradient * -1.f;
						std::cout << yellow << " Killing gradient: " << localSmoothnessEnergyGradient * -1.f;
						std::cout << std::endl;
						std::cout << green << "Energy gradient: " << localEnergyGradient << reset;
						std::cout << " Energy gradient length: " << energyGradeintLength << std::endl << std::endl;

						if (recordVoxelResult) {
							const int& currentFrameIx = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::currentFrameIx;

							std::array<ITMNeighborVoxelIterationInfo, 9> neighbors;
							FindHighlightNeighborInfo(neighbors, voxelPosition, hash, canonicalVoxels,
							                          canonicalHashTable, liveVoxels, liveHashTable, liveCache);
							ITMHighlightIterationInfo info =
									{hash, locId, currentFrameIx, iteration, voxelPosition, warp, canonicalSdf, liveSdf,
									 localEnergyGradient, localDataEnergyGradient, localLevelSetEnergyGradient,
									 localSmoothnessEnergyGradient,
									 localEnergy, localDataEnergy, localLevelSetEnergy, localSmoothnessEnergy,
									 localKillingEnergy, localTikhonovEnergy,
									 liveSdfJacobian, liveSdfJacobian, liveSdfHessian, warpJacobian,
									 warpHessian[0], warpHessian[1], warpHessian[2], neighbors, true};
							ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::sceneLogger->
									LogHighlight(hash, locId, currentFrameIx, info);
						}
					}
				}
			}
		}
	}


	totalEnergy = totalDataEnergy + totalLevelSetEnergy + totalSmoothnessEnergy;

	std::cout << bright_cyan << "*** General Iteration Statistics ***" << reset << std::endl;
	PrintEnergyStatistics(enableDataTerm, enableLevelSetTerm, enableSmoothingTerm,
	                      useIsometryEnforcementFactorInSmoothingTerm, gamma, totalDataEnergy,
	                      totalLevelSetEnergy, totalTikhonovEnergy,
	                      totalKillingEnergy, totalSmoothnessEnergy, totalEnergy);

	//save all energies to file
	energy_stat_file << totalDataEnergy << ", " << totalLevelSetEnergy << ", " << totalKillingEnergy << ", "
	                 << totalSmoothnessEnergy << ", " << totalEnergy << std::endl;

	CalculateAndPrintAdditionalStatistics(enableDataTerm, enableLevelSetTerm, cumulativeCanonicalSdf, cumulativeLiveSdf,
	                                      cumulativeWarpDist, cumulativeSdfDiff, consideredVoxelCount, dataVoxelCount,
	                                      levelSetVoxelCount);
	return maxWarpUpdateLength;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
float ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::CalculateWarpUpdate_MultiThreaded(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

// endregion ===========================================================================================================

enum TraversalDirection : int {
	X = 0, Y = 1, Z = 2
};

template<typename TVoxelCanonical, typename TIndex, TraversalDirection TDirection>
struct GradientSmoothingPassFunctor {
	GradientSmoothingPassFunctor(ITMScene<TVoxelCanonical, TIndex>* scene) :
			scene(scene),
			voxels(scene->localVBA.GetVoxelBlocks()),
			hashEntries(scene->index.GetEntries()),
			cache() {}

	void operator()(TVoxelCanonical& voxel, Vector3i position) {
		const int directionIndex = (int) TDirection;
		const int sourceGradientIndex = directionIndex % 2;
		const int destinationGradientIndex = (directionIndex + 1) % 2;


		Vector3i receptiveVoxelPosition = position;
		receptiveVoxelPosition[directionIndex] -= (sobolevFilterSize / 2);
		Vector3f smoothedGradient;
		int vmIndex;
		for (int iVoxel = 0; iVoxel < sobolevFilterSize; receptiveVoxelPosition[directionIndex]++) {
			const TVoxelCanonical& receptiveVoxel = readVoxel(voxels, hashEntries, receptiveVoxelPosition, vmIndex);
			smoothedGradient += sobolevFilter1D[iVoxel] * GetGradient(receptiveVoxel);
		}
		SetGradient(voxel, smoothedGradient);
	}

private:
	Vector3f GetGradient(const TVoxelCanonical& voxel) const{
		switch (TDirection){
			case X:
				return voxel.gradient0;
			case Y:
				return voxel.gradient1;
			case Z:
				return voxel.gradient0;
		}
	}
	void SetGradient(TVoxelCanonical& voxel, const Vector3f gradient) const{
		switch (TDirection){
			case X:
				voxel.gradient1 = gradient;
				return;
			case Y:
				voxel.gradient0 = gradient;
				return;
			case Z:
				voxel.gradient1 = gradient;
				return;
		}
	}

	ITMScene<TVoxelCanonical, TIndex>* scene;
	TVoxelCanonical* voxels;
	ITMHashEntry* hashEntries;
	typename TIndex::IndexCache cache;

	static const int sobolevFilterSize;
	static const float sobolevFilter1D[];
};

template<typename TVoxelLive, typename TIndex, TraversalDirection TDirection>
const int GradientSmoothingPassFunctor<TVoxelLive, TIndex, TDirection>::sobolevFilterSize = 7;
template<typename TVoxelLive, typename TIndex, TraversalDirection TDirection>
const float GradientSmoothingPassFunctor<TVoxelLive, TIndex, TDirection>::sobolevFilter1D[] = {
		2.995861099047703036e-04f,
		4.410932423926419363e-03f,
		6.571314272194948847e-02f,
		9.956527876693953560e-01f,
		6.571314272194946071e-02f,
		4.410932423926422832e-03f,
		2.995861099045313996e-04f};



template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ApplySmoothingToGradient(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene) {
	GradientSmoothingPassFunctor<TVoxelCanonical, TIndex, X> passFunctorX(canonicalScene);
	GradientSmoothingPassFunctor<TVoxelCanonical, TIndex, Y> passFunctorY(canonicalScene);
	GradientSmoothingPassFunctor<TVoxelCanonical, TIndex, Z> passFunctorZ(canonicalScene);
	VoxelPositionTraversal_CPU(*canonicalScene, passFunctorX);
	VoxelPositionTraversal_CPU(*canonicalScene, passFunctorY);
	VoxelPositionTraversal_CPU(*canonicalScene, passFunctorZ);
}


// region ======================================== APPLY WARP UPDATE TO THE WARP ITSELF ================================


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
float ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ApplyWarpUpdateToWarp_SingleThreadedVerbose(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {

	const float learningRate = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::gradientDescentLearningRate;


	//Warp Update Length Histogram
	// <20%, 40%, 60%, 80%, 100%
	const int histBinCount = 10;
	int warpBins[histBinCount] = {0};
	int updateBins[histBinCount] = {0};


	// *** traversal vars
	// ** canonical frame
	TVoxelCanonical* canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();
	typename TIndex::IndexCache canonicalCache;
	// ** live frame
	TVoxelLive* liveVoxels = liveScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* liveHashTable = liveScene->index.GetEntries();
	int noTotalEntries = liveScene->index.noTotalEntries;
	typename TIndex::IndexCache liveCache;

	//Apply the update
	for (int hash = 0; hash < noTotalEntries; hash++) {
		const ITMHashEntry& currentLiveHashEntry = liveHashTable[hash];
		if (currentLiveHashEntry.ptr < 0) continue;
		ITMHashEntry& currentCanonicalHashEntry = canonicalHashTable[hash];

		// the rare case where we have different positions for live & canonical voxel block with the same index:
		// we have a hash bucket miss, find the canonical voxel with the matching coordinates
		if (currentCanonicalHashEntry.pos != currentLiveHashEntry.pos) {
			int canonicalHash = hash;
			if (!FindHashAtPosition(canonicalHash, currentLiveHashEntry.pos, canonicalHashTable)) {
				DIEWITHEXCEPTION_REPORTLOCATION("Could not find corresponding canonical block at postion!");
			}
			currentCanonicalHashEntry = canonicalHashTable[canonicalHash];
		}

		TVoxelLive* localLiveVoxelBlock = &(liveVoxels[currentLiveHashEntry.ptr * SDF_BLOCK_SIZE3]);
		TVoxelCanonical* localCanonicalVoxelBlock =
				&(canonicalVoxels[currentCanonicalHashEntry.ptr * SDF_BLOCK_SIZE3]);

		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					TVoxelLive& liveVoxel = localLiveVoxelBlock[locId];
					if (liveVoxel.flags != ITMLib::VOXEL_NONTRUNCATED) {
						continue;
					}
					TVoxelCanonical& canonicalVoxel = localCanonicalVoxelBlock[locId];
					canonicalVoxel.gradient0 = -learningRate * canonicalVoxel.gradient1;
					canonicalVoxel.warp += canonicalVoxel.gradient0;
					float warpLength = ORUtils::length(canonicalVoxel.warp);
					float warpUpdateLength = ORUtils::length(canonicalVoxel.gradient0);

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
				}
			}
		}
	}

	//start _DEBUG
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

	std::cout << "Max warp: " << maxWarpLength << " Max update: " << maxWarpUpdateLength << std::endl;
	//end _DEBUG
	return maxWarpUpdateLength;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
float ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ApplyWarpUpdateToWarp_MultiThreaded(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
float ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ApplyWarpUpdateToWarp(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {
#if defined(_DEBUG) && !defined(WITH_OPENMP)
	return ApplyWarpUpdateToWarp_SingleThreadedVerbose(canonicalScene, liveScene);
#else
	return ApplyWarpUpdateToWarp_MultiThreaded(canonicalScene,liveScene);
#endif
};




//endregion ============================================================================================================