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
#include "../../Utils/ITMSceneStatisticsCalculator.h"


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

template<typename TVoxelCanonical>
struct ClearOutGradientStaticFunctor {
	static void run(TVoxelCanonical& voxel) {
		voxel.gradient0 = Vector3f(0.0f);
		voxel.gradient1 = Vector3f(0.0f);
	}
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::CalculateWarpGradient(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
		ITMScene<TVoxelLive, TIndex>* liveScene) {

	StaticVoxelTraversal_CPU<ClearOutGradientStaticFunctor<TVoxelCanonical>>(*canonicalScene);

#if defined(_DEBUG) && !defined(WITH_OPENMP)
	CalculateWarpGradient_SingleThreadedVerbose(canonicalScene, liveScene);
#else
	CalculateWarpGradient_MultiThreaded(canonicalScene, liveScene);
#endif
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
struct CalculateWarpGradient_SingleThreadedVerboseFunctor {

	CalculateWarpGradient_SingleThreadedVerboseFunctor(
			ITMScene<TVoxelLive, TIndex>* liveScene,
			ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
			typename ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::Parameters parameters,
			typename ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::Switches switches,
			unsigned int iteration, unsigned int currentFrameIx,
			bool hasFocusCoordinates, Vector3i focusCoordinates) :

			hasFocusCoordinates(hasFocusCoordinates),
			focusCoordinates(focusCoordinates),
			iteration(iteration),
			sourceSdfIndex((iteration + 1) % 2),
			currentFrameIx(currentFrameIx),
			parameters(parameters),
			switches(switches),

			liveScene(liveScene),
			liveVoxels(liveScene->localVBA.GetVoxelBlocks()),
			liveHashEntries(liveScene->index.GetEntries()),
			liveCache(),

			canonicalScene(canonicalScene),
			canonicalVoxels(canonicalScene->localVBA.GetVoxelBlocks()),
			canonicalHashEntries(canonicalScene->index.GetEntries()),
			canonicalCache() {}

	void operator()(TVoxelLive& liveVoxel, TVoxelCanonical& canonicalVoxel, Vector3i voxelPosition) {
		if (liveVoxel.flag_values[sourceSdfIndex] != ITMLib::VOXEL_NONTRUNCATED) {
			return;
		}

		// region =============================== DECLARATIONS & DEFAULTS FOR ALL TERMS ====================
		float canonicalSdf = TVoxelCanonical::valueToFloat(canonicalVoxel.sdf);
		float liveSdf = TVoxelLive::valueToFloat(liveVoxel.sdf_values[sourceSdfIndex]);
		Vector3f& warp = canonicalVoxel.warp;
		Vector3f localSmoothnessEnergyGradient(0.0f), localDataEnergyGradient(0.0f), localLevelSetEnergyGradient(0.0f);
		float localDataEnergy = 0.0f, localLevelSetEnergy = 0.0f, localSmoothnessEnergy = 0.0f,
				localTikhonovEnergy = 0.0f, localKillingEnergy = 0.0f; // used for energy calculations in verbose output
		float sdfDifferenceBetweenLiveAndCanonical = 0.0f, sdfJacobianNormMinusUnity = 0.0f;
		Matrix3f liveSdfHessian, warpJacobian(0.0f);
		Vector3f liveSdfJacobian, warpLaplacian;
		Matrix3f warpHessian[3] = {Matrix3f(0.0f), Matrix3f(0.0f), Matrix3f(0.0f)};
		// endregion

		bool printVoxelResult = false, recordVoxelResult = false;
		SetUpFocusVoxelPrinting(printVoxelResult, recordVoxelResult, voxelPosition, warp, canonicalSdf, liveSdf);
		// region ============================== RETRIEVE NEIGHBOR'S WARPS =================================

		const int neighborhoodSize = 9; Vector3f neighborWarps[neighborhoodSize];
		bool neighborKnown[neighborhoodSize], neighborTruncated[neighborhoodSize], neighborAllocated[neighborhoodSize];
		//    0        1        2          3         4         5           6         7         8
		//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
		findPoint2ndDerivativeNeighborhoodWarp(neighborWarps/*x9*/, neighborKnown, neighborTruncated,
		                                       neighborAllocated, voxelPosition, canonicalVoxels,
		                                       canonicalHashEntries, canonicalCache);
		//TODO: revise this to reflect new realities
		for (int iNeighbor = 0; iNeighbor < neighborhoodSize; iNeighbor++) {
			if (!neighborAllocated[iNeighbor]) {
				//assign current warp to neighbor warp if the neighbor is not allocated
				neighborWarps[iNeighbor] = warp;
			}
		}
		if (printVoxelResult) {
			std::cout << blue << "Live 6-connected neighbor information:" << reset << std::endl;
			print6ConnectedNeighborInfo(voxelPosition, liveVoxels, liveHashEntries, liveCache);
		}

		//endregion

		if (switches.enableLevelSetTerm || switches.enableDataTerm) {
			//TODO: in case both level set term and data term need to be computed, optimize by retreiving the sdf vals for live jacobian in a separate function. The live hessian needs to reuse them. -Greg (GitHub: Algomorph)
			// compute the gradient of the live frame, ∇φ_n(Ψ) a.k.a ∇φ_{proj}(Ψ)
			ComputeLiveJacobian_CentralDifferences(liveSdfJacobian, voxelPosition, liveVoxels, liveHashEntries,
			                                       liveCache);
		}

		// region =============================== DATA TERM ================================================
		if (switches.enableDataTerm) {
			// Compute data term error / energy
			sdfDifferenceBetweenLiveAndCanonical = liveSdf - canonicalSdf;
			// (φ_n(Ψ)−φ_{global}) ∇φ_n(Ψ) - also denoted as - (φ_{proj}(Ψ)−φ_{model}) ∇φ_{proj}(Ψ)
			// φ_n(Ψ) = φ_n(x+u, y+v, z+w), where u = u(x,y,z), v = v(x,y,z), w = w(x,y,z)
			// φ_{global} = φ_{global}(x, y, z)
			localDataEnergyGradient = sdfDifferenceBetweenLiveAndCanonical * liveSdfJacobian;

			dataVoxelCount++;
			cumulativeSdfDiff += std::abs(sdfDifferenceBetweenLiveAndCanonical);
			localDataEnergy =
					0.5f * (sdfDifferenceBetweenLiveAndCanonical * sdfDifferenceBetweenLiveAndCanonical);
		}
		if (printVoxelResult) {
			if (switches.enableDataTerm) {
				_DEBUG_PrintDataTermStuff(liveSdfJacobian);
			} else {
				std::cout << std::endl;
			}
		}
		// endregion

		// region =============================== LEVEL SET TERM ===========================================

		if (switches.enableLevelSetTerm) {
			ComputeSdfHessian(liveSdfHessian, voxelPosition, liveSdf, liveVoxels, liveHashEntries, liveCache);
			float sdfJacobianNorm = ORUtils::length(liveSdfJacobian);
			sdfJacobianNormMinusUnity = sdfJacobianNorm - parameters.unity;
			localLevelSetEnergyGradient = sdfJacobianNormMinusUnity * (liveSdfHessian * liveSdfJacobian) /
			                              (sdfJacobianNorm + parameters.epsilon);
			levelSetVoxelCount++;
			localLevelSetEnergy =
					parameters.weightLevelSetTerm * 0.5f * (sdfJacobianNormMinusUnity * sdfJacobianNormMinusUnity);
		}
		// endregion =======================================================================================

		// region =============================== SMOOTHING TERM (TIKHONOV & KILLING) ======================

		if (switches.enableSmoothingTerm) {
			if (switches.enableKillingTerm) {
				ComputePerVoxelWarpJacobianAndHessian(canonicalVoxel.warp, neighborWarps,
				                                      warpJacobian, warpHessian);
				if (printVoxelResult) {
					_DEBUG_PrintKillingTermStuff(neighborWarps, neighborKnown, neighborTruncated,
					                             warpJacobian, warpHessian);
				}

				float gamma = parameters.rigidityEnforcementFactor;
				float onePlusGamma = 1.0f + gamma;
				// |0, 3, 6|     |m00, m10, m20|      |u_xx, u_xy, u_xz|
				// |1, 4, 7|     |m01, m11, m21|      |u_xy, u_yy, u_yz|
				// |2, 5, 8|     |m02, m12, m22|      |u_xz, u_yz, u_zz|
				Matrix3f& H_u = warpHessian[0];
				Matrix3f& H_v = warpHessian[1];
				Matrix3f& H_w = warpHessian[2];


				float KillingDeltaEu = -2.0f *
				                 ((onePlusGamma) * H_u.xx + (H_u.yy) + (H_u.zz) + gamma * H_v.xy + gamma * H_w.xz);
				float KillingDeltaEv = -2.0f *
				                 ((onePlusGamma) * H_v.yy + (H_v.zz) + (H_v.xx) + gamma * H_u.xy + gamma * H_w.yz);
				float KillingDeltaEw = -2.0f *
				                 ((onePlusGamma) * H_w.zz + (H_w.xx) + (H_w.yy) + gamma * H_v.yz + gamma * H_u.xz);

				localSmoothnessEnergyGradient = Vector3f(KillingDeltaEu, KillingDeltaEv, KillingDeltaEw);
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
				localSmoothnessEnergy = localTikhonovEnergy;
			}
		}
		// endregion

		// region =============================== COMPUTE ENERGY GRADIENT ==================================

		Vector3f localEnergyGradient =
				localDataEnergyGradient +
				parameters.weightLevelSetTerm * localLevelSetEnergyGradient +
				parameters.weightSmoothnessTerm * localSmoothnessEnergyGradient;

		//_DEBUG
		//localEnergyGradient.z = 0.0f;

		canonicalVoxel.gradient0 = localEnergyGradient;

		// endregion

		// region =============================== AGGREGATE VOXEL STATISTICS ===============================
		float energyGradeintLength = ORUtils::length(localEnergyGradient);//meters
		float warpLength = ORUtils::length(canonicalVoxel.warp);
		double localEnergy = localDataEnergy + parameters.weightLevelSetTerm * localLevelSetEnergy
		                     + parameters.weightSmoothnessTerm * localSmoothnessEnergy;


		totalDataEnergy += localDataEnergy;
		totalLevelSetEnergy += parameters.weightLevelSetTerm * localLevelSetEnergy;
		totalTikhonovEnergy += parameters.weightSmoothnessTerm * localTikhonovEnergy;
		totalKillingEnergy += parameters.weightSmoothnessTerm * localKillingEnergy;
		totalSmoothnessEnergy += parameters.weightSmoothnessTerm * localSmoothnessEnergy;

		cumulativeCanonicalSdf += canonicalSdf;
		cumulativeLiveSdf += liveSdf;
		cumulativeWarpDist += ORUtils::length(canonicalVoxel.warp);
		consideredVoxelCount += 1;
		// endregion

		//TODO: move to separate function?
		if (printVoxelResult) {
			std::cout << blue << "Data gradient: " << localDataEnergyGradient * -1.f;
			std::cout << red << " Level set gradient: " << localLevelSetEnergyGradient * -1.f;
			std::cout << yellow << " Smoothness gradient: " << localSmoothnessEnergyGradient * -1.f;
			std::cout << std::endl;
			std::cout << green << "Energy gradient: " << localEnergyGradient << reset;
			std::cout << " Energy gradient length: " << energyGradeintLength << std::endl << std::endl;

			if (recordVoxelResult) {
				//TODO: function to retrieve voxel hash location for debugging -Greg (GitHub: Algomorph)
				int x = 0, y = 0, z = 0, hash = 0, locId = 0;
				std::array<ITMNeighborVoxelIterationInfo, 9> neighbors;
				FindHighlightNeighborInfo(neighbors, voxelPosition, hash, canonicalVoxels,
				                          canonicalHashEntries, liveVoxels, liveHashEntries, liveCache);
				ITMHighlightIterationInfo info =
						{hash, locId, currentFrameIx, iteration, voxelPosition, warp, canonicalSdf, liveSdf,
						 localEnergyGradient, localDataEnergyGradient, localLevelSetEnergyGradient,
						 localSmoothnessEnergyGradient,
						 localEnergy, localDataEnergy, localLevelSetEnergy, localSmoothnessEnergy,
						 localKillingEnergy, localTikhonovEnergy,
						 liveSdfJacobian, liveSdfJacobian, liveSdfHessian, warpJacobian,
						 warpHessian[0], warpHessian[1], warpHessian[2], neighbors, true};
				sceneLogger->LogHighlight(hash, locId, currentFrameIx, info);
			}
		}
	}

	void FinalizePrintAndRecordStatistics(std::ofstream& energy_stat_file) {
		double totalEnergy = totalDataEnergy + totalLevelSetEnergy + totalSmoothnessEnergy;

		std::cout << bright_cyan << "*** General Iteration Statistics ***" << reset << std::endl;
		PrintEnergyStatistics(this->switches.enableDataTerm, this->switches.enableLevelSetTerm,
		                      this->switches.enableSmoothingTerm, this->switches.enableKillingTerm,
		                      this->parameters.rigidityEnforcementFactor,
		                      totalDataEnergy, totalLevelSetEnergy, totalTikhonovEnergy,
		                      totalKillingEnergy, totalSmoothnessEnergy, totalEnergy);

		//save all energies to file
		energy_stat_file << totalDataEnergy << ", " << totalLevelSetEnergy << ", " << totalKillingEnergy << ", "
		                 << totalSmoothnessEnergy << ", " << totalEnergy << std::endl;

		CalculateAndPrintAdditionalStatistics(
				this->switches.enableDataTerm, this->switches.enableLevelSetTerm, cumulativeCanonicalSdf,
				cumulativeLiveSdf,
				cumulativeWarpDist, cumulativeSdfDiff, consideredVoxelCount, dataVoxelCount, levelSetVoxelCount);
	}

private:

	// *** data structure accessors
	ITMScene<TVoxelLive, TIndex>* liveScene;
	TVoxelLive* liveVoxels;
	ITMHashEntry* liveHashEntries;
	typename TIndex::IndexCache liveCache;

	ITMScene<TVoxelCanonical, TIndex>* canonicalScene;
	TVoxelCanonical* canonicalVoxels;
	ITMHashEntry* canonicalHashEntries;
	typename TIndex::IndexCache canonicalCache;

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
	const bool hasFocusCoordinates;
	const Vector3i focusCoordinates;
	// status variables
	const int iteration;
	const int sourceSdfIndex;
	const int currentFrameIx;

	const typename ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::Parameters parameters;
	const typename ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::Switches switches;

	ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>* sceneLogger;

	void SetUpFocusVoxelPrinting(bool& printVoxelResult, bool& recordVoxelResult, const Vector3i& voxelPosition,
	                             const Vector3f& voxelWarp, const float& canonicalSdf, const float& liveSdf) {
		//TODO: function to retrieve voxel hash location for debugging -Greg (GitHub: Algomorph)
		int x = 0, y = 0, z = 0, hash = 0, locId = 0;
		if (hasFocusCoordinates && voxelPosition == focusCoordinates) {
			std::cout << std::endl << bright_cyan << "*** Printing voxel at " << voxelPosition
			          << " *** " << reset << std::endl;
			std::cout << "Position within block (x,y,z): " << x << ", " << y << ", " << z << std::endl;
			std::cout << "Source SDF vs. target SDF: " << canonicalSdf << "-->" << liveSdf << std::endl
			          << "Warp: " << green << voxelWarp << reset
			          << " Warp length: " << green << ORUtils::length(voxelWarp) << reset;
			std::cout << std::endl;
			printVoxelResult = true;
			if (sceneLogger != nullptr) {
				recordVoxelResult = true;
			}
		}
	}
	void GetWarpNeighborhood(){

	}
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void
ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::CalculateWarpGradient_SingleThreadedVerbose(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {

	CalculateWarpGradient_SingleThreadedVerboseFunctor<TVoxelCanonical, TVoxelLive, TIndex> calculateGradientFunctor(
			liveScene, canonicalScene, this->parameters, this->switches, this->iteration, this->currentFrameIx,
			this->hasFocusCoordinates, this->focusCoordinates);

	DualVoxelPositionTraversal_AllocateSecondaryOnMiss_CPU(
			liveScene, canonicalScene, this->canonicalEntryAllocationTypes, calculateGradientFunctor, true);

	calculateGradientFunctor.FinalizePrintAndRecordStatistics(this->energy_stat_file);
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::CalculateWarpGradient_MultiThreaded(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {

	// region DECLARATIONS / INITIALIZATIONS

	// params
	const float epsilon = this->parameters.epsilon;
	const int currentFrameIx = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::currentFrameIx;
	const int iteration = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::iteration;
	const int sourceSdfIndex = (iteration + 1) % 2;

	const float weightSmoothnessTerm = this->parameters.weightSmoothnessTerm;
	const float weightLevelSetTerm = this->parameters.weightLevelSetTerm;
	const float gamma = this->parameters.rigidityEnforcementFactor;
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
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int hash = 0; hash < noTotalEntries; hash++) {
		ITMHashEntry& currentLiveHashEntry = liveHashTable[hash];
		if (currentLiveHashEntry.ptr < 0) continue;
		ITMHashEntry& currentCanonicalHashEntry = canonicalHashTable[hash];

		// the rare case where we have different positions for live & canonical voxel block with the same index:
		// we have a hash bucket miss, find the canonical voxel with the matching coordinates
		if (currentCanonicalHashEntry.pos != currentLiveHashEntry.pos) {
			int canonicalHash;
			if (!FindHashAtPosition(canonicalHash, currentLiveHashEntry.pos, canonicalHashTable)) {
				int lastFreeVoxelBlockId = canonicalScene->localVBA.lastFreeBlockId;
				int lastFreeExcessListId = canonicalScene->index.GetLastFreeExcessListId();
				int* voxelAllocationList = canonicalScene->localVBA.GetAllocationList();
				int* excessAllocationList = canonicalScene->index.GetExcessAllocationList();
				ITMHashEntry* newEntry;
				if (AllocateHashEntry_CPU(currentLiveHashEntry.pos, canonicalHashTable, newEntry, lastFreeExcessListId,
				                          lastFreeExcessListId, voxelAllocationList, excessAllocationList,
				                          canonicalHash)) {
					currentCanonicalHashEntry = *newEntry;
					uchar* entriesAllocType = this->canonicalEntryAllocationTypes->GetData(MEMORYDEVICE_CPU);
					entriesAllocType[canonicalHash] = ITMLib::BOUNDARY;
					std::cout << bright_red << "Allocating hash entry in canonical during gradient update at: "
					          << currentLiveHashEntry.pos << reset << std::endl;
				} else {
					DIEWITHEXCEPTION_REPORTLOCATION("Could not allocate hash entry...");
				}
				canonicalScene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
				canonicalScene->index.SetLastFreeExcessListId(lastFreeExcessListId);
			} else {
				currentCanonicalHashEntry = canonicalHashTable[canonicalHash];
			}
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
					if (liveVoxel.flag_values[sourceSdfIndex] != ITMLib::VOXEL_NONTRUNCATED || currentFrameIx == 0) {
						continue;
					}
					// region =============================== DECLARATIONS & DEFAULTS FOR ALL TERMS ====================

					Vector3i voxelPosition = hashBlockPosition + Vector3i(x, y, z);
					TVoxelCanonical& canonicalVoxel = localCanonicalVoxelBlock[locId];
					float canonicalSdf = TVoxelCanonical::valueToFloat(canonicalVoxel.sdf);
					float liveSdf = TVoxelLive::valueToFloat(liveVoxel.sdf_values[sourceSdfIndex]);
					Vector3f& warp = canonicalVoxel.warp;

					Vector3f localSmoothnessEnergyGradient(0.0f);
					Vector3f localDataEnergyGradient(0.0f);
					Vector3f localLevelSetEnergyGradient(0.0f);
					float sdfDifferenceBetweenLiveAndCanonical = 0.0f, sdfJacobianNormMinusUnity = 0.0f;
					Matrix3f liveSdfHessian;
					Vector3f liveSdfJacobian, warpLaplacian;
					Matrix3f warpJacobian(0.0f);
					Matrix3f warpHessian[3] = {Matrix3f(0.0f), Matrix3f(0.0f), Matrix3f(0.0f)};
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

					if (this->switches.enableLevelSetTerm || this->switches.enableDataTerm) {
						// compute the gradient of the live frame, ∇φ_n(Ψ)
						ComputeLiveJacobian_CentralDifferences(liveSdfJacobian, voxelPosition, liveVoxels,
						                                       liveHashTable, liveCache);
					}

					// region =============================== DATA TERM ================================================

					if (this->switches.enableDataTerm) {
						// Compute data term error / energy
						sdfDifferenceBetweenLiveAndCanonical = liveSdf - canonicalSdf;
						// (φ_n(Ψ)−φ_{global}) ∇φ_n(Ψ)
						// φ_n(Ψ) = φ_n(x+u, y+v, z+w), where u = u(x,y,z), v = v(x,y,z), w = w(x,y,z)
						// φ_{global} = φ_{global}(x, y, z)
						localDataEnergyGradient = sdfDifferenceBetweenLiveAndCanonical * liveSdfJacobian;
					}

					// endregion

					// region =============================== LEVEL SET TERM ===========================================

					if (this->switches.enableLevelSetTerm) {

						ComputeSdfHessian(liveSdfHessian, voxelPosition, liveSdf, liveVoxels, liveHashTable, liveCache);

						float sdfJacobianNorm = ORUtils::length(liveSdfJacobian);
						sdfJacobianNormMinusUnity = sdfJacobianNorm - unity;

						localLevelSetEnergyGradient =
								sdfJacobianNormMinusUnity * (liveSdfHessian * liveSdfJacobian) /
								(sdfJacobianNorm + epsilon);

					}
					// endregion =======================================================================================

					// region =============================== SMOOTHING TERM (TIKHONOV & KILLING) ======================
					if (this->switches.enableSmoothingTerm) {
						if (this->switches.enableKillingTerm) {
							ComputePerVoxelWarpJacobianAndHessian(canonicalVoxel.warp, neighborWarps,
							                                      warpJacobian, warpHessian);


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
						} else {
							ComputeWarpLaplacianAndJacobian(warpLaplacian, warpJacobian, warp, neighborWarps);
							//∇E_{reg}(Ψ) = −[∆U ∆V ∆W]' ,
							localSmoothnessEnergyGradient = -warpLaplacian;
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
					// endregion
				}
			}
		}
	}
}

// endregion ===========================================================================================================

enum TraversalDirection : int {
	X = 0, Y = 1, Z = 2
};


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex, TraversalDirection TDirection>
struct GradientSmoothingPassFunctor {
	GradientSmoothingPassFunctor(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                             ITMScene<TVoxelLive, TIndex>* liveScene) :
			canonicalScene(canonicalScene),
			canonicalVoxels(canonicalScene->localVBA.GetVoxelBlocks()),
			canoincalHashEntries(canonicalScene->index.GetEntries()),
			canonicalCache(),
			liveScene(liveScene),
			liveVoxels(liveScene->localVBA.GetVoxelBlocks()),
			liveHashEntries(liveScene->index.GetEntries()),
			liveCache() {}

	void operator()(TVoxelCanonical& voxel, Vector3i position) {
		int vmIndex;
		const TVoxelLive& liveVoxel = readVoxel(liveVoxels, liveHashEntries, position, vmIndex, liveCache);
		if (liveVoxel.flags != ITMLib::VOXEL_NONTRUNCATED) return;

		const int directionIndex = (int) TDirection;

		Vector3i receptiveVoxelPosition = position;
		receptiveVoxelPosition[directionIndex] -= (sobolevFilterSize / 2);
		Vector3f smoothedGradient(0.0f);

		for (int iVoxel = 0; iVoxel < sobolevFilterSize; iVoxel++, receptiveVoxelPosition[directionIndex]++) {
			const TVoxelCanonical& receptiveVoxel = readVoxel(canonicalVoxels, canoincalHashEntries,
			                                                  receptiveVoxelPosition, vmIndex, canonicalCache);
			smoothedGradient += sobolevFilter1D[iVoxel] * GetGradient(receptiveVoxel);
		}
		SetGradient(voxel, smoothedGradient);
	}

private:
	Vector3f GetGradient(const TVoxelCanonical& voxel) const {
		switch (TDirection) {
			case X:
				return voxel.gradient0;
			case Y:
				return voxel.gradient1;
			case Z:
				return voxel.gradient0;
		}
	}

	void SetGradient(TVoxelCanonical& voxel, const Vector3f gradient) const {
		switch (TDirection) {
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

	ITMScene<TVoxelCanonical, TIndex>* canonicalScene;
	TVoxelCanonical* canonicalVoxels;
	ITMHashEntry* canoincalHashEntries;
	typename TIndex::IndexCache canonicalCache;

	ITMScene<TVoxelLive, TIndex>* liveScene;
	TVoxelLive* liveVoxels;
	ITMHashEntry* liveHashEntries;
	typename TIndex::IndexCache liveCache;

	static const int sobolevFilterSize;
	static const float sobolevFilter1D[];
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex, TraversalDirection TDirection>
const int GradientSmoothingPassFunctor<TVoxelCanonical, TVoxelLive, TIndex, TDirection>::sobolevFilterSize = 7;
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex, TraversalDirection TDirection>
const float GradientSmoothingPassFunctor<TVoxelCanonical, TVoxelLive, TIndex, TDirection>::sobolevFilter1D[] = {
		2.995861099047703036e-04f,
		4.410932423926419363e-03f,
		6.571314272194948847e-02f,
		9.956527876693953560e-01f,
		6.571314272194946071e-02f,
		4.410932423926422832e-03f,
		2.995861099045313996e-04f};

//_DEBUG -- normalized version
//template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex, TraversalDirection TDirection>
//const float GradientSmoothingPassFunctor<TVoxelCanonical, TVoxelLive, TIndex, TDirection>::sobolevFilter1D[] = {
//		2.636041325812907461e-04f,
//		3.881154276361719040e-03f,
//		5.782062280706985746e-02f,
//		8.760692375679742794e-01f,
//		5.782062280706985746e-02f,
//		3.881154276361719040e-03f,
//		2.636041325812907461e-04f};



template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ApplySmoothingToGradient(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {
	if (this->switches.enableGradientSmoothing) {
		GradientSmoothingPassFunctor<TVoxelCanonical, TVoxelLive, TIndex, X> passFunctorX(canonicalScene, liveScene);
		GradientSmoothingPassFunctor<TVoxelCanonical, TVoxelLive, TIndex, Y> passFunctorY(canonicalScene, liveScene);
		GradientSmoothingPassFunctor<TVoxelCanonical, TVoxelLive, TIndex, Z> passFunctorZ(canonicalScene, liveScene);

		VoxelPositionTraversal_CPU(*canonicalScene, passFunctorX);
		VoxelPositionTraversal_CPU(*canonicalScene, passFunctorY);
		VoxelPositionTraversal_CPU(*canonicalScene, passFunctorZ);
	}
}

// region ======================================== APPLY WARP UPDATE TO THE WARP ITSELF ================================
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
float ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ApplyWarpUpdateToWarp_SingleThreadedVerbose(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {

	const float learningRate = this->parameters.gradientDescentLearningRate;
	const int currentFrameIx = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::currentFrameIx;
	const int iteration = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::iteration;
	const int sourceSdfIndex = (iteration + 1) % 2;

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

	// *** stats
	float maxWarpLength = 0.0f;
	float maxWarpUpdateLength = 0.0f;
	Vector3i maxWarpPosition(0);
	Vector3i maxWarpUpdatePosition(0);

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
				std::stringstream stream;
				stream << "Could not find corresponding canonical block at postion " << currentLiveHashEntry.pos
				       << " at frame " << currentFrameIx << ". " << __FILE__ << ": " << __LINE__;
				DIEWITHEXCEPTION(stream.str());
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
					if (liveVoxel.flag_values[sourceSdfIndex] != ITMLib::VOXEL_NONTRUNCATED) {
						continue;
					}
					TVoxelCanonical& canonicalVoxel = localCanonicalVoxelBlock[locId];
					Vector3f warpUpdate = -learningRate * (this->switches.enableGradientSmoothing ?
					                                       canonicalVoxel.gradient1 : canonicalVoxel.gradient0);

					canonicalVoxel.gradient0 = warpUpdate;
					canonicalVoxel.warp += warpUpdate;
					canonicalVoxel.frame_warp += warpUpdate;
					float warpLength = ORUtils::length(canonicalVoxel.warp);
					float warpUpdateLength = ORUtils::length(warpUpdate);
					if (warpLength > maxWarpLength) {
						maxWarpLength = warpLength;
						maxWarpPosition = currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE + Vector3i(x, y, z);
					}
					if (warpUpdateLength > maxWarpUpdateLength) {
						maxWarpUpdateLength = warpUpdateLength;
						maxWarpUpdatePosition =
								currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE + Vector3i(x, y, z);
					}
				}
			}
		}
	}

	//Warp Update Length Histogram
	// <20%, 40%, 60%, 80%, 100%
	const int histBinCount = 10;
	int warpBins[histBinCount] = {0};
	int updateBins[histBinCount] = {0};

	for (int hash = 0; hash < noTotalEntries; hash++) {
		const ITMHashEntry& currentLiveHashEntry = liveHashTable[hash];
		if (currentLiveHashEntry.ptr < 0) continue;
		if (currentLiveHashEntry.ptr < 0) continue;
		ITMHashEntry& currentCanonicalHashEntry = canonicalHashTable[hash];

		// the rare case where we have different positions for live & canonical voxel block with the same index:
		// we have a hash bucket miss, find the canonical voxel with the matching coordinates
		if (currentCanonicalHashEntry.pos != currentLiveHashEntry.pos) {
			int canonicalHash = hash;
			if (!FindHashAtPosition(canonicalHash, currentLiveHashEntry.pos, canonicalHashTable)) {
				std::stringstream stream;
				stream << "Could not find corresponding canonical block at postion " << currentLiveHashEntry.pos
				       << " at frame " << currentFrameIx << ". " << __FILE__ << ": " << __LINE__;
				DIEWITHEXCEPTION(stream.str());
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

					float warpLength = ORUtils::length(canonicalVoxel.warp);
					float warpUpdateLength = ORUtils::length(canonicalVoxel.gradient0);
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

	std::cout << green << "Max warp: [" << maxWarpLength << " at " << maxWarpPosition << "] Max update: ["
	          << maxWarpUpdateLength << " at " << maxWarpUpdatePosition << "]." << reset << std::endl;

	return maxWarpUpdateLength;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
float ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ApplyWarpUpdateToWarp_MultiThreaded(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {
	const float learningRate = this->parameters.gradientDescentLearningRate;
	const int currentFrameIx = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::currentFrameIx;
	const int iteration = ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::iteration;
	const int sourceSdfIndex = (iteration + 1) % 2;
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

	// *** stats
	float maxWarpLength = 0.0f;
	float maxWarpUpdateLength = 0.0f;
	Vector3i maxWarpPosition(0.f);
	Vector3i maxWarpUpdatePosition(0.f);
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
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
				std::stringstream stream;
				stream << "Could not find corresponding canonical block at postion " << currentLiveHashEntry.pos
				       << " at frame " << currentFrameIx << ". " << __FILE__ << ": " << __LINE__;
				DIEWITHEXCEPTION(stream.str());
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
					if (liveVoxel.flag_values[sourceSdfIndex] != ITMLib::VOXEL_NONTRUNCATED) {
						continue;
					}
					TVoxelCanonical& canonicalVoxel = localCanonicalVoxelBlock[locId];
					Vector3f warpUpdate = -learningRate * (this->switches.enableGradientSmoothing ?
					                                       canonicalVoxel.gradient1 : canonicalVoxel.gradient0);
					canonicalVoxel.gradient0 = warpUpdate;
					canonicalVoxel.warp += warpUpdate;
					float warpLength = ORUtils::length(canonicalVoxel.warp);
					float warpUpdateLength = ORUtils::length(warpUpdate);
					if (warpLength > maxWarpLength) {
						maxWarpLength = warpLength;
						maxWarpPosition = currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE + Vector3i(x, y, z);
					}
					if (warpUpdateLength > maxWarpUpdateLength) {
						maxWarpUpdateLength = warpUpdateLength;
						maxWarpUpdatePosition =
								currentCanonicalHashEntry.pos.toInt() * SDF_BLOCK_SIZE + Vector3i(x, y, z);
					}
				}
			}
		}
	}
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
float ITMSceneMotionTracker_CPU<TVoxelCanonical, TVoxelLive, TIndex>::ApplyWarpUpdateToWarp(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene) {
#if defined(_DEBUG) && !defined(WITH_OPENMP)
	return ApplyWarpUpdateToWarp_SingleThreadedVerbose(canonicalScene, liveScene);
#else
	return ApplyWarpUpdateToWarp_MultiThreaded(canonicalScene, liveScene);
#endif
};




//endregion ============================================================================================================