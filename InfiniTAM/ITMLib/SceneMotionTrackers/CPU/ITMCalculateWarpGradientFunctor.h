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
#include "../Shared/ITMSceneMotionTracker_Shared.h"
#include "../Shared/ITMSceneMotionTracker_Debug.h"
#include "../../Objects/Scene/ITMSceneManipulation.h"
#include "../../Objects/Scene/ITMSceneTraversal.h"
#include "../../Utils/ITMVoxelFlags.h"
#include "../../Utils/Analytics/ITMSceneStatisticsCalculator.h"
#include "../Interface/ITMSceneMotionTracker.h"
#include "ITMSceneMotionTracker_CPU.h"
#include "../../Utils/FileIO/ITMDynamicFusionLogger.h"


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

// region ========================== CALCULATE WARP GRADIENT ===========================================================



template<typename TVoxelCanonical, typename TVoxelLive>
struct ITMCalculateWarpGradientFunctor {
private:

	void SetUpFocusVoxelPrinting(bool& printVoxelResult, bool& recordVoxelResult, const Vector3i& voxelPosition,
	                             const Vector3f& voxelWarp, const float& canonicalSdf, const float& liveSdf) {
		if (hasFocusCoordinates && voxelPosition == focusCoordinates) {
			int x = 0, y = 0, z = 0, vmIndex = 0, locId = 0;
			GetVoxelHashLocals(vmIndex, locId, x, y, z, liveHashEntries, liveCache, voxelPosition);
			std::cout << std::endl << bright_cyan << "*** Printing voxel at " << voxelPosition
			          << " *** " << reset << std::endl;
			std::cout << "Position within block (x,y,z): " << x << ", " << y << ", " << z << std::endl;
			std::cout << "Canonical SDF vs. live SDF: " << canonicalSdf << "-->" << liveSdf << std::endl
			          << "Warp: " << green << voxelWarp << reset
			          << " Warp length: " << green << ORUtils::length(voxelWarp) << reset;

			std::cout << std::endl;
			printVoxelResult = true;
			if (logger.IsRecordingWarps()) {
				recordVoxelResult = true;
			}
		}
	}

public:
	// region ========================================= CONSTRUCTOR ====================================================
	ITMCalculateWarpGradientFunctor(
			typename ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::Parameters parameters,
			typename ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::Switches switches,
			ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>& logger) :
			liveCache(),
			canonicalCache(),
			parameters(parameters),
			switches(switches),
			logger(logger) {}

	void PrepareForOptimization(ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene,
	                            ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene, int sourceSdfIndex,
	                            bool hasFocusCoordinates, Vector3i focusCoordinates,
	                            bool restrictZtrackingForDebugging) {
		this->liveScene = liveScene;
		this->liveVoxels = liveScene->localVBA.GetVoxelBlocks(),
		this->liveHashEntries = liveScene->index.GetEntries(),
		this->canonicalScene = canonicalScene;
		this->canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
		this->canonicalHashEntries = canonicalScene->index.GetEntries();
		this->hasFocusCoordinates = hasFocusCoordinates;
		this->focusCoordinates = focusCoordinates;
		this->restrictZtrackingForDebugging = restrictZtrackingForDebugging;
		this->sourceSdfIndex = sourceSdfIndex;
	}


	// endregion =======================================================================================================

	void operator()(TVoxelLive& liveVoxel, TVoxelCanonical& canonicalVoxel, Vector3i voxelPosition) {
		Vector3f& warp = canonicalVoxel.warp;
		bool haveFullData = liveVoxel.flag_values[sourceSdfIndex] == ITMLib::VOXEL_NONTRUNCATED
		                    && canonicalVoxel.flags == ITMLib::VOXEL_NONTRUNCATED;

		// region =============================== DECLARATIONS & DEFAULTS FOR ALL TERMS ====================
		float liveSdf = TVoxelLive::valueToFloat(liveVoxel.sdf_values[sourceSdfIndex]);
		float canonicalSdf = TVoxelCanonical::valueToFloat(canonicalVoxel.sdf);


		float localDataEnergy = 0.0f, localLevelSetEnergy = 0.0f, localSmoothnessEnergy = 0.0f,
				localTikhonovEnergy = 0.0f, localKillingEnergy = 0.0f; // used for energy calculations in verbose output

		Vector3f localSmoothnessEnergyGradient(0.0f), localDataEnergyGradient(0.0f), localLevelSetEnergyGradient(0.0f);
		Matrix3f liveSdfHessian, warpJacobian(0.0f);
		Vector3f liveSdfJacobian, warpLaplacian;
		Matrix3f warpHessian[3] = {Matrix3f(0.0f), Matrix3f(0.0f), Matrix3f(0.0f)};
		// endregion

		bool printVoxelResult = false, recordVoxelResult = false;
		this->SetUpFocusVoxelPrinting(printVoxelResult, recordVoxelResult, voxelPosition, warp, canonicalSdf, liveSdf);
		// region ============================== RETRIEVE NEIGHBOR'S WARPS =================================

		const int neighborhoodSize = 9;
		Vector3f neighborWarps[neighborhoodSize];
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
			print6ConnectedNeighborInfoIndexedFields(voxelPosition, liveVoxels, liveHashEntries, liveCache,
			                                         sourceSdfIndex);
		}

		//endregion

		if (haveFullData && (switches.enableLevelSetTerm || switches.enableDataTerm)) {
			//TODO: in case both level set term and data term need to be computed, optimize by retrieving the sdf values for live jacobian in a separate function. The live hessian needs to reuse them. -Greg (GitHub: Algomorph)
			ComputeLiveJacobian_CentralDifferences_IndexedFields(
					liveSdfJacobian, voxelPosition, liveVoxels, liveHashEntries, liveCache, sourceSdfIndex);
//			ComputeLiveJacobian_CentralDifferences_SuperHackyVersion_CanonicalSdf(
//					liveSdfJacobian, voxelPosition, liveVoxels, liveHashEntries, liveCache, sourceSdfIndex, canonicalSdf);
//			ComputeLiveJacobian_CentralDifferences_SuperHackyVersion_LiveSdf(
//					liveSdfJacobian, voxelPosition, liveVoxels, liveHashEntries, liveCache, sourceSdfIndex);
//			ComputeLiveJacobian_CentralDifferences_IgnoreUnknown_IndexedFields(
//					liveSdfJacobian, voxelPosition, liveVoxels,liveHashEntries, liveCache, sourceSdfIndex);
//			ComputeLiveJacobian_CentralDifferences_NontruncatedOnly_IndexedFields(
//					liveSdfJacobian, voxelPosition, liveVoxels,liveHashEntries, liveCache, sourceSdfIndex);
		}

		// region =============================== DATA TERM ================================================
		if (switches.enableDataTerm && haveFullData) {
			// Compute data term error / energy
			float sdfDifferenceBetweenLiveAndCanonical = liveSdf - canonicalSdf;
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

		if (switches.enableLevelSetTerm && haveFullData) {
			ComputeSdfHessian_IndexedFields(liveSdfHessian, voxelPosition, liveSdf, liveVoxels,
			                                liveHashEntries, liveCache, sourceSdfIndex);
			float sdfJacobianNorm = ORUtils::length(liveSdfJacobian);
			float sdfJacobianNormMinusUnity = sdfJacobianNorm - parameters.unity;
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
				                       ((onePlusGamma) * H_u.xx + (H_u.yy) + (H_u.zz) + gamma * H_v.xy +
				                        gamma * H_w.xz);
				float KillingDeltaEv = -2.0f *
				                       ((onePlusGamma) * H_v.yy + (H_v.zz) + (H_v.xx) + gamma * H_u.xy +
				                        gamma * H_w.yz);
				float KillingDeltaEw = -2.0f *
				                       ((onePlusGamma) * H_w.zz + (H_w.xx) + (H_w.yy) + gamma * H_v.yz +
				                        gamma * H_u.xz);

				localSmoothnessEnergyGradient = ORUtils::Vector3<float>(KillingDeltaEu, KillingDeltaEv, KillingDeltaEw);
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
				if (printVoxelResult) {
					_DEBUG_PrintTikhonovTermStuff(neighborWarps, warpLaplacian);
				}
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
				parameters.weightDataTerm * localDataEnergyGradient +
				parameters.weightLevelSetTerm * localLevelSetEnergyGradient +
				parameters.weightSmoothnessTerm * localSmoothnessEnergyGradient;

		if (restrictZtrackingForDebugging) localEnergyGradient.z = 0.0f;

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

		// region ======================== FINALIZE RESULT PRINTING / RECORDING ========================================
		//TODO: move to separate function?
		if (printVoxelResult) {
			std::cout << blue << "Data gradient: " << localDataEnergyGradient * -1;
			std::cout << cyan << " Level set gradient: " << localLevelSetEnergyGradient * -1;
			std::cout << yellow << " Smoothness gradient: " << localSmoothnessEnergyGradient * -1;
			std::cout << std::endl;
			std::cout << green << "Energy gradient: " << localEnergyGradient * -1 << reset;
			std::cout << " Energy gradient length: " << energyGradeintLength << red
			          << " Gradients shown are negative." << reset << std::endl << std::endl;

			if (recordVoxelResult) {
				int x = 0, y = 0, z = 0, hash = 0, locId = 0;
				GetVoxelHashLocals(hash, x, y, y, z, liveHashEntries, liveCache, voxelPosition);
				hash -= 1;
				std::array<ITMNeighborVoxelIterationInfo, 9> neighbors;
				FindHighlightNeighborInfo(neighbors, voxelPosition, hash, canonicalVoxels,
				                          canonicalHashEntries, liveVoxels, liveHashEntries, liveCache);
				//TODO: get rid of iteration + frame fields in HighlightIterationInfo
				ITMHighlightIterationInfo info =
						{hash, locId, voxelPosition, warp, canonicalSdf, liveSdf,
						 localEnergyGradient, localDataEnergyGradient, localLevelSetEnergyGradient,
						 localSmoothnessEnergyGradient,
						 localEnergy, localDataEnergy, localLevelSetEnergy, localSmoothnessEnergy,
						 localKillingEnergy, localTikhonovEnergy,
						 liveSdfJacobian, liveSdfJacobian, liveSdfHessian, warpJacobian,
						 warpHessian[0], warpHessian[1], warpHessian[2], neighbors, true};
				logger.LogHighlight(hash, locId, info);
			}
		}
		// endregion ===================================================================================================
	}

	void FinalizePrintAndRecordStatistics() {
		double totalEnergy = totalDataEnergy + totalLevelSetEnergy + totalSmoothnessEnergy;

		std::cout << bright_cyan << "*** General Iteration Statistics ***" << reset << std::endl;
		PrintEnergyStatistics(this->switches.enableDataTerm, this->switches.enableLevelSetTerm,
		                      this->switches.enableSmoothingTerm, this->switches.enableKillingTerm,
		                      this->parameters.rigidityEnforcementFactor,
		                      totalDataEnergy, totalLevelSetEnergy, totalTikhonovEnergy,
		                      totalKillingEnergy, totalSmoothnessEnergy, totalEnergy);

		//save all energies to file
		logger.RecordStatistics(totalDataEnergy, totalLevelSetEnergy, totalKillingEnergy, totalSmoothnessEnergy,
		                        totalEnergy);


		CalculateAndPrintAdditionalStatistics(
				this->switches.enableDataTerm, this->switches.enableLevelSetTerm, cumulativeCanonicalSdf,
				cumulativeLiveSdf,
				cumulativeWarpDist, cumulativeSdfDiff, consideredVoxelCount, dataVoxelCount, levelSetVoxelCount);
	}

	//_DEBUG


private:

	// *** data structure accessors
	ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene;
	int sourceSdfIndex{};
	TVoxelLive* liveVoxels;
	ITMHashEntry* liveHashEntries{};
	ITMVoxelBlockHash::IndexCache liveCache;

	ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene;
	TVoxelCanonical* canonicalVoxels;
	ITMHashEntry* canonicalHashEntries{};
	ITMVoxelBlockHash::IndexCache canonicalCache;

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

	// *** debuging / analysis variables
	bool hasFocusCoordinates{};
	Vector3i focusCoordinates;
	bool restrictZtrackingForDebugging = false;

	const typename ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::Parameters parameters;
	const typename ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::Switches switches;

	ITMDynamicFusionLogger<TVoxelCanonical,TVoxelLive, ITMVoxelBlockHash>& logger;

};