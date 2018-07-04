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
#include "../../Engines/Manipulation/ITMSceneManipulation.h"
#include "../../Objects/Scene/ITMSceneTraversal.h"
#include "../../Utils/ITMVoxelFlags.h"
#include "../../Utils/Analytics/ITMSceneStatisticsCalculator.h"
#include "../Interface/ITMSceneMotionTracker.h"
#include "ITMSceneMotionTracker_CPU.h"
#include "../../Utils/FileIO/ITMDynamicFusionLogger.h"
#include "ITMWarpGradientCommon.h"


namespace ITMLib {

// region ========================== CALCULATE WARP GRADIENT ===========================================================



template<typename TVoxelCanonical, typename TVoxelLive>
struct ITMCalculateWarpGradientBasedOnWarpedLiveFunctor {
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
			if (ITMDynamicFusionLogger::Instance().IsRecordingScene2DSlicesWithUpdates()) {
				recordVoxelResult = true; //TODO: legacy, revise -Greg
			}
		}
	}

public:

	// region ========================================= CONSTRUCTOR ====================================================
	ITMCalculateWarpGradientBasedOnWarpedLiveFunctor(
			typename ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::Parameters parameters,
			typename ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::Switches switches) :
			liveCache(),
			canonicalCache(),
			parameters(parameters),
			switches(switches) {}

	void PrepareForOptimization(ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene,
	                            ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene, int sourceSdfIndex,
	                            bool hasFocusCoordinates, Vector3i focusCoordinates,
	                            bool restrictZtrackingForDebugging) {
		ResetStatistics();
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

	void operator()(TVoxelLive& liveVoxel, TVoxelCanonical& canonicalVoxel, Vector3i position) {

		if (!VoxelIsConsideredForTracking(canonicalVoxel, liveVoxel, sourceSdfIndex)) return;

		Vector3f& framewiseWarp = canonicalVoxel.framewise_warp;
		float liveSdf = TVoxelLive::valueToFloat(liveVoxel.sdf_values[sourceSdfIndex]);
		float canonicalSdf = TVoxelCanonical::valueToFloat(canonicalVoxel.sdf);
		bool haveFullData = true;
//		bool haveFullData = liveVoxel.flag_values[sourceSdfIndex] == ITMLib::VOXEL_NONTRUNCATED
//		                    && canonicalVoxel.flags == ITMLib::VOXEL_NONTRUNCATED;
//		bool haveFullData = liveVoxel.flag_values[sourceSdfIndex] == ITMLib::VOXEL_NONTRUNCATED
//		                    && canonicalVoxel.flags == ITMLib::VOXEL_NONTRUNCATED && std::abs(canonicalSdf - liveSdf) < 1.0;

		// region =============================== DECLARATIONS & DEFAULTS FOR ALL TERMS ====================



		float localDataEnergy = 0.0f, localLevelSetEnergy = 0.0f, localSmoothnessEnergy = 0.0f,
				localTikhonovEnergy = 0.0f, localKillingEnergy = 0.0f; // used for energy calculations in verbose output

		Vector3f localSmoothnessEnergyGradient(0.0f), localDataEnergyGradient(0.0f), localLevelSetEnergyGradient(0.0f);
		Matrix3f liveSdfHessian, framewiseWarpJacobian(0.0f);
		Vector3f liveSdfJacobian, framewiseWarpLaplacian;
		Matrix3f framewiseWarpHessian[3] = {Matrix3f(0.0f), Matrix3f(0.0f), Matrix3f(0.0f)};
		// endregion

		bool printVoxelResult = false, recordVoxelResult = false;
		this->SetUpFocusVoxelPrinting(printVoxelResult, recordVoxelResult, position, framewiseWarp, canonicalSdf,
		                              liveSdf);
		// region ============================== RETRIEVE NEIGHBOR'S WARPS =================================

		const int neighborhoodSize = 9;
		Vector3f neighborFramewiseWarps[neighborhoodSize], neighborWarpUpdates[neighborhoodSize];
		bool neighborKnown[neighborhoodSize], neighborTruncated[neighborhoodSize], neighborAllocated[neighborhoodSize];

		//    0        1        2          3         4         5           6         7         8
		//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)

		if (this->switches.usePreviousUpdateVectorsForSmoothing) {
			findPoint2ndDerivativeNeighborhoodPreviousUpdate(
					neighborWarpUpdates/*x9*/, neighborKnown, neighborTruncated,
					neighborAllocated, position, canonicalVoxels,
					canonicalHashEntries, canonicalCache);
			for (int iNeighbor = 0; iNeighbor < neighborhoodSize; iNeighbor++) {
				if (!neighborAllocated[iNeighbor]) {
					//assign current warp to neighbor warp if the neighbor is not allocated
					neighborWarpUpdates[iNeighbor] = canonicalVoxel.warp_update;
				}
			}
		} else {
			findPoint2ndDerivativeNeighborhoodFramewiseWarp(
					neighborFramewiseWarps/*x9*/, neighborKnown, neighborTruncated,
					neighborAllocated, position, canonicalVoxels,
					canonicalHashEntries, canonicalCache);
			//TODO: revise this to reflect new realities
			for (int iNeighbor = 0; iNeighbor < neighborhoodSize; iNeighbor++) {
				if (!neighborAllocated[iNeighbor]) {
					//assign current warp to neighbor warp if the neighbor is not allocated
					neighborFramewiseWarps[iNeighbor] = framewiseWarp;
				}
			}
		}


		if (printVoxelResult) {
			std::cout << blue << "Live 6-connected neighbor information:" << reset << std::endl;
			print6ConnectedNeighborInfoIndexedFields(position, liveVoxels, liveHashEntries, liveCache,
			                                         sourceSdfIndex);
		}

		//endregion

		if (haveFullData && (switches.enableLevelSetTerm || switches.enableDataTerm)) {
			ComputeLiveJacobian_CentralDifferences_IndexedFields(
					liveSdfJacobian, position, liveVoxels, liveHashEntries, liveCache, sourceSdfIndex);
			//_DEBUG
//			ComputeLiveJacobian_CentralDifferences_SuperHackyVersion_CanonicalSdf2(
//					liveSdfJacobian, position, liveVoxels, liveHashEntries, liveCache, sourceSdfIndex, canonicalSdf);
//			ComputeLiveJacobian_CentralDifferences_SuperHackyVersion_LiveSdf(
//					liveSdfJacobian, position, liveVoxels, liveHashEntries, liveCache, sourceSdfIndex);
//			ComputeLiveJacobian_CentralDifferences_IgnoreUnknown_IndexedFields(
//					liveSdfJacobian, position, liveVoxels,liveHashEntries, liveCache, sourceSdfIndex);
//			ComputeLiveJacobian_CentralDifferences_NontruncatedOnly_IndexedFields(
//					liveSdfJacobian, position, liveVoxels,liveHashEntries, liveCache, sourceSdfIndex);
//			ComputeLiveJacobian_CentralDifferences_SmallDifferences_IndexedFields(
//					liveSdfJacobian, position, liveVoxels,liveHashEntries, liveCache, sourceSdfIndex);
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
			ComputeSdfHessian_IndexedFields(liveSdfHessian, position, liveSdf, liveVoxels,
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
				ComputePerVoxelWarpJacobianAndHessian(framewiseWarp, neighborFramewiseWarps, framewiseWarpJacobian,
				                                      framewiseWarpHessian);
				if (printVoxelResult) {
					_DEBUG_PrintKillingTermStuff(neighborFramewiseWarps, neighborKnown, neighborTruncated,
					                             framewiseWarpJacobian, framewiseWarpHessian);
				}

				float gamma = parameters.rigidityEnforcementFactor;
				float onePlusGamma = 1.0f + gamma;
				// |0, 3, 6|     |m00, m10, m20|      |u_xx, u_xy, u_xz|
				// |1, 4, 7|     |m01, m11, m21|      |u_xy, u_yy, u_yz|
				// |2, 5, 8|     |m02, m12, m22|      |u_xz, u_yz, u_zz|
				Matrix3f& H_u = framewiseWarpHessian[0];
				Matrix3f& H_v = framewiseWarpHessian[1];
				Matrix3f& H_w = framewiseWarpHessian[2];


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
				Matrix3f warpJacobianTranspose = framewiseWarpJacobian.t();

				localTikhonovEnergy = dot(framewiseWarpJacobian.getColumn(0), framewiseWarpJacobian.getColumn(0)) +
				                      dot(framewiseWarpJacobian.getColumn(1), framewiseWarpJacobian.getColumn(1)) +
				                      dot(framewiseWarpJacobian.getColumn(2), framewiseWarpJacobian.getColumn(2));

				localKillingEnergy = gamma *
				                     (dot(warpJacobianTranspose.getColumn(0), framewiseWarpJacobian.getColumn(0)) +
				                      dot(warpJacobianTranspose.getColumn(1), framewiseWarpJacobian.getColumn(1)) +
				                      dot(warpJacobianTranspose.getColumn(2), framewiseWarpJacobian.getColumn(2)));

				localSmoothnessEnergy = localTikhonovEnergy + localKillingEnergy;
			} else {
				if(switches.usePreviousUpdateVectorsForSmoothing){
					ComputeWarpLaplacianAndJacobian(framewiseWarpLaplacian, framewiseWarpJacobian,
					                                canonicalVoxel.warp_update, neighborWarpUpdates);
				}else{
					ComputeWarpLaplacianAndJacobian(framewiseWarpLaplacian, framewiseWarpJacobian, framewiseWarp,
				                                    neighborFramewiseWarps);
				}
				if (printVoxelResult) {
					_DEBUG_PrintTikhonovTermStuff(neighborFramewiseWarps, framewiseWarpLaplacian);
				}
				//∇E_{reg}(Ψ) = −[∆U ∆V ∆W]' ,
				localSmoothnessEnergyGradient = -framewiseWarpLaplacian;
				localTikhonovEnergy = dot(framewiseWarpJacobian.getColumn(0), framewiseWarpJacobian.getColumn(0)) +
				                      dot(framewiseWarpJacobian.getColumn(1), framewiseWarpJacobian.getColumn(1)) +
				                      dot(framewiseWarpJacobian.getColumn(2), framewiseWarpJacobian.getColumn(2));
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
			          << " Gradients shown are negated." << reset << std::endl << std::endl;

			if (recordVoxelResult) {
				//TODO: legacy, revise/remove -Greg
				int x = 0, y = 0, z = 0, hash = 0, locId = 0;
				GetVoxelHashLocals(hash, x, y, y, z, liveHashEntries, liveCache, position);
				hash -= 1;
				std::array<ITMNeighborVoxelIterationInfo, 9> neighbors;
				FindHighlightNeighborInfo(neighbors, position, hash, canonicalVoxels,
				                          canonicalHashEntries, liveVoxels, liveHashEntries, liveCache);

				ITMHighlightIterationInfo info =
						{hash, locId, position, framewiseWarp, canonicalSdf, liveSdf,
						 localEnergyGradient, localDataEnergyGradient, localLevelSetEnergyGradient,
						 localSmoothnessEnergyGradient,
						 localEnergy, localDataEnergy, localLevelSetEnergy, localSmoothnessEnergy,
						 localKillingEnergy, localTikhonovEnergy,
						 liveSdfJacobian, liveSdfJacobian, liveSdfHessian, framewiseWarpJacobian,
						 framewiseWarpHessian[0], framewiseWarpHessian[1], framewiseWarpHessian[2], neighbors, true};
				ITMDynamicFusionLogger::Instance().LogHighlight(hash, locId, info);
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
		ITMDynamicFusionLogger::Instance().RecordAndPlotEnergies(
				totalDataEnergy, totalLevelSetEnergy, totalKillingEnergy, totalSmoothnessEnergy, totalEnergy);


		CalculateAndPrintAdditionalStatistics(
				this->switches.enableDataTerm, this->switches.enableLevelSetTerm, cumulativeCanonicalSdf,
				cumulativeLiveSdf,
				cumulativeWarpDist, cumulativeSdfDiff, consideredVoxelCount, dataVoxelCount, levelSetVoxelCount);
	}


private:
	void ResetStatistics() {
		cumulativeCanonicalSdf = 0.0;
		cumulativeLiveSdf = 0.0;
		cumulativeSdfDiff = 0.0;
		cumulativeWarpDist = 0.0;
		consideredVoxelCount = 0;
		dataVoxelCount = 0;
		levelSetVoxelCount = 0;

		totalDataEnergy = 0.0;
		totalLevelSetEnergy = 0.0;
		totalTikhonovEnergy = 0.0;
		totalKillingEnergy = 0.0;
		totalSmoothnessEnergy = 0.0;
	}

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

};
}// namespace ITMLib