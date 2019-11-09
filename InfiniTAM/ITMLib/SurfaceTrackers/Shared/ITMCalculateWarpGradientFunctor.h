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
#include "ITMSceneMotionTracker_Shared.h"
#include "ITMSceneMotionTracker_Debug.h"
#include "../../Utils/ITMVoxelFlags.h"
#include "../../../ORUtils/PlatformIndependentAtomics.h"
#include "ITMWarpGradientAggregates.h"
#include "../../Utils/ITMCPrintHelpers.h"
#include "ITMWarpGradientCommon.h"
#include "../../Engines/Manipulation/Shared/ITMSceneManipulationEngine_Shared.h"


namespace ITMLib {

// region ========================== CALCULATE WARP GRADIENT ===========================================================
template<typename TWarp, bool hasDebugInformation>
struct SetGradientFunctor;

template<typename TWarp>
struct SetGradientFunctor<TWarp, false> {
	_CPU_AND_GPU_CODE_
	inline static void SetGradient(TWarp& warp,
	                               const Vector3f& localDataEnergyGradient,
	                               const Vector3f& localLevelSetEnergyGradient,
	                               const Vector3f& localSmoothingEnergyGradient,
	                               const bool& restrictZtrackingForDebugging
	) {
		Vector3f localEnergyGradient =
				localDataEnergyGradient +
				localLevelSetEnergyGradient +
				localSmoothingEnergyGradient;
		if (restrictZtrackingForDebugging) localEnergyGradient.z = 0.0f;
		warp.gradient0 = localEnergyGradient;
	}
};

template<typename TWarp>
struct SetGradientFunctor<TWarp, true> {
	_CPU_AND_GPU_CODE_
	inline static void SetGradient(TWarp& warp,
	                               const Vector3f& localDataEnergyGradient,
	                               const Vector3f& localLevelSetEnergyGradient,
	                               const Vector3f& localSmoothingEnergyGradient,
	                               const bool& restrictZtrackingForDebugging
	) {
		warp.data_term_gradient = localDataEnergyGradient;
		warp.smoothing_term_gradient = localSmoothingEnergyGradient;
		Vector3f localEnergyGradient =
				localDataEnergyGradient +
				localLevelSetEnergyGradient +
				localSmoothingEnergyGradient;
		if (restrictZtrackingForDebugging) localEnergyGradient.z = 0.0f;
		warp.gradient0 = localEnergyGradient;
	}
};


template<typename TVoxel, typename TWarp, typename TIndexData, typename TCache>
struct ITMCalculateWarpGradientFunctor {
private:

	_CPU_AND_GPU_CODE_
	void SetUpFocusVoxelPrinting(bool& printVoxelResult, const Vector3i& voxelPosition,
	                             const Vector3f& voxelWarp, const float& canonicalSdf, const float& liveSdf) {
		if (hasFocusCoordinates && voxelPosition == focusCoordinates) {
			int x = 0, y = 0, z = 0, vmIndex = 0, locId = 0;
			GetVoxelHashLocals(vmIndex, locId, x, y, z, liveIndexData, liveCache, voxelPosition);

			printf("\n%s *** Printing voxel at (%d, %d, %d) ***%s\n", c_bright_cyan,
			       voxelPosition.x, voxelPosition.y, voxelPosition.z, c_reset);
			printf("Position within block (x,y,z): (%d, %d, %d)\n", x, y, z);
			printf("Canonical TSDF of %f vs. live TSDF of %f.\n", canonicalSdf, liveSdf);
			printf("Warp: %s%f, %f, %f%s\n", c_green, voxelWarp.x, voxelWarp.y, voxelWarp.z, c_reset);
			printf("Warp length: %s%f%s\n", c_green, ORUtils::length(voxelWarp), c_reset);

			printVoxelResult = true;
		}
	}


public:

	// region ========================================= CONSTRUCTOR ====================================================

	ITMCalculateWarpGradientFunctor(SlavchevaSurfaceTracker::Parameters parameters,
	                                SlavchevaSurfaceTracker::Switches switches,
	                                TVoxel* liveVoxels, const TIndexData* liveIndexData,
	                                TVoxel* canonicalVoxels, const TIndexData* canonicalIndexData,
	                                TWarp* warps, const TIndexData* warpIndexData) :
			parameters(parameters), switches(switches),
			liveVoxels(liveVoxels), liveIndexData(liveIndexData),
			warps(warps), warpIndexData(warpIndexData),
			canonicalVoxels(canonicalVoxels), canonicalIndexData(canonicalIndexData),
			liveCache(), canonicalCache(),
			hasFocusCoordinates(Configuration::Instance().analysisSettings.focus_coordinates_specified),
			focusCoordinates(Configuration::Instance().analysisSettings.focus_coordinates) {}

	// endregion =======================================================================================================

	_DEVICE_WHEN_AVAILABLE_
	void operator()(TVoxel& voxelLive, TVoxel& voxelCanonical, TWarp& warp, Vector3i voxelPosition) {

		if (!VoxelIsConsideredForTracking(voxelCanonical, voxelLive)) return;
		bool computeDataAndLevelSetTerms = VoxelIsConsideredForDataTerm(voxelCanonical, voxelLive);

		Vector3f& framewiseWarp = warp.flow_warp;
		float liveSdf = TVoxel::valueToFloat(voxelLive.sdf);
		float canonicalSdf = TVoxel::valueToFloat(voxelCanonical.sdf);

		// region =============================== DECLARATIONS & DEFAULTS FOR ALL TERMS ====================

		Vector3f localSmoothingEnergyGradient(0.0f), localDataEnergyGradient(0.0f), localLevelSetEnergyGradient(0.0f);

		// endregion


		bool printVoxelResult = false;
		this->SetUpFocusVoxelPrinting(printVoxelResult, voxelPosition, framewiseWarp, canonicalSdf, liveSdf);
		if (printVoxelResult) {
			printf("%sLive 6-connected neighbor information:%s\n", c_blue, c_reset);
			print6ConnectedNeighborInfo(voxelPosition, liveVoxels, liveIndexData, liveCache);
		}

		//_DEBUG


		// region =============================== DATA TERM ================================================
		if (computeDataAndLevelSetTerms) {
			Vector3f liveSdfJacobian;
			ComputeLiveJacobian_CentralDifferences(
					liveSdfJacobian, voxelPosition, liveVoxels, liveIndexData, liveCache);
			if (switches.enableDataTerm) {

				// Compute data term error / energy
				float sdfDifferenceBetweenLiveAndCanonical = liveSdf - canonicalSdf;
				// (φ_n(Ψ)−φ_{global}) ∇φ_n(Ψ) - also denoted as - (φ_{proj}(Ψ)−φ_{model}) ∇φ_{proj}(Ψ)
				// φ_n(Ψ) = φ_n(x+u, y+v, z+w), where u = u(x,y,z), v = v(x,y,z), w = w(x,y,z)
				// φ_{global} = φ_{global}(x, y, z)
				localDataEnergyGradient =
						parameters.weightDataTerm * sdfDifferenceBetweenLiveAndCanonical * liveSdfJacobian;

				ATOMIC_ADD(aggregates.dataVoxelCount, 1u);
				float localDataEnergy = parameters.weightDataTerm * 0.5f *
				                        (sdfDifferenceBetweenLiveAndCanonical * sdfDifferenceBetweenLiveAndCanonical);


				ATOMIC_ADD(energies.totalDataEnergy, localDataEnergy);

				ATOMIC_ADD(aggregates.dataVoxelCount, 1u);

				//printf(*energies.totalDataEnergy);
				if (printVoxelResult) {
					_DEBUG_PrintDataTermStuff(liveSdfJacobian);
				}
			}

			// endregion

			// region =============================== LEVEL SET TERM ===========================================

			if (switches.enableLevelSetTerm) {
				Matrix3f liveSdfHessian;
				ComputeSdfHessian(liveSdfHessian, voxelPosition, liveSdf, liveVoxels, liveIndexData, liveCache);

				float sdfJacobianNorm = ORUtils::length(liveSdfJacobian);
				float sdfJacobianNormMinusUnity = sdfJacobianNorm - parameters.unity;
				localLevelSetEnergyGradient = parameters.weightLevelSetTerm * sdfJacobianNormMinusUnity *
				                              (liveSdfHessian * liveSdfJacobian) /
				                              (sdfJacobianNorm + parameters.epsilon);
				ATOMIC_ADD(aggregates.levelSetVoxelCount, 1u);
				float localLevelSetEnergy = parameters.weightLevelSetTerm *
						0.5f * (sdfJacobianNormMinusUnity * sdfJacobianNormMinusUnity);
				ATOMIC_ADD(energies.totalLevelSetEnergy, localLevelSetEnergy);
				if (printVoxelResult) {
					_DEBUG_PrintLevelSetTermStuff(liveSdfJacobian, liveSdfHessian, sdfJacobianNormMinusUnity);
				}
			}
			// endregion =======================================================================================
		}

		// region =============================== SMOOTHING TERM (TIKHONOV & KILLING) ======================

		if (switches.enableSmoothingTerm) {
			// region ============================== RETRIEVE NEIGHBOR'S WARPS =========================================

			const int neighborhoodSize = 9;
			Vector3f neighborFlowWarps[neighborhoodSize];
			bool neighborKnown[neighborhoodSize], neighborTruncated[neighborhoodSize], neighborAllocated[neighborhoodSize];

			//    0        1        2          3         4         5           6         7         8
			//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
			findPoint2ndDerivativeNeighborhoodFlowWarp(
					neighborFlowWarps/*x9*/, neighborKnown, neighborTruncated, neighborAllocated, voxelPosition,
					warps, warpIndexData, warpCache, canonicalVoxels, canonicalIndexData, canonicalCache);

			for (int iNeighbor = 0; iNeighbor < neighborhoodSize; iNeighbor++) {
				if (!neighborAllocated[iNeighbor]) {
					//assign current warp to neighbor warp if the neighbor is not allocated
					neighborFlowWarps[iNeighbor] = framewiseWarp;
				}
			}
			//endregion=================================================================================================

			if (switches.enableKillingRigidityEnforcementTerm) {
				Matrix3f framewiseWarpJacobian(0.0f);
				Matrix3f framewiseWarpHessian[3] = {Matrix3f(0.0f), Matrix3f(0.0f), Matrix3f(0.0f)};
				ComputePerVoxelWarpJacobianAndHessian(framewiseWarp, neighborFlowWarps, framewiseWarpJacobian,
				                                      framewiseWarpHessian);
//TODO rewrite function to be used within CUDA code, remove guards
#ifndef __CUDACC__
				if (printVoxelResult) {
					_DEBUG_PrintKillingTermStuff(neighborFlowWarps, neighborKnown, neighborTruncated,
					                             framewiseWarpJacobian, framewiseWarpHessian);
				}
#endif

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

				localSmoothingEnergyGradient =
						parameters.weightSmoothingTerm * Vector3f(KillingDeltaEu, KillingDeltaEv, KillingDeltaEw);
				//=================================== ENERGY ===============================================
				// KillingTerm Energy
				Matrix3f warpJacobianTranspose = framewiseWarpJacobian.t();

				float localTikhonovEnergy = parameters.weightSmoothingTerm *
				                            dot(framewiseWarpJacobian.getColumn(0),
				                                framewiseWarpJacobian.getColumn(0)) +
				                            dot(framewiseWarpJacobian.getColumn(1),
				                                framewiseWarpJacobian.getColumn(1)) +
				                            dot(framewiseWarpJacobian.getColumn(2), framewiseWarpJacobian.getColumn(2));

				float localRigidityEnergy = gamma * parameters.weightSmoothingTerm *
				                            (dot(warpJacobianTranspose.getColumn(0),
				                                 framewiseWarpJacobian.getColumn(0)) +
				                             dot(warpJacobianTranspose.getColumn(1),
				                                 framewiseWarpJacobian.getColumn(1)) +
				                             dot(warpJacobianTranspose.getColumn(2),
				                                 framewiseWarpJacobian.getColumn(2)));
				ATOMIC_ADD(energies.totalTikhonovEnergy, localTikhonovEnergy);
				ATOMIC_ADD(energies.totalRigidityEnergy, localRigidityEnergy);
			} else {
				Matrix3f framewiseWarpJacobian(0.0f);
				Vector3f framewiseWarpLaplacian;
				ComputeWarpLaplacianAndJacobian(framewiseWarpLaplacian, framewiseWarpJacobian, framewiseWarp,
				                                neighborFlowWarps);


				if (printVoxelResult) {
					_DEBUG_PrintTikhonovTermStuff(neighborFlowWarps, framewiseWarpLaplacian);
				}

				//∇E_{reg}(Ψ) = −[∆U ∆V ∆W]' ,
				localSmoothingEnergyGradient = -parameters.weightSmoothingTerm * framewiseWarpLaplacian;
				float localTikhonovEnergy = parameters.weightSmoothingTerm *
						dot(framewiseWarpJacobian.getColumn(0), framewiseWarpJacobian.getColumn(0)) +
						dot(framewiseWarpJacobian.getColumn(1), framewiseWarpJacobian.getColumn(1)) +
						dot(framewiseWarpJacobian.getColumn(2), framewiseWarpJacobian.getColumn(2));
				ATOMIC_ADD(energies.totalTikhonovEnergy, localTikhonovEnergy);
			}
		}
		// endregion
		// region =============================== COMPUTE ENERGY GRADIENT ==================================
		SetGradientFunctor<TWarp, TWarp::hasDebugInformation>::SetGradient(
				warp, localDataEnergyGradient, localLevelSetEnergyGradient, localSmoothingEnergyGradient,
				restrictZtrackingForDebugging);
		// endregion
		// region =============================== AGGREGATE VOXEL STATISTICS ===============================


		float warpLength = ORUtils::length(warp.flow_warp);

		ATOMIC_ADD(aggregates.cumulativeCanonicalSdf, canonicalSdf);
		ATOMIC_ADD(aggregates.cumulativeLiveSdf, liveSdf);
		ATOMIC_ADD(aggregates.cumulativeWarpDist, warpLength);
		ATOMIC_ADD(aggregates.consideredVoxelCount, 1u);
		// endregion

		// region ======================== FINALIZE RESULT PRINTING / RECORDING ========================================

		if (printVoxelResult) {
			float energyGradientLength = ORUtils::length(warp.gradient0);
			_DEBUG_printLocalEnergyGradients(localDataEnergyGradient, localLevelSetEnergyGradient,
			                                 localSmoothingEnergyGradient, warp.gradient0, energyGradientLength);
		}
		// endregion ===================================================================================================

	}


	void PrintStatistics() {
		std::cout << bright_cyan << "*** Non-rigid Alignment Iteration Statistics ***" << reset << std::endl;
		PrintEnergyStatistics(this->switches.enableDataTerm, this->switches.enableLevelSetTerm,
		                      this->switches.enableSmoothingTerm, this->switches.enableKillingRigidityEnforcementTerm,
		                      this->parameters.rigidityEnforcementFactor, energies);
		CalculateAndPrintAdditionalStatistics(
				this->switches.enableDataTerm, this->switches.enableLevelSetTerm, aggregates);
	}


private:

	// *** data structure accessors
	const TVoxel* liveVoxels;
	const TIndexData* liveIndexData;
	TCache liveCache;

	const TVoxel* canonicalVoxels;
	const TIndexData* canonicalIndexData;
	TCache canonicalCache;

	TWarp* warps;
	const TIndexData* warpIndexData;
	TCache warpCache;

	AdditionalGradientAggregates aggregates;
	ComponentEnergies energies;

	// *** debugging / analysis variables
	bool hasFocusCoordinates{};
	Vector3i focusCoordinates;
	bool restrictZtrackingForDebugging = false;

	const SlavchevaSurfaceTracker::Parameters parameters;
	const SlavchevaSurfaceTracker::Switches switches;
};

}// namespace ITMLib
