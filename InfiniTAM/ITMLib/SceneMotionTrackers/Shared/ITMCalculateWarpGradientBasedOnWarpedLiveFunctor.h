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
#include "../../Utils/Analytics/ITMSceneStatisticsCalculator.h"
#include "../Interface/ITMSceneMotionTracker.h"

#ifndef __CUDACC__

#include "../CPU/ITMWarpGradientCommon.h"
#include "../../Utils/FileIO/ITMDynamicFusionLogger.h"

#endif


namespace ITMLib {

// region ========================== CALCULATE WARP GRADIENT ===========================================================
template<typename TVoxelCanonical, bool hasDebugInformation>
struct SetGradientFunctor;

template<typename TVoxelCanonical>
struct SetGradientFunctor<TVoxelCanonical, false> {
	_CPU_AND_GPU_CODE_
	inline static void SetGradient(TVoxelCanonical& voxel,
	                               const float& weightDataTerm,
	                               const float& weightLevelSetTerm,
	                               const float& weightSmoothingTerm,
	                               const Vector3f& localDataEnergyGradient,
	                               const Vector3f& localLevelSetEnergyGradient,
	                               const Vector3f& localSmoothingEnergyGradient,
	                               const bool& restrictZtrackingForDebugging
	) {
		Vector3f localEnergyGradient =
				weightDataTerm * localDataEnergyGradient +
				weightLevelSetTerm * localLevelSetEnergyGradient +
				weightSmoothingTerm * localSmoothingEnergyGradient;
		if (restrictZtrackingForDebugging) localEnergyGradient.z = 0.0f;
		voxel.gradient0 = localEnergyGradient;
	}
};

template<typename TVoxelCanonical>
struct SetGradientFunctor<TVoxelCanonical, true> {
	_CPU_AND_GPU_CODE_
	inline static void SetGradient(TVoxelCanonical& voxel,
	                               const float& weightDataTerm,
	                               const float& weightLevelSetTerm,
	                               const float& weightSmoothingTerm,
	                               const Vector3f& localDataEnergyGradient,
	                               const Vector3f& localLevelSetEnergyGradient,
	                               const Vector3f& localSmoothingEnergyGradient,
	                               const bool& restrictZtrackingForDebugging
	) {
		voxel.data_term_gradient = weightDataTerm * localDataEnergyGradient;
		voxel.smoothing_term_gradient = weightLevelSetTerm * localSmoothingEnergyGradient;
		Vector3f localEnergyGradient =
				voxel.data_term_gradient +
				weightLevelSetTerm * localLevelSetEnergyGradient +
				voxel.smoothing_term_gradient;
		if (restrictZtrackingForDebugging) localEnergyGradient.z = 0.0f;
		voxel.gradient0 = localEnergyGradient;
	}
};


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
struct ITMCalculateWarpGradientBasedOnWarpedLiveFunctor {
private:

#ifndef __CUDACC__

	void SetUpFocusVoxelPrinting(bool& printVoxelResult, const Vector3i& voxelPosition,
	                             const Vector3f& voxelWarp, const float& canonicalSdf, const float& liveSdf) {
		if (hasFocusCoordinates && voxelPosition == focusCoordinates) {
			int x = 0, y = 0, z = 0, vmIndex = 0, locId = 0;
			GetVoxelHashLocals(vmIndex, locId, x, y, z, liveIndexData, liveCache, voxelPosition);
			std::cout << std::endl << bright_cyan << "*** Printing voxel at " << voxelPosition
			          << " *** " << reset << std::endl;
			std::cout << "Position within block (x,y,z): " << x << ", " << y << ", " << z << std::endl;
			std::cout << "Canonical SDF vs. live SDF: " << canonicalSdf << "-->" << liveSdf << std::endl
			          << "Warp: " << green << voxelWarp << reset
			          << " Warp length: " << green << ORUtils::length(voxelWarp) << reset;

			std::cout << std::endl;
			printVoxelResult = true;
		}
	}

#endif

public:

	// region ========================================= CONSTRUCTOR ====================================================
	ITMCalculateWarpGradientBasedOnWarpedLiveFunctor(
			typename ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::Parameters parameters,
			typename ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::Switches switches) :
			liveCache(),
			canonicalCache(),
			parameters(parameters),
			switches(switches),
			hasFocusCoordinates(ITMLibSettings::Instance().FocusCoordinatesAreSpecified()),
			focusCoordinates(ITMLibSettings::Instance().GetFocusCoordinates()) {}

	void
	PrepareForOptimization(ITMScene<TVoxelLive, TIndex>* liveScene, ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                       int sourceSdfIndex, bool restrictZtrackingForDebugging) {
		ResetStatistics();
		this->liveScene = liveScene;
		this->liveVoxels = liveScene->localVBA.GetVoxelBlocks(),
		this->liveIndexData = liveScene->index.getIndexData(),
		this->canonicalScene = canonicalScene;
		this->canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
		this->canonicalIndexData = canonicalScene->index.getIndexData();
		this->restrictZtrackingForDebugging = restrictZtrackingForDebugging;
		this->sourceSdfIndex = sourceSdfIndex;

	}

	// endregion =======================================================================================================
	_CPU_AND_GPU_CODE_
	void operator()(TVoxelLive& liveVoxel, TVoxelCanonical& canonicalVoxel, Vector3i position) {

		//_DEBUG
		if(position == Vector3i(-18, 48, 213)){
			int i = 42;
		}

		if (!VoxelIsConsideredForTracking(canonicalVoxel, liveVoxel, sourceSdfIndex)) return;
		bool computeDataTerm = VoxelIsConsideredForDataTerm(canonicalVoxel, liveVoxel, sourceSdfIndex);

		Vector3f& framewiseWarp = canonicalVoxel.flow_warp;
		float liveSdf = TVoxelLive::valueToFloat(liveVoxel.sdf_values[sourceSdfIndex]);
		float canonicalSdf = TVoxelCanonical::valueToFloat(canonicalVoxel.sdf);

		// region =============================== DECLARATIONS & DEFAULTS FOR ALL TERMS ====================

		float localDataEnergy = 0.0f, localLevelSetEnergy = 0.0f, localSmoothingEnergy = 0.0f,
				localTikhonovEnergy = 0.0f, localKillingEnergy = 0.0f; // used for energy calculations in verbose output

		Vector3f localSmoothnessEnergyGradient(0.0f), localDataEnergyGradient(0.0f), localLevelSetEnergyGradient(0.0f);

		// endregion

#ifndef __CUDACC__
		bool printVoxelResult = false;
		this->SetUpFocusVoxelPrinting(printVoxelResult, position, framewiseWarp, canonicalSdf, liveSdf);
		if (printVoxelResult) {
			std::cout << blue << "Live 6-connected neighbor information:" << reset << std::endl;
			print6ConnectedNeighborInfoIndexedFields(position, liveVoxels, liveIndexData, liveCache,
			                                         sourceSdfIndex);
		}
#endif

		// region =============================== DATA TERM ================================================
		if (switches.enableDataTerm && computeDataTerm) {
			computeDataTermUpdateContribution(localDataEnergyGradient, localDataEnergy, liveCache,
					position, liveVoxels, liveIndexData, sourceSdfIndex, liveSdf, canonicalSdf, parameters.sdfToVoxelsFactorSquared);
#ifndef __CUDACC__
			dataVoxelCount++;
#endif
		}
		// endregion

		// region =============================== LEVEL SET TERM ===========================================
		if (switches.enableLevelSetTerm && computeDataTerm) {
			Matrix3f liveSdfHessian;
			Vector3f liveSdfJacobian;
			ComputeLiveGradient_CentralDifferences_IndexedFields_AdvancedGrad(
					liveSdfJacobian, position, liveVoxels, liveIndexData, liveCache, sourceSdfIndex, liveSdf);

			ComputeSdfHessian_IndexedFields(liveSdfHessian, position, liveSdf, liveVoxels,
			                                liveIndexData, liveCache, sourceSdfIndex);
			float sdfJacobianNorm = ORUtils::length(liveSdfJacobian);
			float sdfJacobianNormMinusUnity = sdfJacobianNorm - parameters.unity;
			localLevelSetEnergyGradient = sdfJacobianNormMinusUnity * (liveSdfHessian * liveSdfJacobian) /
			                              (sdfJacobianNorm + parameters.epsilon);
#ifndef __CUDACC__
			levelSetVoxelCount++;
#endif
			localLevelSetEnergy =
					parameters.weightLevelSetTerm * 0.5f * (sdfJacobianNormMinusUnity * sdfJacobianNormMinusUnity);
		}
		// endregion =======================================================================================

		// region =============================== SMOOTHING TERM (TIKHONOV & KILLING) ======================

		if (switches.enableSmoothingTerm) {
			// region ============================== RETRIEVE NEIGHBOR'S WARPS =========================================

			const int neighborhoodSize = 9;
			Vector3f neighborFlowWarps[neighborhoodSize];
			bool neighborKnown[neighborhoodSize], neighborTruncated[neighborhoodSize], neighborAllocated[neighborhoodSize];

			//    0        1        2          3         4         5           6         7         8
			//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)

			findPoint2ndDerivativeNeighborhoodFlowWarp(
					neighborFlowWarps/*x9*/, neighborKnown, neighborTruncated,
					neighborAllocated, position, canonicalVoxels,
					canonicalIndexData, canonicalCache);

			for (int iNeighbor = 0; iNeighbor < neighborhoodSize; iNeighbor++) {
				if (!neighborAllocated[iNeighbor]) {
					//assign current warp to neighbor warp if the neighbor is not allocated
					neighborFlowWarps[iNeighbor] = framewiseWarp;
				}
			}
			//endregion=================================================================================================

			if (switches.enableKillingTerm) {
				Matrix3f framewiseWarpJacobian(0.0f);
				Matrix3f framewiseWarpHessian[3] = {Matrix3f(0.0f), Matrix3f(0.0f), Matrix3f(0.0f)};
				ComputePerVoxelWarpJacobianAndHessian(framewiseWarp, neighborFlowWarps, framewiseWarpJacobian,
				                                      framewiseWarpHessian);
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

				localSmoothingEnergy = localTikhonovEnergy + localKillingEnergy;
			} else {
				Matrix3f framewiseWarpJacobian(0.0f);
				Vector3f framewiseWarpLaplacian;
				ComputeWarpLaplacianAndJacobian(framewiseWarpLaplacian, framewiseWarpJacobian, framewiseWarp,
				                                neighborFlowWarps);

#ifndef __CUDACC__
				if (printVoxelResult) {
					_DEBUG_PrintTikhonovTermStuff(neighborFlowWarps, framewiseWarpLaplacian);
				}
#endif
				//∇E_{reg}(Ψ) = −[∆U ∆V ∆W]' ,
				localSmoothnessEnergyGradient = -framewiseWarpLaplacian;
				localTikhonovEnergy = dot(framewiseWarpJacobian.getColumn(0), framewiseWarpJacobian.getColumn(0)) +
				                      dot(framewiseWarpJacobian.getColumn(1), framewiseWarpJacobian.getColumn(1)) +
				                      dot(framewiseWarpJacobian.getColumn(2), framewiseWarpJacobian.getColumn(2));
				localSmoothingEnergy = localTikhonovEnergy;
			}
		}
		// endregion
		// region =============================== COMPUTE ENERGY GRADIENT ==================================
		SetGradientFunctor<TVoxelCanonical, TVoxelCanonical::hasDebugInformation>::SetGradient(
				canonicalVoxel,
				parameters.weightDataTerm, parameters.weightLevelSetTerm, parameters.weightSmoothnessTerm,
				localDataEnergyGradient, localLevelSetEnergyGradient, localSmoothnessEnergyGradient,
				restrictZtrackingForDebugging);
		// endregion
		// region =============================== AGGREGATE VOXEL STATISTICS ===============================
		//skip aggregates for parallel cuda implementation
#ifndef __CUDACC__
		float energyGradientLength = ORUtils::length(canonicalVoxel.gradient0);//meters
		float warpLength = ORUtils::length(canonicalVoxel.flow_warp);
		double localEnergy = localDataEnergy + parameters.weightLevelSetTerm * localLevelSetEnergy
		                     + parameters.weightSmoothnessTerm * localSmoothingEnergy;

		totalDataEnergy += localDataEnergy;
		totalLevelSetEnergy += parameters.weightLevelSetTerm * localLevelSetEnergy;
		totalTikhonovEnergy += parameters.weightSmoothnessTerm * localTikhonovEnergy;
		totalKillingEnergy += parameters.weightSmoothnessTerm * localKillingEnergy;
		totalSmoothnessEnergy += parameters.weightSmoothnessTerm * localSmoothingEnergy;

		cumulativeCanonicalSdf += canonicalSdf;
		cumulativeLiveSdf += liveSdf;
		cumulativeWarpDist += ORUtils::length(canonicalVoxel.flow_warp);
		consideredVoxelCount += 1;
		// endregion

		// region ======================== FINALIZE RESULT PRINTING / RECORDING ========================================

		if (printVoxelResult) {
			PrintVoxelResult(localDataEnergyGradient, localLevelSetEnergyGradient, localSmoothnessEnergyGradient,
			                 canonicalVoxel.gradient0, energyGradientLength);
		}
		// endregion ===================================================================================================
#endif
	}

	void PrintVoxelResult(const Vector3f& localDataEnergyGradient,
	                      const Vector3f& localLevelSetEnergyGradient,
	                      const Vector3f& localSmoothnessEnergyGradient,
	                      const Vector3f& localCompleteEnergyGradient,
	                      float energyGradientLength
	) {
		std::cout << blue << "Data gradient: " << localDataEnergyGradient * -1;
		std::cout << cyan << " Level set gradient: " << localLevelSetEnergyGradient * -1;
		std::cout << yellow << " Smoothness gradient: " << localSmoothnessEnergyGradient * -1;
		std::cout << std::endl;
		std::cout << green << "Energy gradient: " << localCompleteEnergyGradient * -1 << reset;
		std::cout << " Energy gradient length: " << energyGradientLength << red
		          << " Gradients shown are negated." << reset << std::endl << std::endl;
	}

	void FinalizePrintAndRecordStatistics() {
#ifndef __CUDACC__
		double totalEnergy = totalDataEnergy + totalLevelSetEnergy + totalSmoothnessEnergy;

		std::cout << bright_cyan << "*** General Iteration Statistics ***" << reset << std::endl;
		PrintEnergyStatistics(this->switches.enableDataTerm, this->switches.enableLevelSetTerm,
		                      this->switches.enableSmoothingTerm, this->switches.enableKillingTerm,
		                      this->parameters.rigidityEnforcementFactor,
		                      totalDataEnergy, totalLevelSetEnergy, totalTikhonovEnergy,
		                      totalKillingEnergy, totalSmoothnessEnergy, totalEnergy);

		//save all energies to file
		ITMDynamicFusionLogger<TVoxelCanonical, TVoxelLive, TIndex>::Instance().RecordAndPlotEnergies(
				totalDataEnergy, totalLevelSetEnergy, totalKillingEnergy, totalSmoothnessEnergy, totalEnergy);


		CalculateAndPrintAdditionalStatistics(
				this->switches.enableDataTerm, this->switches.enableLevelSetTerm, cumulativeCanonicalSdf,
				cumulativeLiveSdf,
				cumulativeWarpDist, consideredVoxelCount, dataVoxelCount, levelSetVoxelCount);
#endif
	}


private:
	void ResetStatistics() {
#ifndef __CUDACC__
		cumulativeCanonicalSdf = 0.0;
		cumulativeLiveSdf = 0.0;
		cumulativeWarpDist = 0.0;
		consideredVoxelCount = 0;
		dataVoxelCount = 0;
		levelSetVoxelCount = 0;

		totalDataEnergy = 0.0;
		totalLevelSetEnergy = 0.0;
		totalTikhonovEnergy = 0.0;
		totalKillingEnergy = 0.0;
		totalSmoothnessEnergy = 0.0;
#endif
	}

	// *** data structure accessors
	ITMScene<TVoxelLive, TIndex>* liveScene;
	int sourceSdfIndex{};
	TVoxelLive* liveVoxels;
	typename TIndex::IndexData* liveIndexData;
	typename TIndex::IndexCache liveCache;

	ITMScene<TVoxelCanonical, TIndex>* canonicalScene;
	TVoxelCanonical* canonicalVoxels;
	typename TIndex::IndexData* canonicalIndexData;
	typename TIndex::IndexCache canonicalCache;

#ifndef __CUDACC__
	// *** statistical aggregates
	double cumulativeCanonicalSdf = 0.0;
	double cumulativeLiveSdf = 0.0;
	double cumulativeWarpDist = 0.0;
	unsigned int consideredVoxelCount = 0;
	unsigned int dataVoxelCount = 0;
	unsigned int levelSetVoxelCount = 0;

	double totalDataEnergy = 0.0;
	double totalLevelSetEnergy = 0.0;
	double totalTikhonovEnergy = 0.0;
	double totalKillingEnergy = 0.0;
	double totalSmoothnessEnergy = 0.0;
#endif
	// *** debuging / analysis variables
	bool hasFocusCoordinates{};
	Vector3i focusCoordinates;
	bool restrictZtrackingForDebugging = false;

	const typename ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::Parameters parameters;
	const typename ITMSceneMotionTracker<TVoxelCanonical, TVoxelLive, TIndex>::Switches switches;

};

}// namespace ITMLib
