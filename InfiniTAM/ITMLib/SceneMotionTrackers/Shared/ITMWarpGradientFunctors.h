//  ================================================================
//  Created by Gregory Kramida on 10/10/19.
//  Copyright (c) 2019 Gregory Kramida
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

#include "../../../ORUtils/PlatformIndependence.h"
#include "../../../ORUtils/PlatformIndependentAtomics.h"
#include "../../Utils/ITMMath.h"
#include "../../../ORUtils/JetbrainsCUDASyntax.hpp"
#include "ITMSceneMotionTracker_Shared.h"
#include "ITMSceneMotionOptimizationParameters.h"
#include <unordered_map>

namespace ITMLib {

template<typename TVoxel, typename TWarp>
struct ITMSceneMotionGradientTermFunctor {
	_CPU_AND_GPU_CODE_
	virtual void run(TVoxel& voxelLive, TVoxel& voxelCanonical, TWarp& warp, Vector3i voxelPosition) = 0;
	_CPU_AND_GPU_CODE_
	virtual float getEnergy() const = 0;
};


template<typename TVoxel, typename TWarp, typename TIndexData, typename TCache>
struct ITMSceneMotionDataTermFunctor
		: public ITMSceneMotionGradientTermFunctor<TVoxel, TWarp> {


	ITMSceneMotionDataTermFunctor(TVoxel* liveVoxels, const TIndexData* liveIndexData,
	                              float weight) :
			weight(weight), liveVoxels(liveVoxels), liveIndexData(liveIndexData), liveCache() {
		INITIALIZE_ATOMIC_FLOAT(energy, 0.0f);
	}

	~ITMSceneMotionDataTermFunctor() {
		CLEAN_UP_ATOMIC(energy);
	}

	float weight;
	TVoxel* liveVoxels;
	const TIndexData* liveIndexData;
	TCache liveCache;
	DECLARE_ATOMIC_FLOAT(energy);

	_CPU_AND_GPU_CODE_
	void operator()(TVoxel& voxelLive, TVoxel& voxelCanonical, TWarp& warp, Vector3i voxelPosition) {
		if (!VoxelIsConsideredForTracking(voxelCanonical, voxelLive)
		    || !VoxelIsConsideredForDataTerm(voxelCanonical, voxelLive))
			return;

		float liveSdf = TVoxel::valueToFloat(voxelLive.sdf);
		float canonicalSdf = TVoxel::valueToFloat(voxelCanonical.sdf);
		Vector3f liveSdfJacobian;

		ComputeLiveJacobian_CentralDifferences(
				liveSdfJacobian, voxelPosition, liveVoxels, liveIndexData, liveCache);

		// Compute data term energy & gradient

		// (φ_n(Ψ)−φ_{global}) ∇φ_n(Ψ) - also denoted as - (φ_{proj}(Ψ)−φ_{model}) ∇φ_{proj}(Ψ)
		// φ_n(Ψ) = φ_n(x+u, y+v, z+w), where u = u(x,y,z), v = v(x,y,z), w = w(x,y,z)
		// φ_{global} = φ_{global}(x, y, z)
		float sdfDifferenceBetweenLiveAndCanonical = liveSdf - canonicalSdf;
		float localDataEnergy =
				weight * 0.5f * (sdfDifferenceBetweenLiveAndCanonical * sdfDifferenceBetweenLiveAndCanonical);
		ATOMIC_ADD(energy, localDataEnergy);
		Vector3f localDataEnergyGradient = weight * sdfDifferenceBetweenLiveAndCanonical * liveSdfJacobian;
		warp.gradient += localDataEnergyGradient;

	}

	_CPU_AND_GPU_CODE_
	void run(TVoxel& voxelLive, TVoxel& voxelCanonical, TWarp& warp, Vector3i voxelPosition) override {
		this->operator()(voxelLive, voxelCanonical, warp, voxelPosition);
	}

	_CPU_AND_GPU_CODE_
	float getEnergy() const override {
		return GET_ATOMIC_VALUE(energy);
	}
};

template<typename TVoxel, typename TWarp, typename TIndexData, typename TCache>
struct ITMSceneMotionLevelSetTermFunctor : public ITMSceneMotionGradientTermFunctor<TVoxel, TWarp> {

	ITMSceneMotionLevelSetTermFunctor(TVoxel* liveVoxels, const TIndexData* liveIndexData,
	                                  float weight = 0.2f, float epsilon = 1e-5f, float unity = 0.1f) :
			liveVoxels(liveVoxels), liveIndexData(liveIndexData), liveCache(), epsilon(epsilon), unity(unity),
			weight(weight) {
		INITIALIZE_ATOMIC_FLOAT(energy, 0.0f);
	}

	~ITMSceneMotionLevelSetTermFunctor() {
		CLEAN_UP_ATOMIC(energy);
	}

	TVoxel* liveVoxels;
	const TIndexData* liveIndexData;
	TCache liveCache;
	DECLARE_ATOMIC_FLOAT(energy);
	const float epsilon;
	const float unity;
	const float weight;

	_CPU_AND_GPU_CODE_
	void operator()(TVoxel& voxelLive, TVoxel& voxelCanonical, TWarp& warp, Vector3i voxelPosition) {
		Matrix3f liveSdfHessian;
		Vector3f liveSdfJacobian;
		ComputeLiveJacobian_CentralDifferences(
				liveSdfJacobian, voxelPosition, liveVoxels, liveIndexData, liveCache);

		ComputeSdfHessian(liveSdfHessian, voxelPosition, TVoxel::valueToFloat(voxelLive.sdf), liveVoxels, liveIndexData,
		                  liveCache);

		float sdfJacobianNorm = ORUtils::length(liveSdfJacobian);
		float sdfJacobianNormMinusUnity = sdfJacobianNorm - unity;

		float localLevelSetEnergy =
				weight * 0.5f * (sdfJacobianNormMinusUnity * sdfJacobianNormMinusUnity);
		ATOMIC_ADD(energy, localLevelSetEnergy);

		Vector3f localLevelSetEnergyGradient =
				(sdfJacobianNormMinusUnity * weight) * (liveSdfHessian * liveSdfJacobian) /
				(sdfJacobianNorm + epsilon);
		warp.gradient += localLevelSetEnergyGradient;

	}

	_CPU_AND_GPU_CODE_
	void run(TVoxel& voxelLive, TVoxel& voxelCanonical, TWarp& warp, Vector3i voxelPosition) override {
		this->operator()(voxelLive, voxelCanonical, warp, voxelPosition);
	}


	_CPU_AND_GPU_CODE_
	float getEnergy() const override {
		return GET_ATOMIC_VALUE(energy);
	}
};


template<typename TVoxel, typename TWarp, typename TIndexData, typename TCache>
struct ITMSceneMotionTikhonovTermFunctor : public ITMSceneMotionGradientTermFunctor<TVoxel, TWarp> {

	ITMSceneMotionTikhonovTermFunctor(TVoxel* liveVoxels, const TIndexData* liveIndexData,
	                                  TVoxel* canonicalVoxels, const TIndexData* canonicalIndexData,
	                                  TWarp* warps, const TIndexData* warpIndexData,
	                                  float weight = 0.2f) :
			liveVoxels(liveVoxels), liveIndexData(liveIndexData),
			canonicalVoxels(canonicalVoxels), canonicalIndexData(canonicalIndexData),
			warps(warps), warpIndexData(warpIndexData),
			weight(weight) {
		INITIALIZE_ATOMIC_FLOAT(energy, 0.0f);
	}

	~ITMSceneMotionTikhonovTermFunctor() {
		CLEAN_UP_ATOMIC(energy);
	}

	TVoxel* liveVoxels;
	const TIndexData* liveIndexData;
	TWarp* warps;
	const TIndexData* warpIndexData;
	TVoxel* canonicalVoxels;
	const TIndexData* canonicalIndexData;
	TCache liveCache;
	TCache warpCache;
	TCache canonicalCache;


	DECLARE_ATOMIC_FLOAT(energy);
	const float weight;

	_CPU_AND_GPU_CODE_
	void operator()(TVoxel& voxelLive, TVoxel& voxelCanonical, TWarp& warp, Vector3i voxelPosition) {
		Vector3f& framewiseWarp = warp.flow_warp;
		const int neighborhoodSize = 9;
		Vector3f neighborFlowWarps[neighborhoodSize];
		bool neighborKnown[neighborhoodSize], neighborTruncated[neighborhoodSize], neighborAllocated[neighborhoodSize];

// region ============================== RETRIEVE NEIGHBOR'S WARPS =========================================
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
		Matrix3f framewiseWarpJacobian(0.0f);
		Vector3f framewiseWarpLaplacian;
		ComputeWarpLaplacianAndJacobian(framewiseWarpLaplacian, framewiseWarpJacobian, framewiseWarp,
		                                neighborFlowWarps);

		//∇E_{tikhonov}(Ψ) = −[∆U ∆V ∆W]' ,
		float localTikhonovEnergy = dot(framewiseWarpJacobian.getColumn(0), framewiseWarpJacobian.getColumn(0)) +
		                            dot(framewiseWarpJacobian.getColumn(1), framewiseWarpJacobian.getColumn(1)) +
		                            dot(framewiseWarpJacobian.getColumn(2), framewiseWarpJacobian.getColumn(2));

		ATOMIC_ADD(energy, localTikhonovEnergy);

		Vector3f localTikhonovEnergyGradient = this->weight * -framewiseWarpLaplacian;
		warp.gradient += localTikhonovEnergyGradient;

	}

	_CPU_AND_GPU_CODE_
	void run(TVoxel& voxelLive, TVoxel& voxelCanonical, TWarp& warp, Vector3i voxelPosition) override {
		this->operator()(voxelLive, voxelCanonical, warp, voxelPosition);
	}


	_CPU_AND_GPU_CODE_
	float getEnergy() const override {
		return GET_ATOMIC_VALUE(energy);
	}
};

//TODO: make a term that _ONLY_ uses the rigitity part (for easier balancing with Tikhonov)
//NB: combines Tikhonov & rigidity energy
template<typename TVoxel, typename TWarp, typename TIndexData, typename TCache>
struct ITMSceneMotionKillingTermFunctor : public ITMSceneMotionGradientTermFunctor<TVoxel, TWarp> {

	ITMSceneMotionKillingTermFunctor(TVoxel* liveVoxels, const TIndexData* liveIndexData,
	                                 TVoxel* canonicalVoxels, const TIndexData* canonicalIndexData,
	                                 TWarp* warps, const TIndexData* warpIndexData,
	                                 float weight = 0.5f, float rigidityWeight = 0.1f) :
			liveVoxels(liveVoxels), liveIndexData(liveIndexData),
			warps(warps), warpIndexData(warpIndexData),
			canonicalVoxels(canonicalVoxels), canonicalIndexData(canonicalIndexData),
			weight(weight), rigidityWeight(rigidityWeight) {
		INITIALIZE_ATOMIC_FLOAT(energy, 0.0f);
		INITIALIZE_ATOMIC_FLOAT(rigidityEnergy, 0.0f);
	}

	~ITMSceneMotionKillingTermFunctor() {
		CLEAN_UP_ATOMIC(smoothingEnergy);CLEAN_UP_ATOMIC(rigidityEnergy);
	}

	TVoxel* liveVoxels;
	const TIndexData* liveIndexData;
	TVoxel* canonicalVoxels;
	const TIndexData* canonicalIndexData;
	TWarp* warps;
	const TIndexData* warpIndexData;
	TCache liveCache;
	TCache warpCache;
	TCache canonicalCache;


	DECLARE_ATOMIC_FLOAT(energy);
	DECLARE_ATOMIC_FLOAT(rigidityEnergy);
	const float weight;
	const float rigidityWeight;

	_CPU_AND_GPU_CODE_
	void operator()(TVoxel& voxelLive, TVoxel& voxelCanonical, TWarp& warp, Vector3i voxelPosition) {
		Vector3f& framewiseWarp = warp.flow_warp;
		const int neighborhoodSize = 9;
		Vector3f neighborFlowWarps[neighborhoodSize];
		bool neighborKnown[neighborhoodSize], neighborTruncated[neighborhoodSize], neighborAllocated[neighborhoodSize];

// region ============================== RETRIEVE NEIGHBOR'S WARPS =========================================
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
		Matrix3f framewiseWarpJacobian(0.0f);
		Matrix3f framewiseWarpHessian[3] = {Matrix3f(0.0f), Matrix3f(0.0f), Matrix3f(0.0f)};
		ComputePerVoxelWarpJacobianAndHessian(framewiseWarp, neighborFlowWarps, framewiseWarpJacobian,
		                                      framewiseWarpHessian);

		float gamma = this->rigidityWeight;
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

		Vector3f localEnergyGradient = weight * ORUtils::Vector3<float>(KillingDeltaEu, KillingDeltaEv, KillingDeltaEw);
		warp.gradient += localEnergyGradient;
		//=================================== ENERGY ===============================================
		// KillingTerm Energy
		Matrix3f warpJacobianTranspose = framewiseWarpJacobian.t();

		float localTikhonovEnergy =
				weight *
				(dot(framewiseWarpJacobian.getColumn(0), framewiseWarpJacobian.getColumn(0)) +
				 dot(framewiseWarpJacobian.getColumn(1), framewiseWarpJacobian.getColumn(1)) +
				 dot(framewiseWarpJacobian.getColumn(2), framewiseWarpJacobian.getColumn(2)));

		float localRigidityEnergy =
				(gamma * weight) *
				(dot(warpJacobianTranspose.getColumn(0), framewiseWarpJacobian.getColumn(0)) +
				 dot(warpJacobianTranspose.getColumn(1), framewiseWarpJacobian.getColumn(1)) +
				 dot(warpJacobianTranspose.getColumn(2), framewiseWarpJacobian.getColumn(2)));

		float localSmoothingEnergy = localTikhonovEnergy + localRigidityEnergy;
		ATOMIC_ADD(energy, localSmoothingEnergy);
		ATOMIC_ADD(rigidityEnergy, localRigidityEnergy);
	}

	_CPU_AND_GPU_CODE_
	void run(TVoxel& voxelLive, TVoxel& voxelCanonical, TWarp& warp, Vector3i voxelPosition) override {
		this->operator()(voxelLive, voxelCanonical, warp, voxelPosition);
	}


	_CPU_AND_GPU_CODE_
	float getEnergy() const override {
		return GET_ATOMIC_VALUE(energy);
	}
};

enum GradientFunctorType {
	DATA = 0,
	LEVEL_SET = 1,
	TIKHONOV = 2,
	KILLING = 3,
	RIGIDITY = 4
};

template<typename TVoxel, typename TWarp, typename TIndexData, typename TCache>
class ITMSceneMotionEnergyGradientCompositeFunctor {
public:

	ITMSceneMotionEnergyGradientCompositeFunctor(ITMSceneMotionOptimizationParameters parameters,
	                                             ITMSceneMotionOptimizationSwitches switches,
	                                             TVoxel* liveVoxels, const TIndexData* liveIndexData,
	                                             TVoxel* canonicalVoxels, const TIndexData* canonicalIndexData,
	                                             TWarp* warps, const TIndexData* warpIndexData) :
			parameters(parameters), switches(switches),
			liveVoxels(liveVoxels), liveIndexData(liveIndexData),
			warps(warps), warpIndexData(warpIndexData),
			canonicalVoxels(canonicalVoxels), canonicalIndexData(canonicalIndexData) {
		functorCount = 0;
		if (switches.enableDataTerm) functorCount++;
		if (switches.enableLevelSetTerm) functorCount++;
		if (switches.enableSmoothingTerm) functorCount++;
		this->functors_CPU = new ITMSceneMotionGradientTermFunctor<TVoxel, TWarp>* [functorCount];
		this->functors_CUDA_host = new ITMSceneMotionGradientTermFunctor<TVoxel, TWarp>* [functorCount];
		int nextFunctorIx = 0;
		if (switches.enableDataTerm) {
			AddFunctor(GradientFunctorType::DATA, nextFunctorIx);
			nextFunctorIx++;
		}
		if (switches.enableLevelSetTerm) {
			AddFunctor(GradientFunctorType::LEVEL_SET, nextFunctorIx);
			nextFunctorIx++;
		}
		if (switches.enableSmoothingTerm) {
			if (switches.enableKillingTerm) {
				AddFunctor(GradientFunctorType::KILLING, nextFunctorIx);
				nextFunctorIx++;
			} else {
				AddFunctor(GradientFunctorType::TIKHONOV, nextFunctorIx);
				nextFunctorIx++;
			}
		}
#ifndef COMPILE_WITHOUT_CUDA
		size_t functorArraySize = functorCount * sizeof(ITMSceneMotionGradientTermFunctor<TVoxel, TWarp>*);
		ORcudaSafeCall(cudaMalloc((void**) &this->functors_CUDA_device, functorArraySize));
		ORcudaSafeCall(cudaMemcpy(this->functors_CUDA_device, this->functors_CUDA_host, functorArraySize, cudaMemcpyHostToDevice));
#endif

#ifdef __CUDACC__
		functors_Current = functors_CUDA_device;
#else
		functors_Current = functors_CPU;
#endif
	}

	_CPU_AND_GPU_CODE_
	void operator()(TVoxel& voxelLive, TVoxel& voxelCanonical, TWarp& warp, Vector3i voxelPosition) {
		for (int iFunctor = 0; iFunctor < functorCount; iFunctor++) {
			functors_Current[iFunctor]->run(voxelLive, voxelCanonical, warp, voxelPosition);
		}
	}

#ifndef COMPILE_WITHOUT_CUDA

	void UpdateHostFromDevice() {
		for (int iFunctor = 0; iFunctor < functorCount; iFunctor++) {
			ORcudaSafeCall(cudaMemcpy(functors_CPU[iFunctor], functors_CUDA_host[iFunctor], functorByteSizes[iFunctor],
			                          cudaMemcpyDeviceToHost));
		}
	}

	void UpdateDeviceFromHost() {
		for (int iFunctor = 0; iFunctor < functorCount; iFunctor++) {
			ORcudaSafeCall(cudaMemcpy(functors_CUDA_host[iFunctor], functors_CPU[iFunctor], functorByteSizes[iFunctor],
			                          cudaMemcpyHostToDevice));
		}
	}

#endif

	void PrintStatistics() {
		std::cout << bright_cyan << "*** Non-rigid Alignment Iteration Statistics ***" << reset << std::endl;
#ifdef __CUDACC__
		// assume calculation was done on the GPU
		UpdateHostFromDevice();
#endif

		std::cout << bright_cyan << "*** Non-rigid Alignment Iteration Statistics ***" << reset << std::endl;
		PrintEnergyStatistics(this->switches.enableDataTerm, this->switches.enableLevelSetTerm,
		                      this->switches.enableSmoothingTerm, this->switches.enableKillingTerm,
		                      this->parameters.rigidityEnforcementFactor,
		                      GetEnergy(DATA), GetEnergy(LEVEL_SET), GetEnergy(TIKHONOV),
		                      GetEnergy(RIGIDITY));
	}

	~ITMSceneMotionEnergyGradientCompositeFunctor() {
		for (int iFunctor = 0; iFunctor < functorCount; iFunctor++) {
			delete functors_CPU[iFunctor];
		}
		delete[] functors_CPU;
#ifndef COMPILE_WITHOUT_CUDA
		for (int iFunctor = 0; iFunctor < functorCount; iFunctor++) {
			ORcudaSafeCall(cudaFree(functors_CUDA_host[iFunctor]));
		}
		ORcudaSafeCall(cudaFree(functors_CUDA_device));
#endif
		delete[] functors_CUDA_host;
	}


private:
	const ITMSceneMotionOptimizationParameters parameters;
	const ITMSceneMotionOptimizationSwitches switches;

	TVoxel* liveVoxels;
	const TIndexData* liveIndexData;
	TVoxel* canonicalVoxels;
	const TIndexData* canonicalIndexData;
	TWarp* warps;
	const TIndexData* warpIndexData;

	void AddFunctor(GradientFunctorType type, int at) {
		auto addCudaFunctor = [&](size_t functorSize) {
#ifndef COMPILE_WITHOUT_CUDA
			functorByteSizes.push_back(functorSize);
			ORcudaSafeCall(cudaMalloc((void**) &functors_CUDA_host[at], functorSize));
			ORcudaSafeCall(cudaMemcpy(functors_CUDA_host[at], functors_CPU[at], functorSize, cudaMemcpyHostToDevice));
#endif
		};
		switch (type) {
			case DATA:
				functors_CPU[at] = new ITMSceneMotionDataTermFunctor<TVoxel, TWarp, TIndexData, TCache>(
						liveVoxels, liveIndexData, this->parameters.weightDataTerm);
				addCudaFunctor(sizeof(ITMSceneMotionDataTermFunctor<TVoxel, TWarp, TIndexData, TCache>));
				break;
			case LEVEL_SET:
				functors_CPU[at] = new ITMSceneMotionLevelSetTermFunctor<TVoxel, TWarp, TIndexData, TCache>(liveVoxels,
				                                                                                            liveIndexData);
				addCudaFunctor(sizeof(ITMSceneMotionLevelSetTermFunctor<TVoxel, TWarp, TIndexData, TCache>));
				break;
			case TIKHONOV:
				functors_CPU[at] = new ITMSceneMotionTikhonovTermFunctor<TVoxel, TWarp, TIndexData, TCache>
						(liveVoxels, liveIndexData, canonicalVoxels, canonicalIndexData, warps, warpIndexData,
						 parameters.weightSmoothingTerm);
				addCudaFunctor(sizeof(ITMSceneMotionTikhonovTermFunctor<TVoxel, TWarp, TIndexData, TCache>));
				break;
			case KILLING:
				functors_CPU[at] = new ITMSceneMotionKillingTermFunctor<TVoxel, TWarp, TIndexData, TCache>
						(liveVoxels, liveIndexData, canonicalVoxels, canonicalIndexData, warps, warpIndexData,
						 parameters.weightSmoothingTerm, parameters.rigidityEnforcementFactor);
				addCudaFunctor(sizeof(ITMSceneMotionKillingTermFunctor<TVoxel, TWarp, TIndexData, TCache>));
				break;
			default:
				DIEWITHEXCEPTION_REPORTLOCATION("Unknown functor type");
		}
		functorMap[type] = functors_CPU[at];
	}


	float GetRigidityEnergy() {
		float rigidityEnergy = 0.0f;
		if (functorMap.find(KILLING) != functorMap.end()) {
			auto killingFunctor = dynamic_cast<ITMSceneMotionKillingTermFunctor<TVoxel, TWarp, TIndexData, TCache>*>(functorMap[KILLING]);
			rigidityEnergy = GET_ATOMIC_VALUE(killingFunctor->rigidityEnergy);
		}
		return rigidityEnergy;
	}

	float GetEnergy(GradientFunctorType type) {
		if (type == RIGIDITY) {
			return GetRigidityEnergy();
		}
		if (functorMap.find(type) != functorMap.end()) {
			return functorMap[type]->getEnergy();
		} else if (type == TIKHONOV && functorMap.find(KILLING) != functorMap.end()) {
			float rigidityEnergy = GetRigidityEnergy();
			float smoothingEnergy = functorMap[KILLING]->getEnergy();
			return smoothingEnergy - rigidityEnergy;
		}
		return 0.0f;
	}

	int functorCount;
	ITMSceneMotionGradientTermFunctor<TVoxel, TWarp>** functors_Current;
	ITMSceneMotionGradientTermFunctor<TVoxel, TWarp>** functors_CPU;
	ITMSceneMotionGradientTermFunctor<TVoxel, TWarp>** functors_CUDA_host;
	ITMSceneMotionGradientTermFunctor<TVoxel, TWarp>** functors_CUDA_device;
	std::vector<size_t> functorByteSizes;
	std::unordered_map<GradientFunctorType, ITMSceneMotionGradientTermFunctor<TVoxel, TWarp>*> functorMap;
};

} // namespace ITMlib
