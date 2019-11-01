//  ================================================================
//  Created by Gregory Kramida on 5/28/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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

#include "ITMDynamicSceneReconstructionEngine_CPU.h"
#include "../Shared/ITMDynamicSceneReconstructionEngine_Shared.h"
#include "../../Traversal/CPU/ITMSceneTraversal_CPU_PlainVoxelArray.h"
#include "../Shared/ITMDynamicSceneReconstructionEngine_Functors.h"

using namespace ITMLib;

// region ========================================= PLAIN VOXEL ARRAY ==================================================
template<typename TVoxel, typename TWarp>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, ITMPlainVoxelArray>::IntegrateIntoScene(
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, const ITMView* view,
		const ITMTrackingState* trackingState, const ITMRenderState* renderState) {
	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, M_rgb;
	Vector4f projParams_d, projParams_rgb;

	M_d = trackingState->pose_d->GetM();
	if (TVoxel::hasColorInformation) M_rgb = view->calib.trafo_rgb_to_depth.calib_inv * M_d;

	projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib.intrinsics_rgb.projectionParamsSimple.all;

	float mu = scene->sceneParams->mu;
	int maxW = scene->sceneParams->maxW;

	float* depth = view->depth->GetData(MEMORYDEVICE_CPU);
	Vector4u* rgb = view->rgb->GetData(MEMORYDEVICE_CPU);
	TVoxel* voxelArray = scene->localVBA.GetVoxelBlocks();

	const ITMPlainVoxelArray::IndexData* arrayInfo = scene->index.GetIndexData();

	float* confidence = view->depthConfidence->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int locId = 0; locId < scene->index.GetVolumeSize().x * scene->index.GetVolumeSize().y *
	                            scene->index.GetVolumeSize().z; ++locId) {


		int z = locId / (scene->index.GetVolumeSize().x * scene->index.GetVolumeSize().y);
		int tmp = locId - z * scene->index.GetVolumeSize().x * scene->index.GetVolumeSize().y;
		int y = tmp / scene->index.GetVolumeSize().x;
		int x = tmp - y * scene->index.GetVolumeSize().x;

		Vector4f pt_model;


		pt_model.x = (float) (x + arrayInfo->offset.x) * voxelSize;
		pt_model.y = (float) (y + arrayInfo->offset.y) * voxelSize;
		pt_model.z = (float) (z + arrayInfo->offset.z) * voxelSize;
		pt_model.w = 1.0f;

		ComputeUpdatedLiveVoxelInfo<
				TVoxel::hasColorInformation,
				TVoxel::hasConfidenceInformation,
				TVoxel::hasSemanticInformation,
				TVoxel>::compute(voxelArray[locId], pt_model, M_d,
		                         projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, confidence,
		                         depthImgSize, rgb, rgbImgSize);
	}
}

template<typename TVoxel, typename TWarp>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, ITMPlainVoxelArray>::FuseLiveIntoCanonicalSdf(
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* canonicalScene,
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* liveScene) {
	TSDFFusionFunctor<TVoxel> fusionFunctor(canonicalScene->sceneParams->maxW);
	ITMDualSceneTraversalEngine<TVoxel, TVoxel, ITMPlainVoxelArray, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::
	DualVoxelTraversal(liveScene, canonicalScene, fusionFunctor);
}

template<typename TVoxel, typename TWarp>
void
ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, ITMPlainVoxelArray>::GenerateRawLiveSceneFromView(
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, const ITMView* view,
		const ITMTrackingState* trackingState,
		const ITMRenderState* renderState) {
	this->sceneManager.ResetScene(scene);
	this->IntegrateIntoScene(scene, view, trackingState, renderState);
}

template<typename TVoxel, typename TWarp>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, ITMPlainVoxelArray>::UpdateVisibleList(
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene, const ITMView* view,
		const ITMTrackingState* trackingState,
		const ITMRenderState* renderState, bool resetVisibleList) {
	//do nothing
}


template<typename TVoxel, typename TWarp>
template<WarpType TWarpType>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, ITMPlainVoxelArray>::WarpScene(
		ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField,
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* sourceTSDF,
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* targetTSDF) {
//	 Clear out the flags at target index
	FieldClearFunctor<TVoxel> flagClearFunctor;
	ITMSceneTraversalEngine<TVoxel, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::VoxelTraversal(targetTSDF,
	                                                                                                      flagClearFunctor);

	TrilinearInterpolationFunctor<TVoxel, TWarp, ITMPlainVoxelArray,
			WarpVoxelStaticFunctor<TWarp, TWarpType>>
			trilinearInterpolationFunctor(sourceTSDF, warpField);

//	 Interpolate to obtain the new live frame values (at target index)
	ITMDualSceneTraversalEngine<TVoxel, TWarp, ITMPlainVoxelArray, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::
	DualVoxelPositionTraversal(targetTSDF, warpField, trilinearInterpolationFunctor);
}

template<typename TVoxel, typename TWarp>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, ITMPlainVoxelArray>::CopyScene(
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* sourceTSDF,
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* targetTSDF) {
	ITMDualSceneTraversalEngine<TVoxel, TVoxel, ITMPlainVoxelArray, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::
	        template StaticDualVoxelPositionTraversal<CopySceneFunctor<TVoxel> >(sourceTSDF, targetTSDF);
}

template<typename TVoxel, typename TWarp>
void
ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, ITMPlainVoxelArray>::WarpScene_CumulativeWarps(
		ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField,
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* sourceTSDF,
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* targetTSDF) {
	this->template WarpScene<WARP_CUMULATIVE>(warpField, sourceTSDF, targetTSDF);
}

template<typename TVoxel, typename TWarp>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, ITMPlainVoxelArray>::WarpScene_FlowWarps(
		ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField,
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* sourceTSDF,
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* targetTSDF) {
	this->template WarpScene<WARP_FLOW>(warpField, sourceTSDF, targetTSDF);
}

template<typename TVoxel, typename TWarp>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, ITMPlainVoxelArray>::WarpScene_WarpUpdates(
		ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField,
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* sourceTSDF,
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* targetTSDF) {
	this->template WarpScene<WARP_UPDATE>(warpField, sourceTSDF, targetTSDF);
}
// endregion ===========================================================================================================