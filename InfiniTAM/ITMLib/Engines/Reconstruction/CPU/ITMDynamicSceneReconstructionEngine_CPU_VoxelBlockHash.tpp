// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include <cfloat>
#include "ITMDynamicSceneReconstructionEngine_CPU.h"
#include "../Shared/ITMDynamicSceneReconstructionEngine_Shared.h"
#include "../../Traversal/CPU/ITMSceneTraversal_CPU_VoxelBlockHash.h"
#include "../Shared/ITMDynamicSceneReconstructionEngine_Functors.h"
#include "../../../Objects/RenderStates/ITMRenderState_VH.h"
#include "../../../Utils/Analytics/ITMSceneStatisticsCalculator.h"

using namespace ITMLib;




// region ======================================= MODIFY VOXELS BASED ON VIEW ==========================================

template<typename TVoxel, typename TWarp>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, ITMVoxelBlockHash>::IntegrateIntoScene(
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, const ITMView* view,
		const ITMTrackingState* trackingState, const ITMRenderState* renderState) {
	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, M_rgb;
	Vector4f projParams_d, projParams_rgb;

	ITMRenderState_VH* renderState_vh = (ITMRenderState_VH*) renderState;

	M_d = trackingState->pose_d->GetM();
	if (TVoxel::hasColorInformation) M_rgb = view->calib.trafo_rgb_to_depth.calib_inv * M_d;

	projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib.intrinsics_rgb.projectionParamsSimple.all;

	float mu = scene->sceneParams->mu;
	int maxW = scene->sceneParams->maxW;

	float* depth = view->depth->GetData(MEMORYDEVICE_CPU);
	float* confidence = view->depthConfidence->GetData(MEMORYDEVICE_CPU);
	Vector4u* rgb = view->rgb->GetData(MEMORYDEVICE_CPU);
	TVoxel* localVBA = scene->localVBA.GetVoxelBlocks();
	ITMHashEntry* hashTable = scene->index.GetEntries();

	int* visibleEntryIds = renderState_vh->GetVisibleEntryIDs();
	int noVisibleEntries = renderState_vh->noVisibleEntries;


#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int visibleHash = 0; visibleHash < noVisibleEntries; visibleHash++) {
		Vector3i globalPos;
		int hash = visibleEntryIds[visibleHash];
		const ITMHashEntry& currentHashEntry = hashTable[hash];

		if (currentHashEntry.ptr < 0) continue;

		globalPos.x = currentHashEntry.pos.x;
		globalPos.y = currentHashEntry.pos.y;
		globalPos.z = currentHashEntry.pos.z;
		globalPos *= SDF_BLOCK_SIZE;

		TVoxel* localVoxelBlock = &(localVBA[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

		for (int z = 0; z < SDF_BLOCK_SIZE; z++)
			for (int y = 0; y < SDF_BLOCK_SIZE; y++)
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					Vector4f pt_model;
					int locId;

					locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

					pt_model.x = (float) (globalPos.x + x) * voxelSize;
					pt_model.y = (float) (globalPos.y + y) * voxelSize;
					pt_model.z = (float) (globalPos.z + z) * voxelSize;
					pt_model.w = 1.0f;

					Vector3i pos(globalPos.x + x, globalPos.y + y, globalPos.z + z);


					ComputeUpdatedLiveVoxelInfo<
							TVoxel::hasColorInformation,
							TVoxel::hasConfidenceInformation,
							TVoxel::hasSemanticInformation,
							TVoxel>::compute(localVoxelBlock[locId], pt_model, M_d,
					                             projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, confidence,
					                             depthImgSize, rgb, rgbImgSize);
				}
	}
}
// endregion ===========================================================================================================
// region =================================== FUSION ===================================================================

template<typename TVoxel, typename TWarp>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, ITMVoxelBlockHash>::FuseLiveIntoCanonicalSdf(
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* canonicalScene,
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* liveScene) {
	this->hashManager.AllocateTSDFVolumeFromTSDFVolume(canonicalScene, liveScene);
	TSDFFusionFunctor<TVoxel> fusionFunctor(canonicalScene->sceneParams->maxW);
	ITMDualSceneTraversalEngine<TVoxel, TVoxel, ITMVoxelBlockHash, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::
	DualVoxelTraversal(liveScene, canonicalScene, fusionFunctor);
}

template<typename TVoxel, typename TWarp>
void
ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, ITMVoxelBlockHash>::GenerateRawLiveSceneFromView(
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, const ITMView* view, const ITMTrackingState* trackingState,
		const ITMRenderState* renderState) {
	this->sceneManager.ResetScene(scene);
	this->hashManager.AllocateFromDepth(scene, view, trackingState, renderState);
	this->IntegrateIntoScene(scene, view, trackingState, renderState);
}

// endregion ===========================================================================================================
// region ===================================== APPLY WARP/UPDATE TO LIVE ==============================================



/**
 * \brief Uses trilinear interpolation of the live frame at [canonical voxel positions + TWarpSource vector]
 *  (not complete warps) to generate a new live SDF grid in the target scene. Intended to be used at every iteration.
 * \details Assumes target frame is empty / has been reset.
 * Does 3 things:
 * <ol>
 *  <li> Traverses allocated canonical hash blocks and voxels, checks raw frame at [current voxel position + warp vector],
 *       if there is a non-truncated voxel in the live frame, marks the block in target SDF volume at the current hash
 *       block position for allocation
 *  <li> Traverses target hash blocks, if one is marked for allocation, allocates it
 *  <li> Traverses allocated target hash blocks and voxels, retrieves canonical voxel at same location, if it is marked
 *       as "truncated", skips it, otherwise uses trilinear interpolation at [current voxel position + warp vector] to
 *       retrieve SDF value from live frame, then stores it into the current target voxel, and marks latter voxel as
 *       truncated, non-truncated, or unknown based on the lookup flags & resultant SDF value.
 * </ol>
 * \param canonicalScene canonical scene with warp vectors at each voxel.
 * \param liveScene source (current iteration) live scene
 * \param targetLiveScene target (next iteration) live scene
 * \tparam TWarp type of warp vector to use for interpolation
 */
template<typename TVoxel, typename TWarp>
template<WarpType TWarpType>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, ITMVoxelBlockHash>::WarpScene(
		ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField,
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* sourceTSDF,
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* targetTSDF) {

	// Clear out the flags at target index
	FieldClearFunctor<TVoxel> flagClearFunctor;
	ITMSceneTraversalEngine<TVoxel, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::VoxelTraversal(targetTSDF,
	                                                                                                   flagClearFunctor);

	// Allocate new blocks where necessary, filter based on flags from source
	hashManager.template AllocateFromWarpedVolume<TWarpType>(warpField, sourceTSDF, targetTSDF);

	TrilinearInterpolationFunctor<TVoxel, TWarp, ITMVoxelBlockHash, WarpVoxelStaticFunctor<TWarp, TWarpType>>
			trilinearInterpolationFunctor(sourceTSDF, warpField);

	// Interpolate to obtain the new live frame values (at target index)
	ITMDualSceneTraversalEngine<TVoxel, TWarp, ITMVoxelBlockHash, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::
	DualVoxelPositionTraversal_DefaultForMissingSecondary(targetTSDF, warpField, trilinearInterpolationFunctor);
}

template<typename TVoxel, typename TWarp>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, ITMVoxelBlockHash>::UpdateVisibleList(
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene, const ITMView* view, const ITMTrackingState* trackingState,
		const ITMRenderState* renderState, bool resetVisibleList) {
	hashManager.AllocateFromDepth(scene, view, trackingState, renderState, true, resetVisibleList);
}

template<typename TVoxel, typename TWarp>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, ITMVoxelBlockHash>::CopyScene(
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* sourceTSDF,
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* targetTSDF) {
	hashManager.AllocateTSDFVolumeFromTSDFVolume(targetTSDF, sourceTSDF);
	ITMDualSceneTraversalEngine<TVoxel, TVoxel, ITMVoxelBlockHash, ITMVoxelBlockHash, ITMLibSettings::DEVICE_CPU>::
	        template StaticDualVoxelPositionTraversal<CopySceneFunctor<TVoxel>>(sourceTSDF, targetTSDF);

}

template<typename TVoxel, typename TWarp>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, ITMVoxelBlockHash>::WarpScene_CumulativeWarps(
		ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField,
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* sourceTSDF,
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* targetTSDF) {
	this->template WarpScene<WARP_CUMULATIVE>(warpField, sourceTSDF, targetTSDF);
}

template<typename TVoxel, typename TWarp>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, ITMVoxelBlockHash>::WarpScene_FlowWarps(
		ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField,
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* sourceTSDF,
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* targetTSDF) {
	this->template WarpScene<WARP_FLOW>(warpField, sourceTSDF, targetTSDF);
}

template<typename TVoxel, typename TWarp>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, ITMVoxelBlockHash>::WarpScene_WarpUpdates(
		ITMVoxelVolume<TWarp, ITMVoxelBlockHash>* warpField,
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* sourceTSDF,
		ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* targetTSDF) {
	this->template WarpScene<WARP_UPDATE>(warpField, sourceTSDF, targetTSDF);
}

// endregion ===========================================================================================================
