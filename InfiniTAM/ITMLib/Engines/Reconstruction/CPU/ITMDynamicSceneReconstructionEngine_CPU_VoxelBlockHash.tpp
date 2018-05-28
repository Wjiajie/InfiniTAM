// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include <cfloat>
#include "ITMDynamicSceneReconstructionEngine_CPU.h"

#include "../Shared/ITMDynamicSceneReconstructionEngine_Shared.h"
#include "../../../Objects/RenderStates/ITMRenderState_VH.h"
#include "../../../Objects/Scene/ITMSceneManipulation.h"
#include "../../../Objects/Scene/ITMSceneTraversal.h"
#include "../../../Objects/Scene/ITMTrilinearInterpolation.h"

using namespace ITMLib;


// region ======================================= MODIFY VOXELS BASED ON VIEW ==========================================

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::IntegrateIntoScene(
		ITMScene<TVoxelLive, ITMVoxelBlockHash>* scene, const ITMView* view,
		const ITMTrackingState* trackingState, const ITMRenderState* renderState) {
	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, M_rgb;
	Vector4f projParams_d, projParams_rgb;

	ITMRenderState_VH* renderState_vh = (ITMRenderState_VH*) renderState;

	M_d = trackingState->pose_d->GetM();
	if (TVoxelLive::hasColorInformation) M_rgb = view->calib.trafo_rgb_to_depth.calib_inv * M_d;

	projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib.intrinsics_rgb.projectionParamsSimple.all;

	float mu = scene->sceneParams->mu;
	int maxW = scene->sceneParams->maxW;

	float* depth = view->depth->GetData(MEMORYDEVICE_CPU);
	float* confidence = view->depthConfidence->GetData(MEMORYDEVICE_CPU);
	Vector4u* rgb = view->rgb->GetData(MEMORYDEVICE_CPU);
	TVoxelLive* localVBA = scene->localVBA.GetVoxelBlocks();
	ITMHashEntry* hashTable = scene->index.GetEntries();

	int* visibleEntryIds = renderState_vh->GetVisibleEntryIDs();
	int noVisibleEntries = renderState_vh->noVisibleEntries;

	bool stopIntegratingAtMaxW = scene->sceneParams->stopIntegratingAtMaxW;
	//bool approximateIntegration = !trackingState->requiresFullRendering;

	//_DEBUG
//	std::vector<Vector3i> _DEBUG_positions = {
//			Vector3i(1, 59, 217 ),
//			Vector3i(1, 59, 218 ),
//			Vector3i(1, 58, 213 ),
//			Vector3i(1, 58, 214 ),
//			Vector3i(1, 60, 220 ),
//			Vector3i(1, 60, 221 ),
//			Vector3i(1, 61, 224 ),
//			Vector3i(1, 61, 225 ),
//			Vector3i(-23, 62, 214 ),
//			Vector3i(-23, 62, 215 )};

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int visibleHash = 0; visibleHash < noVisibleEntries; visibleHash++) {
		Vector3i globalPos;
		int hash = visibleEntryIds[visibleHash];
		//_DEBUG
//		if(hash == 260246){
//			int i = 42;
//		}
		const ITMHashEntry& currentHashEntry = hashTable[hash];

		if (currentHashEntry.ptr < 0) continue;

		globalPos.x = currentHashEntry.pos.x;
		globalPos.y = currentHashEntry.pos.y;
		globalPos.z = currentHashEntry.pos.z;
		globalPos *= SDF_BLOCK_SIZE;

		TVoxelLive* localVoxelBlock = &(localVBA[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);

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
					//_DEBUG
//			Vector3i voxelPos = globalPos + Vector3i(x,y,z);
//			if(std::find(_DEBUG_positions.begin(),_DEBUG_positions.end(), voxelPos) != _DEBUG_positions.end()){
//				Vector4f pt_camera; Vector2f pt_image;
//				float depth_measure;
//				pt_camera = M_d * pt_model;
//				pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
//				pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
//				Vector2i pt_image_int = Vector2i((int)(pt_image.x + 0.5f), (int)(pt_image.y + 0.5f));
//				depth_measure = depth[(int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * depthImgSize.x];
//				std::cout << "Processed voxel at " << voxelPos  << ". Image sampled at: " << pt_image_int << ". Depth (m): " << depth_measure << ". Image float coord: " << pt_image << std::endl;
//			}
					ComputeUpdatedLiveVoxelInfo<
							TVoxelLive::hasColorInformation,
							TVoxelLive::hasConfidenceInformation,
							TVoxelLive::hasSemanticInformation,
							TVoxelLive>::compute(localVoxelBlock[locId], pt_model, M_d,
					                             projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, confidence,
					                             depthImgSize, rgb, rgbImgSize);
				}
	}
}
// endregion ===========================================================================================================
// region =================================== FUSION ===================================================================

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::FuseFrame(
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene,
		ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene, int liveSourceFieldIndex) {

	hashManager.AllocateCanonicalFromLive(canonicalScene,liveScene);

	TVoxelCanonical* canonicalVoxels = canonicalScene->localVBA.GetVoxelBlocks();
	ITMHashEntry* canonicalHashTable = canonicalScene->index.GetEntries();

	const TVoxelLive* liveVoxels = liveScene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* liveHashTable = liveScene->index.GetEntries();

	int maximumWeight = canonicalScene->sceneParams->maxW;
	int noTotalEntries = canonicalScene->index.noTotalEntries;

#ifdef WITH_OPENMP
#pragma omp parallel for reduction(+:missedKnownVoxels, sdfTruncatedCount, totalConf)
#endif
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
				stream << "Could not find corresponding canonical block at postion " << currentLiveHashEntry.pos << ". "
				       << __FILE__ << ": " << __LINE__;
				DIEWITHEXCEPTION(stream.str());
			}
			currentCanonicalHashEntry = canonicalHashTable[canonicalHash];
		}

		const TVoxelLive* localLiveVoxelBlock = &(liveVoxels[currentLiveHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		TVoxelCanonical* localCanonicalVoxelBlock = &(canonicalVoxels[currentCanonicalHashEntry.ptr *
		                                                              (SDF_BLOCK_SIZE3)]);

		//TODO: use traversal method / functor
		for (int z = 0; z < SDF_BLOCK_SIZE; z++) {
			for (int y = 0; y < SDF_BLOCK_SIZE; y++) {
				for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
					int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

					const TVoxelLive& liveVoxel = localLiveVoxelBlock[locId];
					if (liveVoxel.flag_values[liveSourceFieldIndex] != ITMLib::VOXEL_NONTRUNCATED) {
						continue;
					}
					TVoxelCanonical& canonicalVoxel = localCanonicalVoxelBlock[locId];
					int oldWDepth = canonicalVoxel.w_depth;
					float oldSdf = TVoxelCanonical::valueToFloat(canonicalVoxel.sdf);

					float liveSdf = TVoxelLive::valueToFloat(liveVoxel.sdf_values[liveSourceFieldIndex]);

					float newSdf = oldWDepth * oldSdf + liveSdf;
					float newWDepth = oldWDepth + 1.0f;
					newSdf /= newWDepth;
					newWDepth = MIN(newWDepth, maximumWeight);

					canonicalVoxel.sdf = TVoxelCanonical::floatToValue(newSdf);
					canonicalVoxel.w_depth = (uchar) newWDepth;

					canonicalVoxel.flags = ITMLib::VOXEL_NONTRUNCATED;
					hashManager.ChangeCanonicalHashEntryState(hash,ITMLib::STABLE);
				}
			}
		}
	}
}

template<typename TVoxelCanonical, typename TVoxelLive>
void
ITMDynamicSceneReconstructionEngine_CPU<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::GenerateRawLiveSceneFromView(
		ITMScene<TVoxelLive, ITMVoxelBlockHash>* scene, const ITMView* view, const ITMTrackingState* trackingState,
		const ITMRenderState* renderState) {
	liveSceneManager.ResetScene(scene);
	hashManager.AllocateLiveSceneFromDepth(scene,view,trackingState,renderState);
	this->IntegrateIntoScene(scene,view,trackingState,renderState);
}

// endregion ===========================================================================================================
// region ===================================== APPLY WARP/UPDATE TO LIVE ==============================================

template<typename TVoxelMulti>
struct IndexedFieldClearFunctor {
	IndexedFieldClearFunctor(int flagFieldIndex) : flagFieldIndex(flagFieldIndex) {}

	void operator()(TVoxelMulti& voxel) {
		voxel.flag_values[flagFieldIndex] = ITMLib::VOXEL_UNKNOWN;
		voxel.sdf_values[flagFieldIndex] = TVoxelMulti::SDF_initialValue();
	}

private:
	const int flagFieldIndex;
};



template<typename TVoxelWarpSource, typename TVoxelSdf>
struct TrilinearInterpolationFunctor {
	/**
	 * \brief Initialize to transfer data from source sdf scene to a target sdf scene using the warps in the warp source scene
	 * \details traverses
	 * \param sdfSourceScene
	 * \param warpSourceScene
	 */
	TrilinearInterpolationFunctor(
			ITMScene<TVoxelSdf, ITMVoxelBlockHash>* sdfSourceScene,
			ITMScene<TVoxelWarpSource, ITMVoxelBlockHash>* warpSourceScene,
			int sourceSdfIndex, int targetSdfIndex) :

			sdfSourceScene(sdfSourceScene),
			sdfSourceVoxels(sdfSourceScene->localVBA.GetVoxelBlocks()),
			sdfSourceHashEntries(sdfSourceScene->index.GetEntries()),
			sdfSourceCache(),

			warpSourceScene(warpSourceScene),
			warpSourceVoxels(warpSourceScene->localVBA.GetVoxelBlocks()),
			warpSourceHashEntries(warpSourceScene->index.GetEntries()),
			warpSourceCache(),
			sourceSdfIndex(sourceSdfIndex),
			targetSdfIndex(targetSdfIndex) {}


	void operator()(TVoxelSdf& destinationVoxel, Vector3i warpAndDestionVoxelPosition) {
		int vmIndex;
		// perform lookup at current position in canonical
		//TODO: perform dual traversal instead to get second voxel without lookup
		const TVoxelWarpSource& warpSourceVoxel = readVoxel(warpSourceVoxels, warpSourceHashEntries,
		                                                    warpAndDestionVoxelPosition, vmIndex, warpSourceCache);
		Vector3f warpedPosition = warpAndDestionVoxelPosition.toFloat() + warpSourceVoxel.gradient0;

		bool struckKnown, struckNonTruncated;
		float cumulativeWeight;
		float sdf = InterpolateMultiSdfTrilinearly_StruckKnown(
				sdfSourceVoxels, sdfSourceHashEntries, warpedPosition, sourceSdfIndex, sdfSourceCache, struckKnown);

		destinationVoxel.sdf_values[targetSdfIndex] = TVoxelSdf::floatToValue(sdf);

		// Update flags
		if (struckKnown) {
			if (1.0f - std::abs(sdf) < FLT_EPSILON) {
				destinationVoxel.flag_values[targetSdfIndex] = ITMLib::VOXEL_TRUNCATED;
			} else {
				destinationVoxel.flag_values[targetSdfIndex] = ITMLib::VOXEL_NONTRUNCATED;
			}
		}
	}


private:

	ITMScene<TVoxelSdf, ITMVoxelBlockHash>* sdfSourceScene;
	TVoxelSdf* sdfSourceVoxels;
	ITMHashEntry* sdfSourceHashEntries;
	ITMVoxelBlockHash::IndexCache sdfSourceCache;

	ITMScene<TVoxelWarpSource, ITMVoxelBlockHash>* warpSourceScene;
	TVoxelWarpSource* warpSourceVoxels;
	ITMHashEntry* warpSourceHashEntries;
	ITMVoxelBlockHash::IndexCache warpSourceCache;

	const int sourceSdfIndex;
	const int targetSdfIndex;
};

/**
 * \brief Uses trilinear interpolation of the live frame at [canonical voxel positions + scaled engergy gradient]
 *  (not complete warps) to generate a new live SDF grid in the target scene. Intended to be used at every iteration.
 * \details Assumes target frame is empty / has been reset.
 * Does 3 things:
 * <ol>
 *  <li> Traverses allocated canonical hash blocks and voxels, checks raw frame at [],
 *       if there is a non-truncated voxel in the live frame, marks the block in target sdf at the current canonical
 *       hash block position for allocation
 *  <li> Traverses target hash blocks, if one is marked for allocation, allocates it
 *  <li> Traverses allocated target hash blocks and voxels, retrieves canonical voxel at same location, if it is marked
 *       as "truncated", skips it, otherwise uses trilinear interpolation at [current voxel position + warp vector] to
 *       retrieve SDF value from live frame, then stores it into the current target voxel, and marks latter voxel as
 *       truncated, non-truncated, or unknown based on the lookup flags & resultant SDF value.
 * </ol>
 * \param canonicalScene canonical scene with warp vectors at each voxel.
 * \param liveScene source (current iteration) live scene
 * \param targetLiveScene target (next iteration) live scene
 */
template<typename TVoxelCanonical, typename TVoxelLive>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::ApplyWarpUpdateToLiveScene(
		ITMScene<TVoxelCanonical, ITMVoxelBlockHash>* canonicalScene,
		ITMScene<TVoxelLive, ITMVoxelBlockHash>* liveScene, int sourceSdfIndex, int targetSdfIndex) {

	// Clear out the flags at target index
	IndexedFieldClearFunctor<TVoxelLive> flagClearFunctor(targetSdfIndex);
	VoxelTraversal_CPU(*liveScene, flagClearFunctor);

	// Allocate new blocks where necessary, filter based on flags from source
	hashManager.AllocateWarpedLive(canonicalScene, liveScene, sourceSdfIndex);

	TrilinearInterpolationFunctor<TVoxelCanonical, TVoxelLive>
			trilinearInterpolationFunctor(liveScene, canonicalScene, sourceSdfIndex, targetSdfIndex);

	// Interpolate to obtain the new live frame values (at target index)
	VoxelPositionTraversal_CPU(liveScene, trilinearInterpolationFunctor);
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxelCanonical, TVoxelLive, ITMVoxelBlockHash>::UpdateVisibleList(
		ITMScene<TVoxelLive, ITMVoxelBlockHash>* scene, const ITMView* view, const ITMTrackingState* trackingState,
		const ITMRenderState* renderState, bool resetVisibleList) {
	hashManager.AllocateLiveSceneFromDepth(scene, view, trackingState, renderState, true, resetVisibleList);
}
// endregion ===========================================================================================================
