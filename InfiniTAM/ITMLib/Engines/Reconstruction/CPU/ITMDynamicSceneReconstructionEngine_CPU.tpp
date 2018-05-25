// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMDynamicSceneReconstructionEngine_CPU.h"

#include "../Shared/ITMDynamicSceneReconstructionEngine_Shared.h"
#include "../../../Objects/RenderStates/ITMRenderState_VH.h"
#include "../../../Objects/Scene/ITMSceneManipulation.h"

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

// endregion ===========================================================================================================
// region ========================================= PLAIN VOXEL ARRAY ==================================================

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::IntegrateIntoScene(
		ITMScene<TVoxelLive, ITMPlainVoxelArray>* scene, const ITMView* view,
		const ITMTrackingState* trackingState, const ITMRenderState* renderState) {
	Vector2i rgbImgSize = view->rgb->noDims;
	Vector2i depthImgSize = view->depth->noDims;
	float voxelSize = scene->sceneParams->voxelSize;

	Matrix4f M_d, M_rgb;
	Vector4f projParams_d, projParams_rgb;

	M_d = trackingState->pose_d->GetM();
	if (TVoxelLive::hasColorInformation) M_rgb = view->calib.trafo_rgb_to_depth.calib_inv * M_d;

	projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib.intrinsics_rgb.projectionParamsSimple.all;

	float mu = scene->sceneParams->mu;
	int maxW = scene->sceneParams->maxW;

	float* depth = view->depth->GetData(MEMORYDEVICE_CPU);
	Vector4u* rgb = view->rgb->GetData(MEMORYDEVICE_CPU);
	TVoxelLive* voxelArray = scene->localVBA.GetVoxelBlocks();

	const ITMPlainVoxelArray::IndexData* arrayInfo = scene->index.getIndexData();

	bool stopIntegratingAtMaxW = scene->sceneParams->stopIntegratingAtMaxW;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (int locId = 0; locId < scene->index.getVolumeSize().x * scene->index.getVolumeSize().y *
	                            scene->index.getVolumeSize().z; ++locId) {
		int z = locId / (scene->index.getVolumeSize().x * scene->index.getVolumeSize().y);
		int tmp = locId - z * scene->index.getVolumeSize().x * scene->index.getVolumeSize().y;
		int y = tmp / scene->index.getVolumeSize().x;
		int x = tmp - y * scene->index.getVolumeSize().x;

		Vector4f pt_model;

		if (stopIntegratingAtMaxW) if (voxelArray[locId].w_depth == maxW) continue;

		pt_model.x = (float) (x + arrayInfo->offset.x) * voxelSize;
		pt_model.y = (float) (y + arrayInfo->offset.y) * voxelSize;
		pt_model.z = (float) (z + arrayInfo->offset.z) * voxelSize;
		pt_model.w = 1.0f;

		ComputeUpdatedLiveVoxelInfo<
				TVoxelLive::hasColorInformation,
				TVoxelLive::hasConfidenceInformation,
				TVoxelLive::hasSemanticInformation,
				TVoxelLive>::compute(voxelArray[locId], pt_model, M_d, projParams_d, M_rgb, projParams_rgb, mu, maxW,
		                             depth, depthImgSize, rgb, rgbImgSize);
	}
}

template<typename TVoxelCanonical, typename TVoxelLive>
void ITMDynamicSceneReconstructionEngine_CPU<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::FuseFrame(
		ITMScene<TVoxelCanonical, ITMPlainVoxelArray>* canonicalScene,
		ITMScene<TVoxelLive, ITMPlainVoxelArray>* liveScene, int liveSourceFieldIndex) {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");//TODO
}

template<typename TVoxelCanonical, typename TVoxelLive>
void
ITMDynamicSceneReconstructionEngine_CPU<TVoxelCanonical, TVoxelLive, ITMPlainVoxelArray>::GenerateRawLiveSceneFromView(
		ITMScene<TVoxelLive, ITMPlainVoxelArray>* scene, const ITMView* view, const ITMTrackingState* trackingState,
		const ITMRenderState* renderState) {
	liveSceneManager.ResetScene(scene);
	this->IntegrateIntoScene(scene,view,trackingState,renderState);
}
// endregion ===========================================================================================================