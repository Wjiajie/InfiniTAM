// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMVisualizationEngine_CPU.h"

#include "../Shared/ITMVisualisationEngine_Shared.h"
#include "../../Reconstruction/Shared/ITMSceneReconstructionEngine_Shared.h"

#include <vector>

using namespace ITMLib;

template<class TVoxel, class TIndex>
static int RenderPointCloud(Vector4u *outRendering, Vector4f *locations, Vector4f *colours, const Vector4f *ptsRay,
	const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, bool skipPoints, float voxelSize,
	Vector2i imgSize, Vector3f lightSource);

template<class TVoxel, class TIndex>
void ITMVisualizationEngine_CPU<TVoxel, TIndex>::FindVisibleBlocks(ITMVoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const
{
}

template<class TVoxel>
void ITMVisualizationEngine_CPU<TVoxel, VoxelBlockHash>::FindVisibleBlocks(
		ITMVoxelVolume<TVoxel, VoxelBlockHash>* scene, const ORUtils::SE3Pose* pose, const ITMIntrinsics* intrinsics,
		ITMRenderState* renderState) const
{
	const ITMHashEntry *hashTable = scene->index.GetEntries();
	int hashEntryCount = scene->index.hashEntryCount;
	float voxelSize = scene->sceneParams->voxel_size;
	Vector2i imgSize = renderState->renderingRangeImage->noDims;

	Matrix4f M = pose->GetM();
	Vector4f projParams = intrinsics->projectionParamsSimple.all;

	int visibleBlockCount = 0;
	int* visibleBlockHashCodes = scene->index.GetVisibleBlockHashCodes();

	//build visible list
	for (int targetIdx = 0; targetIdx < hashEntryCount; targetIdx++)
	{
		unsigned char hashVisibleType = 0;// = blockVisibilityTypes[targetIdx];
		const ITMHashEntry &hashEntry = hashTable[targetIdx];

		if (hashEntry.ptr >= 0)
		{
			bool isVisible, isVisibleEnlarged;
			checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M, projParams, voxelSize, imgSize);
			hashVisibleType = isVisible;
		}

		if (hashVisibleType > 0)
		{
			visibleBlockHashCodes[visibleBlockCount] = targetIdx;
			visibleBlockCount++;
		}
	}
	scene->index.SetVisibleHashBlockCount(visibleBlockCount);
}

template<class TVoxel, class TIndex>
int ITMVisualizationEngine_CPU<TVoxel, TIndex>::CountVisibleBlocks(const ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMRenderState *renderState, int minBlockId, int maxBlockId) const
{
	return 1;
}

template<class TVoxel>
int ITMVisualizationEngine_CPU<TVoxel, VoxelBlockHash>::CountVisibleBlocks(const ITMVoxelVolume<TVoxel,VoxelBlockHash> *scene, const ITMRenderState *renderState, int minBlockId, int maxBlockId) const
{

	int visibleBlockCount = scene->index.GetVisibleHashBlockCount();
	const int *visibleBlockHashCodes = scene->index.GetVisibleBlockHashCodes();

	int ret = 0;
	for (int i = 0; i < visibleBlockCount; ++i) {
		int blockID = scene->index.GetEntries()[visibleBlockHashCodes[i]].ptr;
		if ((blockID >= minBlockId)&&(blockID <= maxBlockId)) ++ret;
	}

	return ret;
}

template<class TVoxel, class TIndex>
void ITMVisualizationEngine_CPU<TVoxel, TIndex>::CreateExpectedDepths(const ITMVoxelVolume<TVoxel,TIndex> *scene,
		const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const
{
	Vector2i imgSize = renderState->renderingRangeImage->noDims;
	Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);

	for (int locId = 0; locId < imgSize.x*imgSize.y; ++locId) {
		//TODO : this could be improved a bit...
		Vector2f & pixel = minmaxData[locId];
		pixel.x = 0.2f;
		pixel.y = 3.0f;
	}
}

template<class TVoxel>
void ITMVisualizationEngine_CPU<TVoxel,VoxelBlockHash>::CreateExpectedDepths(const ITMVoxelVolume<TVoxel,VoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
                                                                                ITMRenderState *renderState) const
{
	Vector2i imgSize = renderState->renderingRangeImage->noDims;
	Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);

	for (int locId = 0; locId < imgSize.x*imgSize.y; ++locId) {
		Vector2f & pixel = minmaxData[locId];
		pixel.x = FAR_AWAY;
		pixel.y = VERY_CLOSE;
	}

	float voxelSize = scene->sceneParams->voxel_size;

	std::vector<RenderingBlock> renderingBlocks(MAX_RENDERING_BLOCKS);
	int numRenderingBlocks = 0;

	const int *visibleBlockHashCodes = scene->index.GetVisibleBlockHashCodes();
	int visibleEntryCount = scene->index.GetVisibleHashBlockCount();

	//go through list of visible 8x8x8 blocks
	for (int blockNo = 0; blockNo < visibleEntryCount; ++blockNo) {
		const ITMHashEntry & blockData(scene->index.GetEntries()[visibleBlockHashCodes[blockNo]]);

		Vector2i upperLeft, lowerRight;
		Vector2f zRange;
		bool validProjection = false;
		if (blockData.ptr>=0) {
			validProjection = ProjectSingleBlock(blockData.pos, pose->GetM(), intrinsics->projectionParamsSimple.all, imgSize, voxelSize, upperLeft, lowerRight, zRange);
		}
		if (!validProjection) continue;

		Vector2i requiredRenderingBlocks((int)ceilf((float)(lowerRight.x - upperLeft.x + 1) / (float)renderingBlockSizeX),
			(int)ceilf((float)(lowerRight.y - upperLeft.y + 1) / (float)renderingBlockSizeY));
		int requiredNumBlocks = requiredRenderingBlocks.x * requiredRenderingBlocks.y;

		if (numRenderingBlocks + requiredNumBlocks >= MAX_RENDERING_BLOCKS) continue;
		int offset = numRenderingBlocks;
		numRenderingBlocks += requiredNumBlocks;

		CreateRenderingBlocks(&(renderingBlocks[0]), offset, upperLeft, lowerRight, zRange);
	}

	// go through rendering blocks
	for (int blockNo = 0; blockNo < numRenderingBlocks; ++blockNo) {
		// fill minmaxData
		const RenderingBlock & b(renderingBlocks[blockNo]);

		for (int y = b.upperLeft.y; y <= b.lowerRight.y; ++y) {
			for (int x = b.upperLeft.x; x <= b.lowerRight.x; ++x) {
				Vector2f & pixel(minmaxData[x + y*imgSize.x]);
				if (pixel.x > b.zRange.x) pixel.x = b.zRange.x;
				if (pixel.y < b.zRange.y) pixel.y = b.zRange.y;
			}
		}
	}
}

template<typename TIndex> static inline HashBlockVisibility* GetBlockVisibilityTypes(TIndex& index);
template<> inline HashBlockVisibility* GetBlockVisibilityTypes<VoxelBlockHash>(VoxelBlockHash& index){
	return index.GetBlockVisibilityTypes();
}
template<> inline HashBlockVisibility* GetBlockVisibilityTypes<PlainVoxelArray>(PlainVoxelArray& index){
	return nullptr;
}

template<class TVoxel, class TIndex>
static void GenericRaycast(ITMVoxelVolume<TVoxel, TIndex> *scene, const Vector2i& imgSize, const Matrix4f& invM, const Vector4f& projParams, const ITMRenderState *renderState, bool updateVisibleList)
{
	const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
	float mu = scene->sceneParams->narrow_band_half_width;
	float oneOverVoxelSize = 1.0f / scene->sceneParams->voxel_size;
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
	const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *voxelIndex = scene->index.GetIndexData();
	HashBlockVisibility* blockVisibilityTypes = GetBlockVisibilityTypes(scene->index);

#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (int locId = 0; locId < imgSize.x*imgSize.y; ++locId)
	{
		int y = locId/imgSize.x;
		int x = locId - y*imgSize.x;

		int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

		if (blockVisibilityTypes != nullptr) castRay<TVoxel, TIndex, true>(
					pointsRay[locId],
					blockVisibilityTypes,
					x, y,
					voxelData,
					voxelIndex,
					invM,
					InvertProjectionParams(projParams),
					oneOverVoxelSize,
					mu,
					minmaximg[locId2]
			);
		else castRay<TVoxel, TIndex, false>(
				pointsRay[locId],
				NULL,
				x, y,
				voxelData,
				voxelIndex,
				invM,
				InvertProjectionParams(projParams),
				oneOverVoxelSize,
				mu,
				minmaximg[locId2]
			);
	}
}

template<class TVoxel, class TIndex>
static void RenderImage_common(ITMVoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
                               const ITMRenderState *renderState, ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type, IITMVisualisationEngine::RenderRaycastSelection raycastType)
{
	Vector2i imgSize = outputImage->noDims;
	Matrix4f invM = pose->GetInvM();

	Vector4f *pointsRay;
    if (raycastType == IITMVisualisationEngine::RENDER_FROM_OLD_RAYCAST)
        pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
    else
    {
        if (raycastType == IITMVisualisationEngine::RENDER_FROM_OLD_FORWARDPROJ)
            pointsRay = renderState->forwardProjection->GetData(MEMORYDEVICE_CPU);
        else
        {
            // this one is generally done for freeview visualisation, so
            // no, do not update the list of visible blocks
            GenericRaycast(scene, imgSize, invM, intrinsics->projectionParamsSimple.all, renderState, false);
            pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
        }
    }

	Vector3f lightSource = -Vector3f(invM.getColumn(2));
	Vector4u *outRendering = outputImage->GetData(MEMORYDEVICE_CPU);
	const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *voxelIndex = scene->index.GetIndexData();

	if ((type == IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME)&&
	    (!TVoxel::hasColorInformation)) type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;

	switch (type) {
	case IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME:
#ifdef WITH_OPENMP
		#pragma omp parallel for
#endif
		for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
		{
			Vector4f ptRay = pointsRay[locId];
			processPixelColour<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex);
		}
		break;
	case IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL:
#ifdef WITH_OPENMP
		#pragma omp parallel for
#endif
		for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
		{
			Vector4f ptRay = pointsRay[locId];
			processPixelNormal<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
		}
		break;
	case IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE:
#ifdef WITH_OPENMP
		#pragma omp parallel for
#endif
		for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
		{
			Vector4f ptRay = pointsRay[locId];
			processPixelConfidence<TVoxel, TIndex>(outRendering[locId], ptRay, ptRay.w > 0, voxelData, voxelIndex, lightSource);
		}
		break;
	case IITMVisualisationEngine::RENDER_SHADED_GREYSCALE_IMAGENORMALS:
#ifdef WITH_OPENMP
		#pragma omp parallel for
#endif
		for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
		{
			int y = locId/imgSize.x;
			int x = locId - y*imgSize.x;

			if (intrinsics->FocalLengthSignsDiffer())
			{
				processPixelGrey_ImageNormals<true, true>(outRendering, pointsRay, imgSize, x, y, scene->sceneParams->voxel_size, lightSource);
			}
			else
			{
				processPixelGrey_ImageNormals<true, false>(outRendering, pointsRay, imgSize, x, y, scene->sceneParams->voxel_size, lightSource);
			}
		}
		break;
		case IITMVisualisationEngine::RENDER_SHADED_GREEN:{
#ifdef WITH_OPENMP
			#pragma omp parallel for
#endif
			for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
			{
				Vector4f ptRay = pointsRay[locId];
				processPixelGreen<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData,
				                                  voxelIndex, lightSource);
			}
			break;
		}
		case IITMVisualisationEngine::RENDER_SHADED_OVERLAY:{
#ifdef WITH_OPENMP
			#pragma omp parallel for
#endif
			for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
			{
				Vector4f ptRay = pointsRay[locId];
				processPixelOverlay<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData,
				                                  voxelIndex, lightSource);
			}
			break;
		}
	case IITMVisualisationEngine::RENDER_SHADED_GREYSCALE:
	default:
#ifdef WITH_OPENMP
		#pragma omp parallel for
#endif
		for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
		{
			Vector4f ptRay = pointsRay[locId];
			processPixelGrey<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex, lightSource);
		}
	}
}

template<class TVoxel, class TIndex>
static void CreatePointCloud_common(ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState,
                                    ITMRenderState *renderState, bool skipPoints)
{
	Vector2i imgSize = renderState->raycastResult->noDims;
	Matrix4f invM = trackingState->pose_d->GetInvM() * view->calib.trafo_rgb_to_depth.calib;

	// this one is generally done for the colour tracker, so yes, update
	// the list of visible blocks if possible
	GenericRaycast(scene, imgSize, invM, view->calib.intrinsics_rgb.projectionParamsSimple.all, renderState, true);
	trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

	trackingState->pointCloud->noTotalPoints = RenderPointCloud<TVoxel, TIndex>(
		trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CPU),
		trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CPU),
		renderState->raycastResult->GetData(MEMORYDEVICE_CPU),
		scene->localVBA.GetVoxelBlocks(),
		scene->index.GetIndexData(),
		skipPoints,
		scene->sceneParams->voxel_size,
		imgSize,
		-Vector3f(invM.getColumn(2))
	);
}

template<class TVoxel, class TIndex>
static void CreateICPMaps_common(ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState)
{
	Vector2i imgSize = renderState->raycastResult->noDims;
	Matrix4f invM = trackingState->pose_d->GetInvM();

	// this one is generally done for the ICP tracker, so yes, update
	// the list of visible blocks if possible
	GenericRaycast(scene, imgSize, invM, view->calib.intrinsics_d.projectionParamsSimple.all, renderState, true);
	trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

	Vector3f lightSource = -Vector3f(invM.getColumn(2));
	Vector4f *normalsMap = trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CPU);
	Vector4f *pointsMap = trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CPU);
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
	float voxelSize = scene->sceneParams->voxel_size;

#ifdef WITH_OPENMP
	#pragma omp parallel for
#endif
	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
	{
		if (view->calib.intrinsics_d.FocalLengthSignsDiffer())
		{
			processPixelICP<true, true>(pointsMap, normalsMap, pointsRay, imgSize, x, y, voxelSize, lightSource);
		}
		else
		{
			processPixelICP<true, false>(pointsMap, normalsMap, pointsRay, imgSize, x, y, voxelSize, lightSource);
		}

	}
}

template<class TVoxel, class TIndex>
static void ForwardRender_common(const ITMVoxelVolume<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState)
{
	Vector2i imgSize = renderState->raycastResult->noDims;
	Matrix4f M = trackingState->pose_d->GetM();
	Matrix4f invM = trackingState->pose_d->GetInvM();
	const Vector4f& projParams = view->calib.intrinsics_d.projectionParamsSimple.all;

	const Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
	Vector4f *forwardProjection = renderState->forwardProjection->GetData(MEMORYDEVICE_CPU);
	float *currentDepth = view->depth->GetData(MEMORYDEVICE_CPU);
	int *fwdProjMissingPoints = renderState->fwdProjMissingPoints->GetData(MEMORYDEVICE_CPU);
	const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
	float voxelSize = scene->sceneParams->voxel_size;
	const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
	const typename TIndex::IndexData *voxelIndex = scene->index.GetIndexData();

	renderState->forwardProjection->Clear();

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
	{
		int locId = x + y * imgSize.x;
		Vector4f pixel = pointsRay[locId];

		int locId_new = forwardProjectPixel(pixel * voxelSize, M, projParams, imgSize);
		if (locId_new >= 0) forwardProjection[locId_new] = pixel;
	}

	int noMissingPoints = 0;
	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
	{
		int locId = x + y * imgSize.x;
		int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

		Vector4f fwdPoint = forwardProjection[locId];
		Vector2f minmaxval = minmaximg[locId2];
		float depth = currentDepth[locId];

		if ((fwdPoint.w <= 0) && ((fwdPoint.x == 0 && fwdPoint.y == 0 && fwdPoint.z == 0) || (depth >= 0)) && (minmaxval.x < minmaxval.y))
		//if ((fwdPoint.w <= 0) && (minmaxval.x < minmaxval.y))
		{
			fwdProjMissingPoints[noMissingPoints] = locId;
			noMissingPoints++;
		}
	}

	renderState->noFwdProjMissingPoints = noMissingPoints;
	const Vector4f invProjParams = InvertProjectionParams(projParams);

	for (int pointId = 0; pointId < noMissingPoints; pointId++)
	{
		int locId = fwdProjMissingPoints[pointId];
		int y = locId / imgSize.x, x = locId - y*imgSize.x;
		int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

		castRay<TVoxel, TIndex, false>(forwardProjection[locId], NULL, x, y, voxelData, voxelIndex, invM, invProjParams,
			1.0f / scene->sceneParams->voxel_size, scene->sceneParams->narrow_band_half_width, minmaximg[locId2]);
	}
}

template<class TVoxel, class TIndex>
void ITMVisualizationEngine_CPU<TVoxel,TIndex>::RenderImage(ITMVoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
                                                            const ITMRenderState *renderState, ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type,
                                                            IITMVisualisationEngine::RenderRaycastSelection raycastType) const
{
	RenderImage_common(scene, pose, intrinsics, renderState, outputImage, type, raycastType);
}

template<class TVoxel>
void ITMVisualizationEngine_CPU<TVoxel,VoxelBlockHash>::RenderImage(ITMVoxelVolume<TVoxel,VoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
                                                                       const ITMRenderState *renderState, ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type,
                                                                       IITMVisualisationEngine::RenderRaycastSelection raycastType) const
{
	RenderImage_common(scene, pose, intrinsics, renderState, outputImage, type, raycastType);
}

template<class TVoxel, class TIndex>
void ITMVisualizationEngine_CPU<TVoxel, TIndex>::FindSurface(ITMVoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const
{
	// this one is generally done for freeview visualisation, so no, do not
	// update the list of visible blocks
	GenericRaycast(scene, renderState->raycastResult->noDims, pose->GetInvM(), intrinsics->projectionParamsSimple.all, renderState, false);
}

template<class TVoxel>
void ITMVisualizationEngine_CPU<TVoxel,VoxelBlockHash>::FindSurface(ITMVoxelVolume<TVoxel,VoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
                                                                       const ITMRenderState *renderState) const
{
	// this one is generally done for freeview visualisation, so no, do not
	// update the list of visible blocks
	GenericRaycast(scene, renderState->raycastResult->noDims, pose->GetInvM(), intrinsics->projectionParamsSimple.all, renderState, false);
}

template<class TVoxel, class TIndex>
void ITMVisualizationEngine_CPU<TVoxel,TIndex>::CreatePointCloud(ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState,
                                                                 ITMRenderState *renderState, bool skipPoints) const
{
	CreatePointCloud_common(scene, view, trackingState, renderState, skipPoints);
}

template<class TVoxel>
void ITMVisualizationEngine_CPU<TVoxel,VoxelBlockHash>::CreatePointCloud(ITMVoxelVolume<TVoxel,VoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState,
                                                                            ITMRenderState *renderState, bool skipPoints) const
{
	CreatePointCloud_common(scene, view, trackingState, renderState, skipPoints);
}

template<class TVoxel, class TIndex>
void ITMVisualizationEngine_CPU<TVoxel,TIndex>::CreateICPMaps(ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const
{
	CreateICPMaps_common(scene, view, trackingState, renderState);
}

template<class TVoxel>
void ITMVisualizationEngine_CPU<TVoxel,VoxelBlockHash>::CreateICPMaps(ITMVoxelVolume<TVoxel,VoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState,
                                                                         ITMRenderState *renderState) const
{
	CreateICPMaps_common(scene, view, trackingState, renderState);
}

template<class TVoxel, class TIndex>
void ITMVisualizationEngine_CPU<TVoxel, TIndex>::ForwardRender(const ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState,
                                                               ITMRenderState *renderState) const
{
	ForwardRender_common(scene, view, trackingState, renderState);
}

template<class TVoxel>
void ITMVisualizationEngine_CPU<TVoxel, VoxelBlockHash>::ForwardRender(const ITMVoxelVolume<TVoxel,VoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState,
                                                                          ITMRenderState *renderState) const
{
	ForwardRender_common(scene, view, trackingState, renderState);
}

template<class TVoxel, class TIndex>
static int RenderPointCloud(Vector4f *locations, Vector4f *colours, const Vector4f *ptsRay,
	const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, bool skipPoints, float voxelSize,
	Vector2i imgSize, Vector3f lightSource)
{
	int noTotalPoints = 0;

	for (int y = 0, locId = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++, locId++)
	{
		Vector3f outNormal; float angle;
		Vector4f pointRay = ptsRay[locId];
		Vector3f point = pointRay.toVector3();
		bool foundPoint = pointRay.w > 0;

		computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

		if (skipPoints && ((x % 2 == 0) || (y % 2 == 0))) foundPoint = false;

		if (foundPoint)
		{
			Vector4f tmp;
			tmp = VoxelColorReader<TVoxel::hasColorInformation, TVoxel, TIndex>::interpolate(voxelData, voxelIndex, point);
			if (tmp.w > 0.0f) { tmp.x /= tmp.w; tmp.y /= tmp.w; tmp.z /= tmp.w; tmp.w = 1.0f; }
			colours[noTotalPoints] = tmp;

			Vector4f pt_ray_out;
			pt_ray_out.x = point.x * voxelSize; pt_ray_out.y = point.y * voxelSize;
			pt_ray_out.z = point.z * voxelSize; pt_ray_out.w = 1.0f;
			locations[noTotalPoints] = pt_ray_out;

			noTotalPoints++;
		}
	}

	return noTotalPoints;
}
