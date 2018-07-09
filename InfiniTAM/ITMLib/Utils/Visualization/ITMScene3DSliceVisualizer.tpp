//  ================================================================
//  Created by Gregory Kramida on 1/22/18.
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

//stdlib
#include <sstream>

//VTK
#include <vtkActor.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkGlyph3DMapper.h>
#include <vtkBox.h>
#include <vtkProperty.h>
#include <vtkLookupTable.h>
#include <vtkPolyDataMapper.h>
#include <vtkCubeSource.h>
#include <vtkBox.h>
#include <vtkRenderWindow.h>
#include <mutex>
#include <vtkHedgeHog.h>
#include <vtkVertexGlyphFilter.h>


//Local
#include "ITMScene3DSliceVisualizer.h"
#include "ITMScene3DSliceVisualizerCommon.h"
#include "ITMVisualizationCommon.h"
#include "ITMScene3DSliceVisualizerInteractorStyle.h"
#include "../../Objects/Scene/ITMSceneTraversal_PlainVoxelArray.h"
#include "../../Objects/Scene/ITMSceneTraversal_VoxelBlockHash.h"


//TODO: alter OO design such that constructor isn't calling virual / override member functions -Greg (GitHub: Algomorph)

using namespace ITMLib;

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const char* ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::colorAttributeName = "color";
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const char* ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::scaleUnknownsHiddenAttributeName = "scale";
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const char* ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::scaleUnknownsVisibleAttributeName = "alternative_scale";


enum RequestedOperation {
	NONE,
	DRAW_WARP_UPDATES,
	DRAW_SMOOTHING_VECTORS,
	BUILD_FUSED_CANONICAL,
	UPDATE_LIVE_STATE

};

// Used for multithreaded interop with VTK
namespace ITMLib {
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ThreadInteropCommand : public vtkCommand {
public:
vtkTypeMacro(ThreadInteropCommand, vtkCommand);
	RequestedOperation operation = NONE;

	static ThreadInteropCommand* New() {
		return new ThreadInteropCommand;
	}

	void Execute(vtkObject*vtkNotUsed(caller), unsigned long vtkNotUsed(eventId),
	             void*vtkNotUsed(callData)) {
		switch (operation) {
			case NONE:
				break;
			case DRAW_WARP_UPDATES:
				parent->DrawWarpUpdates();
				break;
			case DRAW_SMOOTHING_VECTORS:
				parent->DrawSmoothnessVectors();
			case BUILD_FUSED_CANONICAL:
				parent->BuildFusedCanonicalFromCurrentScene();
			case UPDATE_LIVE_STATE:
				parent->UpdateLiveState();
			default:
				break;
		}
		operation = NONE;
	}

	ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>* parent;
};
} //namespace ITMLib


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::SceneSlice::SetUpMappersAndActors(
		std::array<double, 3>& hashBlockEdgeColor, vtkAlgorithmOutput* hashBlockGeometrySourceOutput,
		vtkAlgorithmOutput* voxelGeometrySourceOutput) {

	voxelMapper->SetInputData(voxelVizData);
	voxelMapper->SetSourceConnection(voxelGeometrySourceOutput);
	voxelMapper->SetLookupTable(voxelColorLookupTable);
	voxelMapper->ScalingOn();
	voxelMapper->SetScaleArray(scaleUnknownsVisibleAttributeName);
	voxelMapper->SetScaleModeToScaleByMagnitude();
	voxelMapper->ScalarVisibilityOn();
	voxelMapper->SetScalarModeToUsePointData();
	voxelMapper->SetColorModeToMapScalars();
	voxelMapper->SetScalarRange(0.0, static_cast<double>(COLOR_INDEX_COUNT));
	voxelMapper->InterpolateScalarsBeforeMappingOff();
	voxelMapper->Update();

	hashBlockMapper->SetInputData(hashBlockGridVizData);
	hashBlockMapper->SetSourceConnection(hashBlockGeometrySourceOutput);
	hashBlockMapper->ScalarVisibilityOff();
	hashBlockMapper->ScalingOff();
	hashBlockMapper->SetScaleFactor(1.0);

// set up voxel actors
	voxelActor->SetMapper(voxelMapper);
	voxelActor->GetProperty()->SetPointSize(20.0f);
	voxelActor->VisibilityOn();

	hashBlockActor->SetMapper(hashBlockMapper);
	hashBlockActor->GetProperty()->SetRepresentationToWireframe();
	hashBlockActor->GetProperty()->SetColor(hashBlockEdgeColor.data());
	hashBlockActor->VisibilityOff();

}

inline std::string to_string(Vector6i vector) {
	std::ostringstream s;
	s << vector;
	return s.str();
}

inline Vector6i ComputeBoundsAroundPoint(Vector3i point, int radiusInPlane, int radiusOutOfPlane, const Plane& plane) {
	Vector6i bounds;
	switch (plane) {
		case PLANE_YZ:
			bounds.min_y = point.y - radiusInPlane;
			bounds.max_y = point.y + radiusInPlane + 1;
			bounds.min_z = point.z - radiusInPlane;
			bounds.max_z = point.z + radiusInPlane + 1;
			bounds.min_x = point.x - radiusOutOfPlane;
			bounds.max_x = point.x + radiusOutOfPlane + 1;
			break;
		case PLANE_XZ:
			bounds.min_x = point.x - radiusInPlane;
			bounds.max_x = point.x + radiusInPlane + 1;
			bounds.min_z = point.z - radiusInPlane;
			bounds.max_z = point.z + radiusInPlane + 1;
			bounds.min_y = point.y - radiusOutOfPlane;
			bounds.max_y = point.y + radiusOutOfPlane + 1;
			break;
		case PLANE_XY:
			bounds.min_x = point.x - radiusInPlane;
			bounds.max_x = point.x + radiusInPlane + 1;
			bounds.min_y = point.y - radiusInPlane;
			bounds.max_y = point.y + radiusInPlane + 1;
			bounds.min_z = point.z - radiusOutOfPlane;
			bounds.max_z = point.z + radiusOutOfPlane + 1;
			break;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("PLANE NOT SUPPORTED");
	}
	return bounds;
}




// region ====================================== CONSTRUCTORS / DESTRUCTORS ============================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::ITMScene3DSliceVisualizer(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
		ITMScene<TVoxelLive, TIndex>* liveScene,
		Vector3i focusCoordinates, Plane plane,
		int radiusInPlane, int radiusOutOfPlane):
		canonicalScene(canonicalScene),
		liveScene(liveScene),
		bounds(ComputeBoundsAroundPoint(focusCoordinates, radiusInPlane, radiusOutOfPlane, plane)),
		focusCoordinates(focusCoordinates),
		scaleMode(VOXEL_SCALE_HIDE_UNKNOWNS) {
	this->thread = new std::thread(&ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::Run, this);
	std::unique_lock<std::mutex> lock(mutex);
	conditionVariable.wait(lock, [this] { return this->initialized; });
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::~ITMScene3DSliceVisualizer() {
}

// endregion ===========================================================================================================


// region ========================================== BUILD POLYDATA FROM VOXELS ========================================

template<typename TVoxel>
struct RetrieveVoxelDataBasicStaticFunctior {
	static float GetSdf(const TVoxel& voxel) {
		return TVoxel::valueToFloat(voxel.sdf);
	}

	static ITMLib::VoxelFlags GetFlags(const TVoxel& voxel) {
		return static_cast<ITMLib::VoxelFlags>(voxel.flags);
	}
};

template<typename TVoxel>
struct RetrieveVoxelDataFieldIndex0StaticFunctior {
	static float GetSdf(const TVoxel& voxel) {
		return TVoxel::valueToFloat(voxel.sdf_values[0]);
	}

	static ITMLib::VoxelFlags GetFlags(const TVoxel& voxel) {
		return static_cast<ITMLib::VoxelFlags>(voxel.flag_values[0]);
	}
};

template<typename TVoxel>
struct RetrieveVoxelDataFieldIndex1StaticFunctior {
	static float GetSdf(const TVoxel& voxel) {
		return TVoxel::valueToFloat(voxel.sdf_values[1]);
	}

	static ITMLib::VoxelFlags GetFlags(const TVoxel& voxel) {
		return static_cast<ITMLib::VoxelFlags>(voxel.flag_values[1]);
	}
};

template<typename TVoxel, typename VoxelDataRetrievalFunctor>
struct AddVoxelPointFunctor {
public:
	AddVoxelPointFunctor(vtkFloatArray* scaleAttribute,
	                     vtkFloatArray* alternativeScaleAttribute,
	                     vtkIntArray* colorAttribute,
	                     vtkPoints* voxelPoints,
	                     vtkPoints* hashBlockPoints,
	                     Vector3i focusCoordinates) :
			scaleAttribute(scaleAttribute),
			alternativeScaleAttribute(alternativeScaleAttribute),
			colorAttribute(colorAttribute),
			voxelPoints(voxelPoints),
			hashBlockPoints(hashBlockPoints),
			focusCoordinates(focusCoordinates) {}

	void operator()(const TVoxel& voxel, const Vector3i& position) {

		float sdf = VoxelDataRetrievalFunctor::GetSdf(voxel);
		ITMLib::VoxelFlags flags = VoxelDataRetrievalFunctor::GetFlags(voxel);
		float voxelScale = COMPUTE_VOXEL_SCALE_HIDE_UNKNOWNS(sdf, flags);
		float alternativeVoxelScale = COMPUTE_VOXEL_SCALE(sdf);

		int voxelColorIndex;

		if (position == focusCoordinates) {
			voxelColorIndex = HIGHLIGHT_SDF_COLOR_INDEX;
		} else {
			bool truncated = voxel.flags == ITMLib::VOXEL_TRUNCATED;
			voxelColorIndex = voxel.flags == ITMLib::VOXEL_UNKNOWN ? UNKNOWN_SDF_COLOR_INDEX :
			                  sdf > 0 ?
			                  (truncated ? POSITIVE_TRUNCATED_SDF_COLOR_INDEX : POSITIVE_NON_TRUNCATED_SDF_COLOR_INDEX)
			                          :
			                  (truncated ? NEGATIVE_TRUNCATED_SDF_COLOR_INDEX : NEGATIVE_NON_TRUNCATED_SDF_COLOR_INDEX);
		}

		voxelPoints->InsertNextPoint(position.x, position.y, position.z);
		scaleAttribute->InsertNextValue(voxelScale);
		alternativeScaleAttribute->InsertNextValue(alternativeVoxelScale);
		colorAttribute->InsertNextValue(voxelColorIndex);
		voxelCount++;
	}

	void processHashEntry(const ITMHashEntry& hashEntry) {
		Vector3i currentBlockPositionVoxels = hashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		const double centerOffset = -0.5;
		//draw hash block
		hashBlockPoints->InsertNextPoint((currentBlockPositionVoxels.x + centerOffset),
		                                 (currentBlockPositionVoxels.y + centerOffset),
		                                 (currentBlockPositionVoxels.z + centerOffset));
	}

	int voxelCount = 0;
private:
	Vector3i focusCoordinates;
	vtkFloatArray* scaleAttribute;
	vtkFloatArray* alternativeScaleAttribute;
	vtkIntArray* colorAttribute;
	vtkPoints* voxelPoints;
	vtkPoints* hashBlockPoints;

};
//
//template<typename TVoxel, typename TVoxelDataRetriever, typename TIndex>
//struct VoxelVisualizationStructureBuildFunctor;
//
//template<typename TVoxel, typename TVoxelDataRetriever>
//struct VoxelVisualizationStructureBuildFunctor<TVoxel, TVoxelDataRetriever, ITMVoxelBlockHash> {
//	static void BuildVisualizationStructures(vtkSmartPointer<vtkFloatArray> scaleAttribute,
//	                                         vtkSmartPointer<vtkFloatArray> alternativeScaleAttribute,
//	                                         vtkSmartPointer<vtkIntArray> colorAttribute,
//	                                         vtkSmartPointer<vtkPoints> voxelPoints,
//	                                         vtkSmartPointer<vtkPoints> hashBlockPoints, Vector3i focusCoordinates,
//	                                         ITMScene<TVoxel, ITMVoxelBlockHash>* scene, Vector6i bounds) {
//		AddVoxelPointFunctor<TVoxel, TVoxelDataRetriever> addVoxelPointFunctor(
//				scaleAttribute, alternativeScaleAttribute, colorAttribute, voxelPoints, hashBlockPoints,
//				focusCoordinates);
//		VoxelPositionAndHashEntryTraversalWithinBounds_CPU(scene, addVoxelPointFunctor, bounds);
//	}
//};
//
//
//template<typename TVoxel, typename TVoxelDataRetriever>
//struct VoxelVisualizationStructureBuildFunctor<TVoxel, TVoxelDataRetriever, ITMPlainVoxelArray> {
//	static void BuildVisualizationStructures(vtkSmartPointer<vtkFloatArray> scaleAttribute,
//	                                         vtkSmartPointer<vtkFloatArray> alternativeScaleAttribute,
//	                                         vtkSmartPointer<vtkIntArray> colorAttribute,
//	                                         vtkSmartPointer<vtkPoints> voxelPoints,
//	                                         vtkSmartPointer<vtkPoints> hashBlockPoints, Vector3i focusCoordinates,
//	                                         ITMScene<TVoxel, ITMPlainVoxelArray>* scene, Vector6i bounds) {
//		AddVoxelPointFunctor<TVoxel, TVoxelDataRetriever> addVoxelPointFunctor(
//				scaleAttribute, alternativeScaleAttribute, colorAttribute, voxelPoints, hashBlockPoints,
//				focusCoordinates);
//		VoxelPositionAndHashEntryTraversalWithinBounds_CPU(scene, addVoxelPointFunctor, bounds);
//	}
//};


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
template<typename TVoxel, typename TVoxelDataRetriever>
void ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::BuildVoxelAndHashBlockPolyDataFromScene(
		ITMScene<TVoxel, TIndex>* scene, SceneSlice& sceneSlice) {
	vtkSmartPointer<vtkPoints> voxelPoints = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> hashBlockPoints = vtkSmartPointer<vtkPoints>::New();

	//holds color for each voxel
	vtkSmartPointer<vtkIntArray> colorAttribute = vtkSmartPointer<vtkIntArray>::New();
	colorAttribute->SetName(colorAttributeName);

	//holds scale of each voxel
	vtkSmartPointer<vtkFloatArray> scaleAttribute = vtkSmartPointer<vtkFloatArray>::New();
	scaleAttribute->SetName(scaleUnknownsHiddenAttributeName);

	//holds alternative scale of each voxel (showing -1 value voxels)
	vtkSmartPointer<vtkFloatArray> alternativeScaleAttribute = vtkSmartPointer<vtkFloatArray>::New();
	alternativeScaleAttribute->SetName(scaleUnknownsVisibleAttributeName);

	AddVoxelPointFunctor<TVoxel, TVoxelDataRetriever> addVoxelPointFunctor(
			scaleAttribute, alternativeScaleAttribute, colorAttribute, voxelPoints, hashBlockPoints, focusCoordinates);
	VoxelPositionAndHashEntryTraversalWithinBounds_CPU(scene, addVoxelPointFunctor, bounds);
	sceneSlice.voxelCount = addVoxelPointFunctor.voxelCount;

	vtkSmartPointer<vtkPolyData> voxelVizData = sceneSlice.voxelVizData;
	voxelVizData->SetPoints(voxelPoints);
	voxelVizData->GetPointData()->AddArray(colorAttribute);
	voxelVizData->GetPointData()->AddArray(scaleAttribute);
	voxelVizData->GetPointData()->AddArray(alternativeScaleAttribute);
	voxelVizData->GetPointData()->SetActiveScalars(colorAttributeName);

	sceneSlice.hashBlockGridVizData->SetPoints(hashBlockPoints);
}

// endregion ===========================================================================================================
// region ========================== PRIVATE INIT MEMBER FUNCTIONS =====================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::BuildInitialSlices() {
	BuildVoxelAndHashBlockPolyDataFromScene<TVoxelCanonical, RetrieveVoxelDataBasicStaticFunctior<TVoxelCanonical>>(
			canonicalScene, canonicalSlice);
	BuildVoxelAndHashBlockPolyDataFromScene<TVoxelLive, RetrieveVoxelDataFieldIndex1StaticFunctior<TVoxelLive>>(
			liveScene, liveSlice);

	this->liveSliceStates.push_back(liveSlice.voxelVizData);

	canonicalSlice.SetUpMappersAndActors(canonicalHashBlockEdgeColor, hashBlockVizGeometrySource->GetOutputPort(),
	                                     voxelVizGeometrySource->GetOutputPort());
	liveSlice.SetUpMappersAndActors(liveHashBlockEdgeColor, hashBlockVizGeometrySource->GetOutputPort(),
	                                voxelVizGeometrySource->GetOutputPort());
	liveSlice.voxelActor->VisibilityOff();
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::AddSliceActorsToRenderers() {
	window->AddActorToFirstLayer(this->canonicalSlice.voxelActor);
	window->AddActorToFirstLayer(this->canonicalSlice.hashBlockActor);
	window->AddActorToFirstLayer(this->liveSlice.voxelActor);
	window->AddActorToFirstLayer(this->liveSlice.hashBlockActor);
}
// endregion ===========================================================================================================
// region ================================== STATIC INIT FUNCTIONS =====================================================
/**
 * \brief Sets up the geometry to use for voxel & hash block display
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::SetUpGeometrySources() {
	//Individual voxel shape
	voxelVizGeometrySource->SetThetaResolution(6);
	voxelVizGeometrySource->SetPhiResolution(6);
	voxelVizGeometrySource->SetRadius(0.5);//size of half a voxel
	voxelVizGeometrySource->Update();

	//Voxel hash block shape
	hashBlockVizGeometrySource->SetBounds(0, SDF_BLOCK_SIZE, 0, SDF_BLOCK_SIZE, 0, SDF_BLOCK_SIZE);
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::SetUpSDFColorLookupTable(
		vtkSmartPointer<vtkLookupTable>& table,
		const double* highlightColor,
		const double* positiveTruncatedColor,
		const double* positiveNonTruncatedColor,
		const double* negativeNonTruncatedColor,
		const double* negativeTruncatedColor,
		const double* unknownColor) {
	table->SetTableRange(0.0, static_cast<double>(COLOR_INDEX_COUNT));
	table->SetNumberOfTableValues(COLOR_INDEX_COUNT);
	table->SetNumberOfColors(COLOR_INDEX_COUNT);
	table->SetTableValue(POSITIVE_TRUNCATED_SDF_COLOR_INDEX, positiveTruncatedColor);
	table->SetTableValue(POSITIVE_NON_TRUNCATED_SDF_COLOR_INDEX, positiveNonTruncatedColor);
	table->SetTableValue(NEGATIVE_NON_TRUNCATED_SDF_COLOR_INDEX, negativeNonTruncatedColor);
	table->SetTableValue(NEGATIVE_TRUNCATED_SDF_COLOR_INDEX, negativeTruncatedColor);
	table->SetTableValue(UNKNOWN_SDF_COLOR_INDEX, unknownColor);
	table->SetTableValue(HIGHLIGHT_SDF_COLOR_INDEX, highlightColor);
	table->SetNanColor(0.4, 0.7, 0.1, 1.0);
}


// endregion ===========================================================================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::ToggleScaleMode() {
	if (scaleMode == VoxelScaleMode::VOXEL_SCALE_HIDE_UNKNOWNS) {
		scaleMode = VoxelScaleMode::VOXEL_SCALE_SHOW_UNKNOWNS;
		canonicalSlice.voxelMapper->SetScaleArray(scaleUnknownsVisibleAttributeName);
		liveSlice.voxelMapper->SetScaleArray(scaleUnknownsVisibleAttributeName);
	} else {
		scaleMode = VoxelScaleMode::VOXEL_SCALE_HIDE_UNKNOWNS;
		canonicalSlice.voxelMapper->SetScaleArray(scaleUnknownsHiddenAttributeName);
		liveSlice.voxelMapper->SetScaleArray(scaleUnknownsHiddenAttributeName);
	}
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
VoxelScaleMode ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::GetCurrentScaleMode() {
	return this->scaleMode;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::InitializeVoxels() {

	// *** initializations ***
	liveSlice.Initialize();
	canonicalSlice.Initialize();
	fusedCanonicalSlice.Initialize();
	scaleMode = VOXEL_SCALE_SHOW_UNKNOWNS;

	voxelVizGeometrySource = vtkSmartPointer<vtkSphereSource>::New();
	hashBlockVizGeometrySource = vtkSmartPointer<vtkCubeSource>::New();


	window = ITMVisualizationWindowManager::Instance().MakeOrGet3DWindow(
			"Scene3DSliceVisualizer" + to_string(this->bounds),
			"Scene 3D Slice Visualizer for bounds (" + to_string(this->bounds) + "))");

	// Create the color maps
	SetUpSDFColorLookupTable(liveSlice.voxelColorLookupTable, liveHighlightVoxelColor.data(),
	                         livePositiveTruncatedVoxelColor.data(),
	                         livePositiveNonTruncatedVoxelColor.data(),
	                         liveNegativeNonTruncatedVoxelColor.data(),
	                         liveNegativeTruncatedVoxelColor.data(), liveUnknownVoxelColor.data());
	SetUpSDFColorLookupTable(canonicalSlice.voxelColorLookupTable, canonicalHighlightVoxelColor.data(),
	                         canonicalPositiveTruncatedVoxelColor.data(),
	                         canonicalPositiveNonTruncatedVoxelColor.data(),
	                         canonicalNegativeNonTruncatedVoxelColor.data(),
	                         canonicalNegativeTruncatedVoxelColor.data(), canonicalUnknownVoxelColor.data());
	fusedCanonicalSlice.voxelColorLookupTable = canonicalSlice.voxelColorLookupTable;

	SetUpGeometrySources();
	BuildInitialSlices();
	AddSliceActorsToRenderers();

}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::InitializeWarps() {
	vtkSmartPointer<vtkPoints> updatePoints = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkFloatArray> updateVectors = vtkSmartPointer<vtkFloatArray>::New();
	updateVectors->SetName("Warp update vectors");
	updateVectors->SetNumberOfComponents(3);

	this->updatesData->SetPoints(updatePoints);
	this->updatesData->GetPointData()->SetVectors(updateVectors);

	vtkSmartPointer<vtkHedgeHog> hedgehog = vtkSmartPointer<vtkHedgeHog>::New();
	hedgehog->SetInputData(this->updatesData);
	updatesMapper->SetInputConnection(hedgehog->GetOutputPort());

	updatesActor->SetMapper(updatesMapper);
	updatesActor->GetProperty()->SetColor(0, 0, 0);

	this->window->AddLayer(Vector4d(1.0, 1.0, 1.0, 0.0));
	this->window->AddActorToLayer(updatesActor, 1);

	//TODO: DRY violation below

	vtkSmartPointer<vtkPoints> smoothingVectorPoints = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkFloatArray> smoothingVectorVectors = vtkSmartPointer<vtkFloatArray>::New();
	smoothingVectorVectors->SetName("Warp smoothingVector vectors");
	smoothingVectorVectors->SetNumberOfComponents(3);

	this->smoothingVectorData->SetPoints(smoothingVectorPoints);
	this->smoothingVectorData->GetPointData()->SetVectors(smoothingVectorVectors);

	vtkSmartPointer<vtkHedgeHog> smoothingVectorHedgehog = vtkSmartPointer<vtkHedgeHog>::New();
	smoothingVectorHedgehog->SetInputData(this->smoothingVectorData);
	smoothingVectorMapper->SetInputConnection(smoothingVectorHedgehog->GetOutputPort());

	smoothingVectorActor->SetMapper(smoothingVectorMapper);
	smoothingVectorActor->GetProperty()->SetColor(0, .8, .2);

	this->window->AddLayer(Vector4d(1.0, 1.0, 1.0, 0.0));
	this->window->AddActorToLayer(smoothingVectorActor, 1);
}

//region ============================= VISUALIZE WARP UPDATES ==========================================================

template<typename TVoxel>
struct TransferWarpUpdatesToVtkStructuresFunctor {
public:
	TransferWarpUpdatesToVtkStructuresFunctor(vtkPoints* points, vtkFloatArray* vectors) :
			points(points), vectors(vectors) {}

	void operator()(const TVoxel& voxel, const Vector3i& position) {
		Vector3f updateStartPoint = position.toFloat() + voxel.framewise_warp - voxel.warp_update;
		Vector3f updateVector = voxel.warp_update;
		points->InsertNextPoint(updateStartPoint.x, updateStartPoint.y, updateStartPoint.z);
		vectors->InsertNextTuple(updateVector.values);
	}

private:
	vtkPoints* points;
	vtkFloatArray* vectors;
};


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::DrawWarpUpdates() {
	std::unique_lock<std::mutex> lock(mutex);
	vtkPoints* updatePoints = this->updatesData->GetPoints();
	vtkFloatArray* updateVectors = vtkFloatArray::SafeDownCast(this->updatesData->GetPointData()->GetVectors());
	TransferWarpUpdatesToVtkStructuresFunctor<ITMVoxelCanonical> transferWarpUpdatesToVtkStructuresFunctor(
			updatePoints, updateVectors);
	VoxelPositionTraversalWithinBounds_CPU(this->canonicalScene, transferWarpUpdatesToVtkStructuresFunctor,
	                                       this->bounds);

	this->updatesData->Modified();
	this->window->Update();
	this->warpUpdatePerformed = true;
	lock.unlock();
	conditionVariable.notify_all();
}

static inline
std::string stringifyVoxelCoordinate(const Vector3i& position) {
	std::stringstream ss;
	ss << position;
	return ss.str();
}

template<typename TVoxel>
struct TransferSmoothingVectorsToVtkStructuresFunctor {
public:
	TransferSmoothingVectorsToVtkStructuresFunctor(vtkPoints* points, vtkFloatArray* vectors,
	                                               std::unordered_map<std::string, Vector3d>* smoothingHedgehogEndpoints)
			:
			points(points), vectors(vectors), smoothingHedgehogEndpoints(smoothingHedgehogEndpoints) {}

	void operator()(const TVoxel& voxel, const Vector3i& position) {
		Vector3d updateStartPoint;
		std::string positionAsString = stringifyVoxelCoordinate(position);
		auto iterator = smoothingHedgehogEndpoints->find(positionAsString);
		if (iterator == smoothingHedgehogEndpoints->end()) {
			updateStartPoint = position.toDouble();
		} else {
			updateStartPoint = iterator->second;
		}
		//TODO: remove magic value -- use -learningRate from the settings -Greg
		Vector3f updateVector = -0.1 * voxel.gradient1;
		points->InsertNextPoint(updateStartPoint.values);
		vectors->InsertNextTuple(updateVector.values);
		Vector3d endpoint = updateStartPoint + updateVector.toDouble();
		(*smoothingHedgehogEndpoints)[positionAsString] = endpoint;
	}

private:
	std::unordered_map<std::string, Vector3d>* smoothingHedgehogEndpoints;
	vtkPoints* points;
	vtkFloatArray* vectors;
};

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::DrawSmoothnessVectors() {
	std::unique_lock<std::mutex> lock(mutex);
	vtkPoints* updatePoints = this->smoothingVectorData->GetPoints();
	vtkFloatArray* updateVectors = vtkFloatArray::SafeDownCast(this->smoothingVectorData->GetPointData()->GetVectors());
	TransferSmoothingVectorsToVtkStructuresFunctor<ITMVoxelCanonical> TransferSmoothingVectorsToVtkStructuresFunctor(
			updatePoints, updateVectors, &this->smoothingHedgehogEndpoints);
	VoxelPositionTraversalWithinBounds_CPU(this->canonicalScene, TransferSmoothingVectorsToVtkStructuresFunctor,
	                                       this->bounds);

	this->smoothingVectorData->Modified();
	this->window->Update();
	this->smoothingVectorsDrawn = true;
	lock.unlock();
	conditionVariable.notify_all();
}

// endregion ===========================================================================================================


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::BuildFusedCanonicalFromCurrentScene() {
	std::unique_lock<std::mutex> lock(mutex);
	BuildVoxelAndHashBlockPolyDataFromScene<TVoxelCanonical, RetrieveVoxelDataBasicStaticFunctior<TVoxelCanonical>>(
			canonicalScene, fusedCanonicalSlice);

	fusedCanonicalSlice.SetUpMappersAndActors(canonicalHashBlockEdgeColor, hashBlockVizGeometrySource->GetOutputPort(),
	                                          voxelVizGeometrySource->GetOutputPort());
	this->window->AddActorToFirstLayer(fusedCanonicalSlice.voxelActor);
	this->window->AddActorToFirstLayer(fusedCanonicalSlice.hashBlockActor);
	SetVisibilityMode(VISIBILITY_FUSED_CANONICAL);
	this->fusedCanonicalBuilt = true;
	lock.unlock();
	conditionVariable.notify_all();
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::UpdateLiveState() {
	std::unique_lock<std::mutex> lock(mutex);
	liveSlice.voxelVizData = vtkSmartPointer<vtkPolyData>::New();
	switch (liveSamplingFieldIndex) {
		case 0:
			BuildVoxelAndHashBlockPolyDataFromScene<TVoxelLive, RetrieveVoxelDataFieldIndex0StaticFunctior<TVoxelLive>>(
					liveScene, liveSlice);
			break;
		case 1:
			BuildVoxelAndHashBlockPolyDataFromScene<TVoxelLive, RetrieveVoxelDataFieldIndex1StaticFunctior<TVoxelLive>>(
					liveScene, liveSlice);
			break;
		default:
			break;
	}
	liveSlice.voxelMapper->SetInputData(liveSlice.voxelVizData);
	liveSlice.voxelMapper->Modified();
	this->liveSliceStates.push_back(liveSlice.voxelVizData);
	this->window->Update();

	visibleOptimizationStepIndex++;
	liveSamplingFieldIndex = !liveSamplingFieldIndex;

	this->liveStateUpdated = true;
	lock.unlock();
	conditionVariable.notify_all();
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::Run() {
	std::unique_lock<std::mutex> lock(mutex);

	this->InitializeVoxels();
	this->InitializeWarps();


	threadCallback = vtkSmartPointer<ThreadInteropCommand<TVoxelCanonical, TVoxelLive, TIndex>>::New();
	threadCallback->parent = this;
	this->window->AddLoopCallback(threadCallback);
	vtkSmartPointer<ITMScene3DSliceVisualizerInteractorStyle<TVoxelCanonical, TVoxelLive, TIndex>> style =
			vtkSmartPointer<ITMScene3DSliceVisualizerInteractorStyle<TVoxelCanonical, TVoxelLive, TIndex>>::New();
	style->SetParent(this);

	this->window->SetInteractorStyle(style);

	window->ResetCamera();
	window->Update();

	initialized = true;
	lock.unlock();
	conditionVariable.notify_all();
	this->window->RunInteractor();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::TriggerDrawWarpUpdates() {
	this->threadCallback->operation = DRAW_WARP_UPDATES;
	std::unique_lock<std::mutex> lock(mutex);
	conditionVariable.wait(lock, [this] { return this->warpUpdatePerformed; });
	this->warpUpdatePerformed = false;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::TriggerDrawSmoothingVectors() {
	this->threadCallback->operation = DRAW_SMOOTHING_VECTORS;
	std::unique_lock<std::mutex> lock(mutex);
	conditionVariable.wait(lock, [this] { return this->smoothingVectorsDrawn; });
	this->smoothingVectorsDrawn = false;
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::TriggerUpdateLiveState() {
	this->threadCallback->operation = UPDATE_LIVE_STATE;
	std::unique_lock<std::mutex> lock(mutex);
	conditionVariable.wait(lock, [this] { return this->liveStateUpdated; });
	this->liveStateUpdated = false;
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::TriggerBuildFusedCanonical() {
	this->threadCallback->operation = BUILD_FUSED_CANONICAL;
	std::unique_lock<std::mutex> lock(mutex);
	conditionVariable.wait(lock, [this] { return this->fusedCanonicalBuilt; });
	this->fusedCanonicalBuilt = false;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::SetVisibilityMode(VisibilityMode mode) {
	this->visibilityMode = mode;
	switch (mode) {
		case VISIBILITY_CANONICAL_WITH_UPDATES:
			this->canonicalSlice.voxelActor->VisibilityOn();
			this->updatesActor->VisibilityOn();
			this->liveSlice.voxelActor->VisibilityOff();
			this->fusedCanonicalSlice.voxelActor->VisibilityOff();
			break;
		case VISIBILITY_LIVE:
			this->canonicalSlice.voxelActor->VisibilityOff();
			this->updatesActor->VisibilityOff();
			this->liveSlice.voxelActor->VisibilityOn();
			this->fusedCanonicalSlice.voxelActor->VisibilityOff();
			break;
		case VISIBILITY_LIVE_AND_CANONICAL_WITH_UPDATES:
			this->canonicalSlice.voxelActor->VisibilityOn();
			this->updatesActor->VisibilityOn();
			this->liveSlice.voxelActor->VisibilityOn();
			this->fusedCanonicalSlice.voxelActor->VisibilityOff();
			break;
		case VISIBILITY_FUSED_CANONICAL:
			this->canonicalSlice.voxelActor->VisibilityOff();
			this->updatesActor->VisibilityOff();
			this->liveSlice.voxelActor->VisibilityOff();
			this->fusedCanonicalSlice.voxelActor->VisibilityOn();
			break;
	}
	this->window->Update();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::AdvanceLiveStateVizualization() {
	if (visibleOptimizationStepIndex < this->liveSliceStates.size() - 1) {
		visibleOptimizationStepIndex++;
		this->liveSlice.voxelMapper->SetInputData(liveSliceStates[visibleOptimizationStepIndex]);
		liveSlice.voxelMapper->Modified();
		window->Update();
	}
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>::RetreatLiveStateVizualization() {
	if (visibleOptimizationStepIndex > 0) {
		visibleOptimizationStepIndex--;
		this->liveSlice.voxelMapper->SetInputData(liveSliceStates[visibleOptimizationStepIndex]);
		liveSlice.voxelMapper->Modified();
		window->Update();
	}
}