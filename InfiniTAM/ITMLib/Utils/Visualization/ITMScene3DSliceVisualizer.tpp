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


//Local
#include "ITMScene3DSliceVisualizer.h"
#include "ITMScene3DSliceVisualizerCommon.h"
#include "../../../Apps/SDFViz/SDFVizGlobalDefines.h"
#include "ITMVisualizationCommon.h"
#include "../../Objects/Scene/ITMSceneTraversal.h"


//TODO: alter OO design such that constructor isn't calling virual / override member functions -Greg (GitHub: Algomorph)

using namespace ITMLib;

template<typename TVoxel, typename TIndex>
const char* ITMScene3DSliceVisualizer<TVoxel, TIndex>::colorAttributeName = "color";
template<typename TVoxel, typename TIndex>
const char* ITMScene3DSliceVisualizer<TVoxel, TIndex>::scaleUnknownsHiddenAttributeName = "scale";
template<typename TVoxel, typename TIndex>
const char* ITMScene3DSliceVisualizer<TVoxel, TIndex>::scaleUnknownsVisibleAttributeName = "alternative_scale";

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

template<typename TVoxel, typename TIndex>
ITMScene3DSliceVisualizer<TVoxel, TIndex>::ITMScene3DSliceVisualizer(ITMScene<TVoxel, TIndex>* scene,
                                                                     Vector3i focusCoordinates, Plane plane,
                                                                     int radiusInPlane, int radiusOutOfPlane):

		scene(scene),
		bounds(ComputeBoundsAroundPoint(focusCoordinates, radiusInPlane, radiusOutOfPlane, plane)),
		scaleMode(VOXEL_SCALE_HIDE_UNKNOWNS) {

}


template<typename TVoxel, typename TIndex>
ITMScene3DSliceVisualizer<TVoxel, TIndex>::ITMScene3DSliceVisualizer(ITMScene<TVoxel, TIndex>* scene, Vector6i bounds)
		:

		scene(scene),
		bounds(bounds) {

}

template<typename TVoxel, typename TIndex>
ITMScene3DSliceVisualizer<TVoxel, TIndex>::ITMScene3DSliceVisualizer(ITMScene<TVoxel, TIndex>* scene, Vector3i bounds,
                                                                     const std::array<double, 4>& positiveTruncatedVoxelColor,
                                                                     const std::array<double, 4>& positiveNonTruncatedVoxelColor,
                                                                     const std::array<double, 4>& negativeNonTruncatedVoxelColor,
                                                                     const std::array<double, 4>& negativeTruncatedVoxelColor,
                                                                     const std::array<double, 4>& unknownVoxelColor,
                                                                     const std::array<double, 4>& highlightVoxelColor,
                                                                     const std::array<double, 3>& hashBlockEdgeColor)
		:
		scene(scene),
		bounds(bounds),

		positiveTruncatedVoxelColor(positiveTruncatedVoxelColor),
		positiveNonTruncatedVoxelColor(positiveNonTruncatedVoxelColor),
		negativeNonTruncatedVoxelColor(negativeNonTruncatedVoxelColor),
		negativeTruncatedVoxelColor(negativeTruncatedVoxelColor),
		unknownVoxelColor(unknownVoxelColor),
		highlightVoxelColor(highlightVoxelColor),
		hashBlockEdgeColor(hashBlockEdgeColor) {

}

template<typename TVoxel, typename TIndex>
ITMScene3DSliceVisualizer<TVoxel, TIndex>::~ITMScene3DSliceVisualizer() {
}

// endregion ===========================================================================================================


// region ========================================== BUILD POLYDATA ====================================================

template<typename TVoxel>
struct AddVoxelPointFunctor {
public:
	AddVoxelPointFunctor(vtkFloatArray* scaleAttribute,
	                     vtkFloatArray* alternativeScaleAttribute,
	                     vtkIntArray* colorAttribute,
	                     vtkPoints* voxelPoints,
	                     vtkPoints* hashBlockPoints) :
			scaleAttribute(scaleAttribute),
			alternativeScaleAttribute(alternativeScaleAttribute),
			colorAttribute(colorAttribute),
			voxelPoints(voxelPoints),
			hashBlockPoints(hashBlockPoints) {}

	void operator()(const TVoxel& voxel, const Vector3i& position) {

		float sdf = TVoxel::valueToFloat(voxel.sdf);
		float voxelScale = COMPUTE_VOXEL_SCALE_HIDE_UNKNOWNS(sdf, voxel.flags);
		float alternativeVoxelScale = COMPUTE_VOXEL_SCALE(sdf);
		bool truncated = voxel.flags == ITMLib::VOXEL_TRUNCATED;
		int voxelColor;

		voxelColor = voxel.flags == ITMLib::VOXEL_UNKNOWN ? UNKNOWN_SDF_COLOR_INDEX :
		             sdf > 0 ?
		             (truncated ? POSITIVE_TRUNCATED_SDF_COLOR_INDEX : POSITIVE_NON_TRUNCATED_SDF_COLOR_INDEX) :
		             (truncated ? NEGATIVE_TRUNCATED_SDF_COLOR_INDEX : NEGATIVE_NON_TRUNCATED_SDF_COLOR_INDEX);

		voxelPoints->InsertNextPoint(position.x, position.y,position.z);
		scaleAttribute->InsertNextValue(voxelScale);
		alternativeScaleAttribute->InsertNextValue(alternativeVoxelScale);
		colorAttribute->InsertNextValue(voxelColor);
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

private:
	vtkFloatArray* scaleAttribute;
	vtkFloatArray* alternativeScaleAttribute;
	vtkIntArray* colorAttribute;
	vtkPoints* voxelPoints;
	vtkPoints* hashBlockPoints;
	int voxelCount = 0;
};

template<typename TVoxel, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxel, TIndex>::BuildVoxelAndHashBlockPolydataFromScene() {
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

	AddVoxelPointFunctor<TVoxel> addVoxelPointFunctor(scaleAttribute, alternativeScaleAttribute, colorAttribute,
	                                                  voxelPoints,hashBlockPoints);
	VoxelPositionAndHashEntryTraversalWithinBounds_CPU(scene, addVoxelPointFunctor, bounds);

	//Points pipeline
	voxelVizData->SetPoints(voxelPoints);
	voxelVizData->GetPointData()->AddArray(colorAttribute);
	voxelVizData->GetPointData()->AddArray(scaleAttribute);
	voxelVizData->GetPointData()->AddArray(alternativeScaleAttribute);
	voxelVizData->GetPointData()->SetActiveScalars(colorAttributeName);

	hashBlockGridVizData->SetPoints(hashBlockPoints);
}

// endregion ===========================================================================================================

template<typename TVoxel, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxel, TIndex>::PreparePipeline() {
	BuildVoxelAndHashBlockPolydataFromScene();

	// set up hash block mapper
	SetUpSceneHashBlockMapper(hashBlockVizGeometrySource->GetOutputPort(), hashBlockMapper, hashBlockGridVizData);

	// set up voxel mapper
	SetUpSceneVoxelMapper(voxelVizGeometrySource->GetOutputPort(), voxelMapper, voxelColorLookupTable,
	                      voxelVizData);

	scaleMode = VOXEL_SCALE_HIDE_UNKNOWNS;// reset scale mode

	// set up voxel actor
	voxelActor->SetMapper(voxelMapper);
	voxelActor->GetProperty()->SetPointSize(20.0f);
	voxelActor->VisibilityOn();

	// set up hash block actor
	hashBlockActor->SetMapper(hashBlockMapper);
	hashBlockActor->GetProperty()->SetRepresentationToWireframe();
	hashBlockActor->GetProperty()->SetColor(hashBlockEdgeColor.data());
	hashBlockActor->VisibilityOff();
}


/**
 * \brief Sets up the geometry to use for voxel & hash block display
 */
template<typename TVoxel, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxel, TIndex>::SetUpGeometrySources() {
	//Individual voxel shape
	voxelVizGeometrySource->SetThetaResolution(6);
	voxelVizGeometrySource->SetPhiResolution(6);
	voxelVizGeometrySource->SetRadius(0.5);//size of half a voxel
	voxelVizGeometrySource->Update();

	//Voxel hash block shape
	hashBlockVizGeometrySource->SetBounds(0, SDF_BLOCK_SIZE, 0, SDF_BLOCK_SIZE, 0, SDF_BLOCK_SIZE);
}


template<typename TVoxel, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxel, TIndex>::SetUpSceneHashBlockMapper(vtkAlgorithmOutput* sourceOutput,
                                                                          vtkSmartPointer<vtkGlyph3DMapper>& mapper,
                                                                          vtkSmartPointer<vtkPolyData>& pointsPolydata) {
	mapper->SetInputData(pointsPolydata);
	mapper->SetSourceConnection(sourceOutput);
	mapper->ScalarVisibilityOff();
	mapper->ScalingOff();
	mapper->SetScaleFactor(1.0);
}

template<typename TVoxel, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxel, TIndex>::SetUpSDFColorLookupTable(vtkSmartPointer<vtkLookupTable>& table,
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


//GPU glyph version w/o filtering
template<typename TVoxel, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxel, TIndex>::SetUpSceneVoxelMapper(
		vtkAlgorithmOutput* sourceOutput,
		vtkSmartPointer<vtkGlyph3DMapper>& mapper,
		vtkSmartPointer<vtkLookupTable>& table,
		vtkSmartPointer<vtkPolyData>& pointsPolydata) {
	mapper->SetInputData(pointsPolydata);
	SetUpSceneVoxelMapperHelper(sourceOutput, mapper, table);
}

template<typename TVoxel, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxel, TIndex>::SetUpSceneVoxelMapperHelper(vtkAlgorithmOutput* sourceOutput,
                                                                            vtkSmartPointer<vtkGlyph3DMapper>& mapper,
                                                                            vtkSmartPointer<vtkLookupTable>& table) {
	mapper->SetSourceConnection(sourceOutput);
	mapper->SetLookupTable(table);
	mapper->ScalingOn();
	mapper->SetScaleArray(scaleUnknownsHiddenAttributeName);
	mapper->SetScaleModeToScaleByMagnitude();
	mapper->ScalarVisibilityOn();
	mapper->SetScalarModeToUsePointData();
	mapper->SetColorModeToMapScalars();
	mapper->SetScalarRange(0.0, static_cast<double>(COLOR_INDEX_COUNT));
	mapper->InterpolateScalarsBeforeMappingOff();
	mapper->Update();
}

template<typename TVoxel, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxel, TIndex>::ToggleScaleMode() {
	if (scaleMode == VoxelScaleMode::VOXEL_SCALE_HIDE_UNKNOWNS) {
		scaleMode = VoxelScaleMode::VOXEL_SCALE_SHOW_UNKNOWNS;
		voxelMapper->SetScaleArray(scaleUnknownsVisibleAttributeName);
	} else {
		scaleMode = VoxelScaleMode::VOXEL_SCALE_HIDE_UNKNOWNS;
		voxelMapper->SetScaleArray(scaleUnknownsHiddenAttributeName);
	}
}

template<typename TVoxel, typename TIndex>
VoxelScaleMode ITMScene3DSliceVisualizer<TVoxel, TIndex>::GetCurrentScaleMode() {
	return this->scaleMode;
}

template<typename TVoxel, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxel, TIndex>::Initialize() {
	// ** voxels **
	voxelVizData = vtkSmartPointer<vtkPolyData>::New();
	voxelColorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
	voxelVizGeometrySource = vtkSmartPointer<vtkSphereSource>::New();
	voxelActor = vtkSmartPointer<vtkActor>::New();
	voxelMapper = vtkSmartPointer<vtkGlyph3DMapper>::New();
	// ** hash-block grid **
	hashBlockGridVizData = vtkSmartPointer<vtkPolyData>::New();
	hashBlockVizGeometrySource = vtkSmartPointer<vtkCubeSource>::New();
	hashBlockActor = vtkSmartPointer<vtkActor>::New();
	hashBlockMapper = vtkSmartPointer<vtkGlyph3DMapper>::New();

	window = ITMVisualizationWindowManager::Instance().MakeOrGet3DWindow(
			"Scene3DSliceVisualizer" + to_string(this->bounds),
			"Scene 3D Slice Visualizer for bounds (" + to_string(this->bounds) + "))"),
	// Create the color maps
	SetUpSDFColorLookupTable(voxelColorLookupTable, highlightVoxelColor.data(), positiveTruncatedVoxelColor.data(),
	                         positiveNonTruncatedVoxelColor.data(), negativeNonTruncatedVoxelColor.data(),
	                         negativeTruncatedVoxelColor.data(), unknownVoxelColor.data());
	SetUpGeometrySources();
	PreparePipeline();
	AddActorsToRenderers();
	window->ResetCamera();
	window->Update();
}

template<typename TVoxel, typename TIndex>
void ITMScene3DSliceVisualizer<TVoxel, TIndex>::AddActorsToRenderers() {
	window->AddActorToFirstLayer(this->voxelActor);
	window->AddActorToFirstLayer(this->hashBlockActor);
}
