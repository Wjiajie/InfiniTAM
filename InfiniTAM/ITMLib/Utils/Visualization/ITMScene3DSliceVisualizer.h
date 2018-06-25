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
#pragma once

//VTK
#include <vtkSmartPointer.h>
#include <vtkExtractPolyDataGeometry.h>
#include <vtkSphereSource.h>
#include <vtkCubeSource.h>
#include <vtkPointSet.h>

//ITMLib
#include "../../Objects/Scene/ITMScene.h"
#include "../FileIO/ITMSceneLogger.h"
#include "ITMScene3DSliceVisualizerCommon.h"
#include "ITMVisualizationWindowManager.h"
#include "ITMVisualizationCommon.h"
#include "../../ITMLibDefines.h"

class vtkPoints;
class vtkPolyData;
class vtkActor;
class vtkGlyph3DMapper;

class vtkPolyDataMapper;

namespace ITMLib {

template<typename TVoxel, typename TIndex>
class ITMScene3DSliceVisualizer {
public:
	//================= CONSTANTS ================
	static const char* colorAttributeName;
	static const char* scaleUnknownsHiddenAttributeName;
	static const char* scaleUnknownsVisibleAttributeName;

	// ====================== CONSTRUCTORS / DESTRUCTORS ==================
	ITMScene3DSliceVisualizer(ITMScene<TVoxel, TIndex>* scene, Vector6i bounds);
	ITMScene3DSliceVisualizer(ITMScene<TVoxel, TIndex>* scene, Vector3i focusCoordinates, Plane plane = PLANE_XY,
	                          int radiusInPlane = 10, int radiusOutOfPlane = 0);

	ITMScene3DSliceVisualizer(ITMScene<TVoxel, TIndex>* scene, Vector3i bounds,
	                          const std::array<double, 4>& positiveTruncatedVoxelColor,
	                          const std::array<double, 4>& positiveNonTruncatedVoxelColor,
	                          const std::array<double, 4>& negativeNonTruncatedVoxelColor,
	                          const std::array<double, 4>& negativeTruncatedVoxelColor,
	                          const std::array<double, 4>& unknownVoxelColor,
	                          const std::array<double, 4>& highlightVoxelColor,
	                          const std::array<double, 3>& hashBlockEdgeColor);
	virtual ~ITMScene3DSliceVisualizer();

	// ====================== MEMBER FUNCTIONS ===========================


	VoxelScaleMode GetCurrentScaleMode();

	virtual void ToggleScaleMode();

protected:
	// region =============== STATIC FUNCTIONS ============================

	static void SetUpSceneHashBlockMapper(vtkAlgorithmOutput* sourceOutput, vtkSmartPointer<vtkGlyph3DMapper>& mapper,
	                                      vtkSmartPointer<vtkPolyData>& pointsPolydata);
	static void SetUpSDFColorLookupTable(vtkSmartPointer<vtkLookupTable>& table,
	                                     const double* highlightColor,
	                                     const double* positiveTruncatedColor,
	                                     const double* positiveNonTruncatedColor,
	                                     const double* negativeNonTruncatedColor,
	                                     const double* negativeTruncatedColor,
	                                     const double* unknownColor);
	static void SetUpSceneVoxelMapper(vtkAlgorithmOutput* sourceOutput, vtkSmartPointer<vtkGlyph3DMapper>& mapper,
	                                  vtkSmartPointer<vtkLookupTable>& table,
	                                  vtkSmartPointer<vtkPolyData>& pointsPolydata);
	static void SetUpSceneVoxelMapperHelper(vtkAlgorithmOutput* sourceOutput, vtkSmartPointer<vtkGlyph3DMapper>& mapper,
	                                        vtkSmartPointer<vtkLookupTable>& table);
	// endregion
	// region ============= MEMBER FUNCTIONS =============================
	virtual void BuildVoxelAndHashBlockPolydataFromScene();
	virtual void PreparePipeline();

	// ===================== MEMBER VARIABLES ============================
	// ** individual voxels **
	vtkSmartPointer<vtkPolyData> voxelVizData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkLookupTable> voxelColorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
	vtkSmartPointer<vtkSphereSource> voxelVizGeometrySource = vtkSmartPointer<vtkSphereSource>::New();
	vtkSmartPointer<vtkActor> voxelActor = vtkSmartPointer<vtkActor>::New();



	// ** hash-block grid **
	vtkSmartPointer<vtkPolyData> hashBlockGridVizData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkCubeSource> hashBlockVizGeometrySource = vtkSmartPointer<vtkCubeSource>::New();
	ITMScene<TVoxel, TIndex>* scene;

	VoxelScaleMode scaleMode;

	//** highlights **
	ITM3DNestedMapOfArrays<ITMHighlightIterationInfo> highlights;

	// scene limits/boundaries/extent
	Vector6i bounds;

	ITM3DWindow* window;

	//endregion
private:
	// region ============== MEMBER FUNCTIONS ===========================
	void Initialize();
	void AddActorsToRenderers();

	// endregion
	// region ============== MEMBER VARIABLES ===========================


	// ** voxels **
	vtkSmartPointer<vtkGlyph3DMapper> voxelMapper = vtkSmartPointer<vtkGlyph3DMapper>::New();



	// ** hash block grid **
	vtkSmartPointer<vtkActor> hashBlockActor = vtkSmartPointer<vtkActor>::New();
	vtkSmartPointer<vtkGlyph3DMapper> hashBlockMapper = vtkSmartPointer<vtkGlyph3DMapper>::New();

	// ** colors **
	std::array<double, 4> positiveTruncatedVoxelColor = ITMLib::Viz::liveTruncatedPositiveVoxelColor;
	std::array<double, 4> positiveNonTruncatedVoxelColor = ITMLib::Viz::liveNonTruncatedPositiveVoxelColor;
	std::array<double, 4> negativeNonTruncatedVoxelColor = ITMLib::Viz::liveNonTruncatedNegativeVoxelColor;
	std::array<double, 4> negativeTruncatedVoxelColor = ITMLib::Viz::liveTruncatedNegativeVoxelColor;
	std::array<double, 4> unknownVoxelColor = ITMLib::Viz::liveUnknownVoxelColor;
	std::array<double, 4> highlightVoxelColor = ITMLib::Viz::highlightVoxelColor;
	std::array<double, 3> hashBlockEdgeColor = ITMLib::Viz::liveHashBlockEdgeColor;

	//endregion
	void SetUpGeometrySources();
};


}//namespace ITMLib;



