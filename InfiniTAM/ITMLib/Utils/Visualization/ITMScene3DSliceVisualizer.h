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
#include <thread>
#include <condition_variable>

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

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ThreadInteropCommand;

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMScene3DSliceVisualizerInteractorStyle;

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMScene3DSliceVisualizer {

	friend class ThreadInteropCommand<TVoxelCanonical, TVoxelLive, TIndex>;
	friend class ITMScene3DSliceVisualizerInteractorStyle<TVoxelCanonical, TVoxelLive, TIndex>;

public:
	//================= CONSTANTS ================
	static const char* colorAttributeName;
	static const char* scaleUnknownsHiddenAttributeName;
	static const char* scaleUnknownsVisibleAttributeName;

	// ====================== CONSTRUCTORS / DESTRUCTORS ==================
	ITMScene3DSliceVisualizer(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                          ITMScene<TVoxelLive, TIndex>* liveScene,
	                          Vector3i focusCoordinates, Plane plane = PLANE_XY,
	                          int radiusInPlane = 10, int radiusOutOfPlane = 0);
	virtual ~ITMScene3DSliceVisualizer();

	// ====================== MEMBER FUNCTIONS ===========================


	VoxelScaleMode GetCurrentScaleMode();
	void TriggerDrawWarpUpdates();
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

	//endregion


	VoxelScaleMode scaleMode;

	//** highlights **
	ITM3DNestedMapOfArrays<ITMHighlightIterationInfo> highlights;

	// scene limits/boundaries/extent
	Vector6i bounds;
	Vector3i focusCoordinates;

	ITM3DWindow* window;


private:
	// region ============== MEMBER FUNCTIONS ===========================
	void InitializeVoxels();
	template <typename TVoxel, typename TVoxelDataRetriever>
	void BuildVoxelAndHashBlockPolyDataFromScene(
			ITMScene <TVoxel, TIndex>* scene, vtkSmartPointer<vtkPolyData>& voxelVizData,
			vtkSmartPointer<vtkPolyData>& hashBlockVizData);
	void PreparePipeline();
	void InitializeWarps();
	void AddActorsToRenderers();
	void SetUpGeometrySources();
	void Run();
	void DrawWarpUpdates();


	// endregion
	// region ============== MEMBER VARIABLES ===========================

	// ===================== MEMBER VARIABLES ============================
	ITMScene<TVoxelCanonical, TIndex>* canonicalScene;
	ITMScene<TVoxelLive, TIndex>* liveScene;

	// ** individual voxels **
	vtkSmartPointer<vtkSphereSource> voxelVizGeometrySource;

	vtkSmartPointer<vtkPolyData> canonicalVoxelVizData;
	vtkSmartPointer<vtkPolyData> liveVoxelVizData;
	vtkSmartPointer<vtkGlyph3DMapper> canonicalVoxelMapper;
	vtkSmartPointer<vtkGlyph3DMapper> liveVoxelMapper;


	vtkSmartPointer<vtkActor> canonicalVoxelActor;
	vtkSmartPointer<vtkActor> liveVoxelActor;

	vtkSmartPointer<vtkLookupTable> canonicalVoxelColorLookupTable;
	vtkSmartPointer<vtkLookupTable> liveVoxelColorLookupTable;


	// ** hash-block grid **
	vtkSmartPointer<vtkCubeSource> hashBlockVizGeometrySource;

	vtkSmartPointer<vtkPolyData> canonicalHashBlockGridVizData;
	vtkSmartPointer<vtkPolyData> liveHashBlockGridVizData;
	vtkSmartPointer<vtkGlyph3DMapper> canonicalHashBlockMapper;
	vtkSmartPointer<vtkGlyph3DMapper> liveHashBlockMapper;
	vtkSmartPointer<vtkActor> canonicalHashBlockActor;
	vtkSmartPointer<vtkActor> liveHashBlockActor;

	// ** warp updates
	vtkSmartPointer<vtkPolyData> updatesData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyDataMapper> updatesMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkActor> updatesActor = vtkSmartPointer<vtkActor>::New();

	// ** colors **
	// = live =
	std::array<double, 4> livePositiveTruncatedVoxelColor = ITMLib::Viz::livePositiveTruncatedVoxelColor;
	std::array<double, 4> livePositiveNonTruncatedVoxelColor = ITMLib::Viz::livePositiveNonTruncatedVoxelColor;
	std::array<double, 4> liveNegativeNonTruncatedVoxelColor = ITMLib::Viz::liveNegativeNonTruncatedVoxelColor;
	std::array<double, 4> liveNegativeTruncatedVoxelColor = ITMLib::Viz::liveNegativeTruncatedVoxelColor;
	std::array<double, 4> liveUnknownVoxelColor = ITMLib::Viz::liveUnknownVoxelColor;
	std::array<double, 4> liveHighlightVoxelColor = ITMLib::Viz::highlightVoxelColor;
	std::array<double, 3> liveHashBlockEdgeColor = ITMLib::Viz::liveHashBlockEdgeColor;
	// = canonical =
	std::array<double, 4> canonicalPositiveTruncatedVoxelColor = ITMLib::Viz::canonicalPositiveTruncatedVoxelColor;
	std::array<double, 4> canonicalPositiveNonTruncatedVoxelColor = ITMLib::Viz::canonicalPositiveNonTruncatedVoxelColor;
	std::array<double, 4> canonicalNegativeNonTruncatedVoxelColor = ITMLib::Viz::canonicalNegativeNonTruncatedVoxelColor;
	std::array<double, 4> canonicalNegativeTruncatedVoxelColor = ITMLib::Viz::canonicalNegativeTruncatedVoxelColor;
	std::array<double, 4> canonicalUnknownVoxelColor = ITMLib::Viz::canonicalUnknownVoxelColor;
	std::array<double, 4> canonicalHighlightVoxelColor = ITMLib::Viz::highlightVoxelColor;
	std::array<double, 3> canonicalHashBlockEdgeColor = ITMLib::Viz::canonicalHashBlockEdgeColor;

	// ** threading controls **
	std::mutex mutex;
	std::condition_variable conditionVariable;
	bool initialized = false;
	bool warpUpdatePerformed = false;
	std::thread* thread = nullptr;
	vtkSmartPointer<ThreadInteropCommand<TVoxelCanonical, TVoxelLive, TIndex>> threadCallback;

	//endregion

};


}//namespace ITMLib;



