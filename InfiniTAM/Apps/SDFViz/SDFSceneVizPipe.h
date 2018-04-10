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

//ITMLib
#include "../../ITMLib/Objects/Scene/ITMScene.h"
#include "../../ITMLib/Utils/FileIO/ITMSceneLogger.h"
#include "VizPipeShared.h"

class vtkPoints;
class vtkPolyData;
class vtkActor;
class vtkGlyph3DMapper;
class vtkGlyph3D;
class vtkPolyDataMapper;

using namespace ITMLib;

template <typename TVoxel, typename TIndex>
class SDFSceneVizPipe {
public:
	//================= CONSTANTS ================
	static const char* colorAttributeName;
	static const char* scaleUnknownsHiddenAttributeName;
	static const char* scaleUnknownsVisibleAttributeName;

	// ====================== CONSTRUCTORS / DESTRUCTORS ==================
	SDFSceneVizPipe(const std::array<double, 4>& positiveTruncatedVoxelColor,
	                const std::array<double, 4>& positiveNonTruncatedVoxelColor,
	                const std::array<double, 4>& negativeNonTruncatedVoxelColor,
	                const std::array<double, 4>& negativeTruncatedVoxelColor,
	                const std::array<double, 4>& unknownVoxelColor,
	                const std::array<double, 4>& highlightVoxelColor,
	                const std::array<double, 3>& hashBlockEdgeColor,
	                bool applyExtractionFilter);
	~SDFSceneVizPipe();

	// ====================== MEMBER FUNCTIONS ===========================
	virtual void PreparePipeline(vtkAlgorithmOutput* voxelSourceGeometry,
		                             vtkAlgorithmOutput* hashBlockSourceGeometry,
		                             const ITMScene<TVoxel, TIndex>* scene);

	virtual vtkSmartPointer<vtkActor>& GetVoxelActor();
	vtkSmartPointer<vtkActor>& GetHashBlockActor();
	VoxelScaleMode GetCurrentScaleMode();
	void ResetExtractionBounds();
	void SetExtractionBounds(const Vector3i& minPoint, const Vector3i& maxPoint);

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
	                                  vtkSmartPointer<vtkExtractPolyDataGeometry> extractor);
	static void SetUpSceneVoxelMapper(vtkAlgorithmOutput* sourceOutput, vtkSmartPointer<vtkGlyph3DMapper>& mapper,
	                                  vtkSmartPointer<vtkLookupTable>& table, vtkSmartPointer<vtkPolyData>& pointsPolydata);
	static void SetUpSceneVoxelMapperHelper(vtkAlgorithmOutput* sourceOutput, vtkSmartPointer<vtkGlyph3DMapper>& mapper,
	                                  vtkSmartPointer<vtkLookupTable>& table);
	// endregion
	// region ============= MEMBER FUNCTIONS =============================

	virtual void PreparePointsForRendering(const ITMScene<TVoxel, TIndex>* scene);

	// ===================== MEMBER VARIABLES ============================
	// ** individual voxels **
	vtkSmartPointer<vtkPolyData> voxelPolydata;
	vtkSmartPointer<vtkLookupTable> voxelColorLookupTable;
	vtkSmartPointer<vtkExtractPolyDataGeometry> extractionFilter;

	// ** hash-block grid **
	vtkSmartPointer<vtkPolyData> hashBlockGrid;
	VoxelScaleMode scaleMode;

	//** highlights **
	ITM3DNestedMapOfArrays<ITMHighlightIterationInfo> highlights;

	//** pipeline properties **
	const bool applyExtractionFilter;
	//endregion
private:
	// region ============== MEMBER FUNCTIONS ===========================

	// endregion
	// region ============== MEMBER VARIABLES ===========================

	// ** voxels **
	vtkSmartPointer<vtkGlyph3DMapper> voxelMapper;
	vtkSmartPointer<vtkActor> voxelActor;

	// ** hash block grid **
	vtkSmartPointer<vtkActor> hashBlockActor;
	vtkSmartPointer<vtkGlyph3DMapper> hashBlockMapper;

	// ** colors **
	std::array<double, 4> positiveTruncatedVoxelColor;
	std::array<double, 4> positiveNonTruncatedVoxelColor;
	std::array<double, 4> negativeNonTruncatedVoxelColor;
	std::array<double, 4> negativeTruncatedVoxelColor;
	std::array<double, 4> unknownVoxelColor;
	std::array<double, 4> highlightVoxelColor;
	std::array<double, 3> hashBlockEdgeColor;

	// ** scene limits/boundaries **
	Vector3i minPoint, maxPoint;
	//endregion
};




