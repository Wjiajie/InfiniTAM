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
	                const std::array<double, 4>& highlightVoxelColor, const std::array<double, 3>& hashBlockEdgeColor);
	~SDFSceneVizPipe();

	// ====================== MEMBER FUNCTIONS ===========================
	virtual void PreparePipeline(vtkAlgorithmOutput* voxelSourceGeometry, vtkAlgorithmOutput* hashBlockSourceGeometry);

	ITMScene<TVoxel, TIndex>* GetScene();
	vtkSmartPointer<vtkActor>& GetVoxelActor();
	vtkSmartPointer<vtkActor>& GetHashBlockActor();
	VoxelScaleMode GetCurrentScaleMode();

	virtual void ToggleScaleMode();

protected:
	// ===================== STATIC FUNCTIONS ============================

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

	// ===================== MEMBER FUNCTIONS ===========================
	virtual void PreparePointsForRendering();

	// ===================== MEMBER VARIABLES ===========================
	ITMScene<TVoxel, TIndex>* scene;

	// ** individual voxels **
	vtkSmartPointer<vtkPolyData> voxelPolydata;
	// ** hash-block grid **
	vtkSmartPointer<vtkPolyData> hashBlockGrid;
	VoxelScaleMode scaleMode;
private:
	// ===================== MEMBER FUNCTIONS ===========================

	// ===================== MEMBER VARIABLES ===========================
	// ** voxels **
	vtkSmartPointer<vtkLookupTable> voxelColorLookupTable;
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



};




