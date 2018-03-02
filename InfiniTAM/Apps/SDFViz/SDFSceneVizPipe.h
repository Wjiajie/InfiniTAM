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
#include "../../ITMLib/Utils/ITMSceneLogger.h"

#define COMPUTE_VOXEL_SCALE_HIDE_UNKNOWNS(sdf) (sdf == -1.0f ? 0.0f : 1.0f - 0.9f * std::abs(sdf))
#define COMPUTE_VOXEL_SCALE(sdf) (1.0f - 0.9f * std::abs(sdf))

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
	static const char* colorPointAttributeName;
	static const char* scalePointAttributeName;
	static const char* alternativeScalePointAttributeName;

	SDFSceneVizPipe(std::array<double,4> negativeSDFVoxelColor,
	                std::array<double,4> positiveSDFVoxelColor,
	                std::array<double,3> hashBlockEdgeColor);
	~SDFSceneVizPipe();

	virtual void PreparePipeline(vtkAlgorithmOutput* voxelSourceGeometry, vtkAlgorithmOutput* hashBlockSourceGeometry);

	ITMScene<TVoxel, TIndex>* GetScene();
	vtkSmartPointer<vtkActor>& GetVoxelActor();
	vtkSmartPointer<vtkActor>& GetHashBlockActor();

protected:
	ITMScene<TVoxel, TIndex>* scene;
	virtual void PreparePointsForRendering();
	// ** individual voxels **
	vtkSmartPointer<vtkPolyData> voxelPolydata;
	// ** hash-block grid **
	vtkSmartPointer<vtkPolyData> hashBlockGrid;


	static void SetUpSceneHashBlockMapper(vtkAlgorithmOutput* sourceOutput, vtkSmartPointer<vtkGlyph3DMapper>& mapper,
	                                      vtkSmartPointer<vtkPolyData>& pointsPolydata);
	static void SetUpSDFColorLookupTable(vtkSmartPointer<vtkLookupTable>& table, const double* rgbaFirstColor,
	                                     const double* rgbaSecondColor);
	static void SetUpGlyph(vtkAlgorithmOutput* sourceOutput, vtkSmartPointer<vtkPolyData>& polydata,
	                       vtkSmartPointer<vtkGlyph3D>& glyph);
	static void SetUpSceneVoxelMapper(vtkSmartPointer<vtkPolyDataMapper>& mapper, vtkSmartPointer<vtkLookupTable>& table,
	                                  vtkSmartPointer<vtkGlyph3D>& glyph);
	static void SetUpSceneVoxelMapper(vtkAlgorithmOutput* sourceOutput, vtkSmartPointer<vtkGlyph3DMapper>& mapper,
	                                  vtkSmartPointer<vtkLookupTable>& table,
	                                  vtkSmartPointer<vtkExtractPolyDataGeometry> extractor);
	static void SetUpSceneVoxelMapper(vtkAlgorithmOutput* sourceOutput, vtkSmartPointer<vtkGlyph3DMapper>& mapper,
	                                  vtkSmartPointer<vtkLookupTable>& table, vtkSmartPointer<vtkPolyData>& pointsPolydata);

private:

	// ** individual voxels **
	vtkSmartPointer<vtkLookupTable> voxelColorLookupTable;
	vtkSmartPointer<vtkGlyph3DMapper> voxelMapper;
	vtkSmartPointer<vtkActor> voxelActor;

	// ** hash block grid **
	vtkSmartPointer<vtkActor> hashBlockActor;
	vtkSmartPointer<vtkGlyph3DMapper> hashBlockMapper;

	// ** colors **
	std::array<double, 4> negativeVoxelColor;
	std::array<double, 4> positiveVoxelColor;
	std::array<double, 3> hashBlockEdgeColor;
	// ** scene limits/boundaries **
	Vector3i minPoint, maxPoint;


};




