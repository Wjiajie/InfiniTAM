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

//ITMLib
#include "../../ITMLib/Objects/Scene/ITMScene.h"

class vtkPoints;
class vtkPolyData;
class vtkActor;

using namespace ITMLib;

template <typename TVoxel, typename TIndex>
class SDFSceneVizPipe {
public:
	//================= CONSTANTS ================
	static const double maxVoxelDrawSize;
	static const char* colorPointAttributeName;
	static const char* scalePointAttributeName;

	SDFSceneVizPipe(double* negativeSDFVoxelColor, double* positiveSDFVoxelColor, double* hashBlockEdgeColor);
	~SDFSceneVizPipe();
	void PreparePipeline(vtkAlgorithmOutput* voxelSourceGeometry, vtkAlgorithmOutput* hashBlockSourceGeometry);

	ITMScene<TVoxel, TIndex>* GetScene();
private:
	void PrepareSceneForRendering();

	ITMScene<TVoxel, TIndex>* scene;
	// **individual voxels**
	vtkSmartPointer<vtkPoints> initialPoints;
	vtkSmartPointer<vtkPolyData> voxelPolydata;
	vtkSmartPointer<vtkActor> voxelActor;
	// **hash block grid**
	vtkSmartPointer<vtkPolyData> hashBlockGrid;
	vtkSmartPointer<vtkActor> hashBlockActor;
};




