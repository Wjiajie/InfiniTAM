//  ================================================================
//  Created by Gregory Kramida on 1/24/18.
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

#include "SDFSceneVizPipe.h"
#include "../../ITMLib/ITMLibDefines.h"

class CanonicalVizPipe : public SDFSceneVizPipe<ITMVoxelCanonical,ITMVoxelIndex> {
public:
	CanonicalVizPipe(const std::array<double, 4>& negativeNonInterestVoxelColor,
	                 const std::array<double, 4>& positiveNonInterestVoxelColor,
	                 const std::array<double, 4>& negativeInterestVoxelColor,
	                 const std::array<double, 4>& positiveInterestVoxelColor,
	                 const std::array<double, 3>& hashBlockEdgeColor);

	void UpdatePointPositionsFromBuffer(void* buffer);
	void SetInterestRegionHashes(std::set<int> interestRegions);

	vtkSmartPointer<vtkActor>& GetInterestVoxelActor();

	void PrepareInterestRegions(vtkAlgorithmOutput* voxelSourceGeometry);

protected:
	void PreparePointsForRendering() override;
	vtkSmartPointer<vtkPoints> initialPoints;

private:
	// ** colors **
	std::array<double, 4> negativeInterestVoxelColor;
	std::array<double, 4> positiveInterestVoxelColor;

	const char* interestColorArrayName = "interest_color";
	const char* interestScaleArrayName = "interest_scale";

	bool interestRegionHashesAreSet = false;
	bool preparePipelineWasCalled = false;
	std::set<int> interestRegionHashes;

	// ** individual voxels **
	vtkSmartPointer<vtkPolyData> interestVoxelPolydata;
	vtkSmartPointer<vtkLookupTable> interestVoxelColorLookupTable;
	vtkSmartPointer<vtkGlyph3DMapper> interestVoxelMapper;
	vtkSmartPointer<vtkActor> interestVoxelActor;

};


