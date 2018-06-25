//  ================================================================
//  Created by Gregory Kramida on 6/21/18.
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

#include "../../ITMLibDefines.h"
#include "ITMScene3DSliceVisualizer.h"

namespace ITMLib{
template <typename TIndex>
class ITMCanonicalScene3DSliceVisualizer :  public ITMScene3DSliceVisualizer<ITMVoxelCanonical, TIndex> {
public:
	ITMCanonicalScene3DSliceVisualizer(ITMScene<ITMVoxelCanonical, TIndex>* scene, const Vector3i& focusCoordinates,
	                                   Plane plane = PLANE_XY, int radiusInPlane = 10, int radiusOutOfPlane = 0);
	virtual ~ITMCanonicalScene3DSliceVisualizer() = default;
	void DrawWarpUpdates();
private:
	// ** warp updates
	vtkSmartPointer<vtkPolyData> updatesData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyDataMapper> updatesMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkActor> updatesActor = vtkSmartPointer<vtkActor>::New();


};

}//namespace ITMLib


