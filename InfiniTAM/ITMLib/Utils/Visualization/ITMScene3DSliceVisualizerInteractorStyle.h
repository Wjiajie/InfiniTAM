//  ================================================================
//  Created by Gregory Kramida on 6/28/18.
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
#include <vtkInteractorStyleTrackballCamera.h>

namespace ITMLib{
template<typename TVoxelCanonical,typename TVoxelLive,typename TIndex>
class ITMScene3DSliceVisualizerInteractorStyle : public vtkInteractorStyleTrackballCamera {
public:
	static ITMScene3DSliceVisualizerInteractorStyle* New();
	vtkTypeMacro(ITMScene3DSliceVisualizerInteractorStyle , vtkInteractorStyleTrackballCamera);
	ITMScene3DSliceVisualizerInteractorStyle();
	void SetParent(ITMScene3DSliceVisualizer<TVoxelCanonical,TVoxelLive,TIndex>* parent);

	void OnKeyPress() override;
private:
	ITMScene3DSliceVisualizer<TVoxelCanonical,TVoxelLive,TIndex>* parent;

	typedef void (ITMScene3DSliceVisualizerInteractorStyle<TVoxelCanonical,TVoxelLive,TIndex>::*MFP) (void);

	void PrintA();
	std::map<std::string, MFP> createBindingMap();
	const std::map<std::string, MFP> bindingMap;

};
}


