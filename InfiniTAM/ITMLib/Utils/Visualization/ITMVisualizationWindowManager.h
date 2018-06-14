//  ================================================================
//  Created by Gregory Kramida on 6/7/18.
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

//stdlib
#include <unordered_map>

//vtk
#include <vtkRenderWindowInteractor.h>
#include <vtkContextView.h>
#include <vtkChartXY.h>


namespace ITMLib {

class ITMChartWindow {
public:
	ITMChartWindow(const std::string& name, const std::string& title, int width, int height);
	~ITMChartWindow();
	vtkSmartPointer<vtkChartXY> GetChart();
	vtkSmartPointer<vtkRenderWindow> GetRenderWindow();
	void Update();

private:
	std::string name;
	vtkSmartPointer<vtkRenderWindowInteractor> interactor;
	vtkSmartPointer<vtkContextView> view;
	vtkSmartPointer<vtkRenderWindow> renderWindow;
	vtkSmartPointer<vtkChartXY> chart;
};

class ITMVisualizationWindowManager {
public:
	static ITMVisualizationWindowManager& Instance() {
		static ITMVisualizationWindowManager instance;
		return instance;
	}

	ITMChartWindow* MakeOrGetWindow(const std::string& name,
	                                const std::string& title = "VTK Window",
	                                int width = 1024, int height = 768);
	ITMChartWindow* GetWindow(const std::string& name);

	ITMVisualizationWindowManager(ITMVisualizationWindowManager const&) = delete;
	void operator=(ITMVisualizationWindowManager const&)  = delete;
private:

	std::unordered_map<std::string, ITMChartWindow> windows;

	ITMVisualizationWindowManager() = default;
	~ITMVisualizationWindowManager() = default;
};
}//namespace ITMLib