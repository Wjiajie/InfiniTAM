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
#include <thread>

//vtk
#include <vtkRenderWindowInteractor.h>
#include <vtkContextView.h>
#include <vtkChartXY.h>


namespace ITMLib {
class ITMVTKVisualizer {
public:
	static ITMVTKVisualizer& Instance() {
		static ITMVTKVisualizer instance;
		return instance;
	}

	bool Run();
	bool ShutDown();

	vtkSmartPointer<vtkChartXY>& GetChart();

	ITMVTKVisualizer(ITMVTKVisualizer const&) = delete;
	void operator=(ITMVTKVisualizer const&)  = delete;

	void Update();

private:
	void RunVTKView() {
		interactor->Start();
	}

	bool isRunning = false;
	std::unique_ptr<std::thread> thread;

	vtkSmartPointer<vtkRenderWindowInteractor> interactor = nullptr;
	vtkSmartPointer<vtkContextView> view = nullptr;
	vtkSmartPointer<vtkRenderWindow> renderWindow = nullptr;
	vtkSmartPointer<vtkChartXY> chart = nullptr;


	ITMVTKVisualizer();
	~ITMVTKVisualizer() = default;
};
}//namespace ITMLib