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

//stdlib
#include <memory>

//VTK
#include <vtkContextScene.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>

//local
#include "ITMVTKVisualizer.h"

using namespace ITMLib;

ITMVTKVisualizer::ITMVTKVisualizer() :
		view(vtkSmartPointer<vtkContextView>::New()),
		chart(vtkSmartPointer<vtkChartXY>::New()) {
	renderWindow = view->GetRenderWindow();
	renderWindow->SetSize(1024, 768);
	view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
	view->GetScene()->AddItem(chart);
	interactor = view->GetInteractor();
	interactor->Initialize();
}

bool ITMVTKVisualizer::Run() {
	if (isRunning) return false;
	isRunning = true;
	thread = std::unique_ptr<std::thread>(new std::thread(&ITMVTKVisualizer::RunVTKView, this));
	return true;
}

bool ITMVTKVisualizer::ShutDown() {
	if (!isRunning) return false;
	interactor->TerminateApp();
	thread->join();
	isRunning = false;
}

vtkSmartPointer<vtkChartXY>& ITMVTKVisualizer::GetChart() {
	return chart;
}

void ITMVTKVisualizer::Update() {
	renderWindow->Render();
}
