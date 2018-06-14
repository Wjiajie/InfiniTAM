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
#include "ITMVisualizationWindowManager.h"

using namespace ITMLib;

ITMChartWindow::ITMChartWindow(const std::string& name, const std::string& title, int width, int height) :
		view(vtkSmartPointer<vtkContextView>::New()),
		chart(vtkSmartPointer<vtkChartXY>::New()),
		name(name){
	renderWindow = view->GetRenderWindow();
	renderWindow->SetWindowName(title.c_str());
	renderWindow->SetSize(width, height);
	view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
	view->GetScene()->AddItem(chart);
	interactor = view->GetInteractor();
	interactor->Initialize();
}


vtkSmartPointer<vtkChartXY> ITMChartWindow::GetChart() {
	return chart;
}

void ITMChartWindow::Update() {
	renderWindow->Render();
}

ITMChartWindow::~ITMChartWindow() {
	interactor->TerminateApp();
}

vtkSmartPointer<vtkRenderWindow> ITMChartWindow::GetRenderWindow() {
	return this->renderWindow;
}

ITMChartWindow*
ITMVisualizationWindowManager::MakeOrGetWindow(const std::string& name, const std::string& title, int width, int height) {
	auto it = windows.find(name);
	if(it == windows.end()){
		windows.emplace(name, ITMChartWindow(name, title, width, height));
		it = windows.find(name);
	}
	return it == windows.end() ? nullptr : &(windows.find(name)->second);
}

ITMChartWindow* ITMVisualizationWindowManager::GetWindow(const std::string& name) {
	auto it = windows.find(name);
	return it == windows.end() ? nullptr : &(windows.find(name)->second);
}
