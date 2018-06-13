//  ================================================================
//  Created by Gregory Kramida on 6/12/18.
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

#include "ITMVisualizationWindowManager.h"
#include "../ITMMath.h"

namespace ITMLib {

class ITMSceneTrackingEnergyPlotter {
public:
	static ITMSceneTrackingEnergyPlotter& Instance() {
		static ITMSceneTrackingEnergyPlotter instance;
		return instance;
	}

	void AddDataPoints(float dataEnergy, float smoothingEnergy, float levelSetEnergy = 0.0f, float killingEnergy = 0.0f);
	void ClearChart();
	void SaveScreenshot(std::string path);


	ITMSceneTrackingEnergyPlotter(ITMSceneTrackingEnergyPlotter const&) = delete;
	void operator=(ITMSceneTrackingEnergyPlotter const&) = delete;
private:

	void PreparePlot(Vector3i colorDataEnergy = Vector3i(0, 0, 0),
	                 Vector3i colorSmoothingEnergy = Vector3i(0, 0, 0),
	                 Vector3i colorLevelSetEnergy = Vector3i(0, 0, 0),
	                 Vector3i colorKillingEnergy = Vector3i(0, 0, 0));
	ITMSceneTrackingEnergyPlotter();

	vtkSmartPointer<vtkFloatArray> dataEnergyValues;
	vtkSmartPointer<vtkFloatArray> smoothingEnergyValues;
	vtkSmartPointer<vtkFloatArray> levelSetEnergyValues;
	vtkSmartPointer<vtkFloatArray> killingEnergyValues;
	vtkSmartPointer<vtkTable> table;

	ITMChartWindow* window;

};

}//namespace ITMLib


