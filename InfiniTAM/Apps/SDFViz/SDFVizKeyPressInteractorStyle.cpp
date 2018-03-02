//  ================================================================
//  Created by Gregory Kramida on 3/2/18.
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
//VTK
#include <vtkObjectFactory.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>

//local
#include "SDFVizKeyPressInteractorStyle.h"
#include "SDFViz.h"

vtkStandardNewMacro(SDFVizKeyPressInteractorStyle);

void SDFVizKeyPressInteractorStyle::OnKeyPress() {

	// Get the keypress
	vtkRenderWindowInteractor* rwi = this->Interactor;
	std::string key = rwi->GetKeySym();


	if (parent != nullptr) {
		if (key == "c") {
			//record camera position
			double x, y, z;
			double xFocalPoint, yFocalPoint, zFocalPoint;
			double xUpVector, yUpVector, zUpVector;
			double dNear, dFar;
			parent->sdfRenderer->GetActiveCamera()->GetPosition(x, y, z);
			parent->sdfRenderer->GetActiveCamera()->GetFocalPoint(xFocalPoint, yFocalPoint, zFocalPoint);
			parent->sdfRenderer->GetActiveCamera()->GetViewUp(xUpVector, yUpVector, zUpVector);
			parent->sdfRenderer->GetActiveCamera()->GetClippingRange(dNear, dFar);
			std::cout << "Camera:" << std::endl;
			std::cout << "  Current position: " << x << ", " << y << ", " << z << std::endl;
			std::cout << "  Current focal point: " << xFocalPoint << ", " << yFocalPoint << ", " << zFocalPoint
			          << std::endl;
			std::cout << "  Current up-vector: " << xUpVector << ", " << yUpVector << ", " << zUpVector
			          << std::endl;
			std::cout << "  Current clipping range: " << dNear << ", " << dFar << std::endl;
			std::cout.flush();
		} else if (key == "v") {
			//toggle voxel blocks visibility
			if (rwi->GetAltKey()) {
				parent->ToggleCanonicalVoxelVisibility();
			} else {
				parent->ToggleLiveVoxelVisibility();
			}
		} else if (key == "h") {
			//toggle hash blocks visibility
			if (rwi->GetAltKey()) {
				parent->ToggleCanonicalHashBlockVisibility();
			} else {
				parent->ToggleLiveHashBlockVisibility();
			}
		} else if (key == "i") {
			//toggle interest region visibility
			parent->ToggleInterestVoxelVisibility();
		} else if (key == "period") {
			if (parent->hasWarpIterationInfo) {
				std::cout << "Loading next iteration warp & updates." << std::endl;
				if (parent->NextNonInterestWarps()) {
					std::cout << "Next warps loaded and display updated." << std::endl;
				} else {
					std::cout << "Could not load next iteration warp & updates." << std::endl;
				}
			}
		} else if (key == "comma") {
			if (parent->hasWarpIterationInfo) {
				std::cout << "Loading previous iteration warp & updates." << std::endl;
				if (parent->PreviousNonInterestWarps()) {
					std::cout << "Previous warps loaded and display updated." << std::endl;
				} else {
					std::cout << "Could not load previous iteration warp & updates." << std::endl;
				}
			}
		} else if (key == "minus" || key == "KP_Subtract") {
			if (rwi->GetAltKey()) {
				parent->DecreaseCanonicalVoxelOpacity();
			} else {
				parent->DecreaseLiveVoxelOpacity();
			}
		} else if (key == "equal" || key == "KP_Add") {
			if (rwi->GetAltKey()) {
				parent->IncreaseCanonicalVoxelOpacity();
			} else {
				parent->IncreaseLiveVoxelOpacity();
			}
		} else if (key == "bracketright") {
			if (parent->hasHighlightInfo) {
				parent->NextInterestWarps();
				std::cout << "Loading next interest voxel warps." << std::endl;
			}
		} else if (key == "bracketleft") {
			if (parent->hasHighlightInfo) {
				parent->PreviousInterestWarps();
				std::cout << "Loading previous interest voxel warps." << std::endl;
			}
		} else if (key == "Prior") {
			if (parent->hasHighlightInfo) {
				parent->MoveFocusToPreviousHighlight();
			} else {
				parent->sdfRenderer->GetActiveCamera()->SetViewUp(0.0, 1.0, 0.0);
				parent->renderWindow->Render();
			}
		} else if (key == "Next") {
			if (parent->hasHighlightInfo) {
				parent->MoveFocusToNextHighlight();
			} else {
				parent->sdfRenderer->GetActiveCamera()->SetViewUp(0.0, -1.0, 0.0);
				parent->renderWindow->Render();
			}
		} else if (key == "Escape") {
			rwi->TerminateApp();
			std::cout << "Exiting application..." << std::endl;
		} else if (key == "Home") {
			if (parent->hasHighlightInfo) {
				parent->RefocusAtCurrentHighlight();
			} else {
				parent->sdfRenderer->ResetCamera();
				parent->renderWindow->Render();
			}
		} else if (key == "b"){
			if(rwi->GetAltKey()){
				parent->PreviousBackgroundColor();
			}else{
				parent->NextBackgroundColor();
			}
		} else if (key == "t"){
			if(rwi->GetAltKey()){
				parent->canonicalScenePipe.ToggleScaleMode();
				parent->renderWindow->Render();
			}else{
				parent->liveScenePipe.ToggleScaleMode();
				parent->renderWindow->Render();
			}
		}
	}
	std::cout << "Key symbol: " << key << std::endl;
	std::cout << "Key code: " << static_cast<int>(rwi->GetKeyCode()) << std::endl;

	// Forward events
	vtkInteractorStyleTrackballCamera::OnKeyPress();

}