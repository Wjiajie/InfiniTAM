//  ================================================================
//  Created by Gregory Kramida on 1/3/18.
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


#include <vtk-8.1/vtkSmartPointer.h>
#include <vtk-8.1/vtkInteractorStyleTrackballCamera.h>
#include <vtk-8.1/vtkRenderWindowInteractor.h>
#include <vtk-8.1/vtkObjectFactory.h>
//local
#include "../../ITMLib/ITMLibDefines.h"
#include "../../ITMLib/Objects/Scene/ITMScene.h"

using namespace ITMLib;
namespace ITMLib {
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMSceneWarpFileIO;
}


class vtkRenderer;
class vtkRenderWindow;
class SDFViz;

/**
 * \brief A standard VTK trackball interator style with added functionality for
 * some keyboard keys
 */
class KeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera {


public:
	static KeyPressInteractorStyle* New();

vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleTrackballCamera);
	SDFViz* parent;

	virtual void OnKeyPress();

};

/**
 * \brief SDF Visualization application main class.
 */
class SDFViz {
	friend class KeyPressInteractorStyle;

private:

	//================= MEMBERS =================
	//data loader
	ITMSceneWarpFileIO <ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>* sceneLogger;

	//data structures
	ITMScene<ITMVoxelCanonical, ITMVoxelIndex>* canonicalScene;
	ITMScene<ITMVoxelLive, ITMVoxelIndex>* liveScene;

	//visualization setup
	// The renderer generates the image
	// which is then displayed on the render window.
	// It can be thought of as a scene to which the actor is added
	vtkSmartPointer<vtkRenderer> renderer;
	// The render window is the actual GUI window
	// that appears on the computer screen
	vtkSmartPointer<vtkRenderWindow> renderWindow;

	// The render window interactor captures mouse events
	// and will perform appropriate camera or actor manipulation
	// depending on the nature of the events.
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;

	//================ METHODS ====================
	void initializeRendering();

public:
	SDFViz();

	virtual ~SDFViz();

	int run();

};

