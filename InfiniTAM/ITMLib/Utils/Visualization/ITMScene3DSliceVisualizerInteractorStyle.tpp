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
//VTK
#include <vtkObjectFactory.h>
#include <map>

//local
#include "ITMScene3DSliceVisualizerInteractorStyle.h"

using namespace ITMLib;

// Can't use vtkStandardNewMacro on a templated class.
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMScene3DSliceVisualizerInteractorStyle<TVoxelCanonical, TVoxelLive, TIndex>*
ITMScene3DSliceVisualizerInteractorStyle<TVoxelCanonical, TVoxelLive, TIndex>::New() {
	ITMScene3DSliceVisualizerInteractorStyle<TVoxelCanonical, TVoxelLive, TIndex>* result = new ITMScene3DSliceVisualizerInteractorStyle<TVoxelCanonical, TVoxelLive, TIndex>();
	result->InitializeObjectBase();
	return result;
};


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMScene3DSliceVisualizerInteractorStyle<TVoxelCanonical, TVoxelLive, TIndex>::
        ITMScene3DSliceVisualizerInteractorStyle() : bindingMap(createBindingMap()) {

}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizerInteractorStyle<TVoxelCanonical, TVoxelLive, TIndex>::OnKeyPress() {
	// Get the keypress
	vtkRenderWindowInteractor* rwi = this->Interactor;
	std::string key = rwi->GetKeySym();
	bool shiftPressed = static_cast<bool>(rwi->GetShiftKey());
	bool altPressed = static_cast<bool>(rwi->GetAltKey());
	bool ctrlPressed = static_cast<bool>(rwi->GetControlKey());
	std::string hashKey = std::string((ctrlPressed ? "Ctrl+" : "")) + (altPressed ? "Alt+" : "") +
			(shiftPressed ? "Shift+" : "") +key;

	auto item = this->bindingMap.find(hashKey);
	if(item != this->bindingMap.end()){
		(this->*(item->second))();
	}

	//forward events
	vtkInteractorStyle::OnKeyPress();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizerInteractorStyle<TVoxelCanonical, TVoxelLive, TIndex>::SetParent(
		ITMScene3DSliceVisualizer<TVoxelCanonical, TVoxelLive, TIndex>* parent) {
	this->parent = parent;
	this->parent->window->AddLayer(Vector4d(0.0));

}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
std::map<std::string, void (ITMLib::ITMScene3DSliceVisualizerInteractorStyle<TVoxelCanonical, TVoxelLive, TIndex>::*)()>
ITMScene3DSliceVisualizerInteractorStyle<TVoxelCanonical, TVoxelLive, TIndex>::createBindingMap() {
	std::map<std::string, ITMScene3DSliceVisualizerInteractorStyle<TVoxelCanonical, TVoxelLive, TIndex>::MFP> map;
	// CREATE NEW KEY BINDINGS HERE!
	// Example keys: "Ctrl+a", "Alt+b", "Ctrl+Alt+k", "Alt+Shift+b", "Ctrl+Shift+c", "Ctrl+Alt+Shift+g"
	map.insert(std::make_pair("a", &ITMScene3DSliceVisualizerInteractorStyle<TVoxelCanonical, TVoxelLive, TIndex>::PrintA));
	return map;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMScene3DSliceVisualizerInteractorStyle<TVoxelCanonical, TVoxelLive, TIndex>::PrintA() {
	debug_print("MIAU!");
}
