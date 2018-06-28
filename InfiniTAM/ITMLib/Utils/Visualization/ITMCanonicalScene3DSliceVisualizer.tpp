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
#include <vtkHedgeHog.h>
#include <vtkStructuredGrid.h>
#include "ITMCanonicalScene3DSliceVisualizer.h"
#include "../../Objects/Scene/ITMSceneTraversal.h"

using namespace ITMLib;


enum RequestedOperation{
	NONE,
	DRAW_WARP_UPDATES
};

namespace ITMLib{
template <typename TIndex>
class ThreadInteropCommand : public vtkCommand {
public:
vtkTypeMacro(ThreadInteropCommand, vtkCommand);
	RequestedOperation operation = NONE;
	static ThreadInteropCommand* New() {
		return new ThreadInteropCommand;
	}

	void Execute(vtkObject*vtkNotUsed(caller), unsigned long vtkNotUsed(eventId),
	             void*vtkNotUsed(callData)) {
		switch (operation){
			case NONE:

				break;
			case DRAW_WARP_UPDATES:
				operation = NONE;
				parent->DrawWarpUpdates();
				break;
			default:
				break;
		}

	}
	ITMCanonicalScene3DSliceVisualizer<TIndex>* parent;

};
}

template<typename TIndex>
ITMCanonicalScene3DSliceVisualizer<TIndex>::ITMCanonicalScene3DSliceVisualizer(
		ITMScene<ITMVoxelCanonical, TIndex>* scene, const Vector3i& focusCoordinates, Plane plane, int radiusInPlane,
		int radiusOutOfPlane)
		: ITMScene3DSliceVisualizer<ITMVoxelCanonical, TIndex>(scene, focusCoordinates, plane, radiusInPlane,
		                                                       radiusOutOfPlane),
		mutex(), conditionVariable(){
	this->thread = new std::thread(&ITMCanonicalScene3DSliceVisualizer<TIndex>::Run, this);
	std::unique_lock<std::mutex> lock(mutex);
	conditionVariable.wait(lock, [this]{return this->initialized;});

}



template<typename TVoxel>
struct TransferWarpUpdatesToVtkStructuresFunctor {
public:
	TransferWarpUpdatesToVtkStructuresFunctor(vtkPoints* points, vtkFloatArray* vectors) :
			points(points), vectors(vectors) {}

	void operator()(const TVoxel& voxel, const Vector3i& position) {
		Vector3f updateStartPoint = position.toFloat() + voxel.framewise_warp - voxel.warp_update;
		Vector3f updateVector = voxel.warp_update;
		points->InsertNextPoint(updateStartPoint.x, updateStartPoint.y, updateStartPoint.z);
		vectors->InsertNextTuple(updateVector.values);
	}

private:
	vtkPoints* points;
	vtkFloatArray* vectors;
};


template<typename TIndex>
void ITMCanonicalScene3DSliceVisualizer<TIndex>::DrawWarpUpdates() {

	std::unique_lock<std::mutex> lock(mutex);

	vtkPoints* updatePoints = this->updatesData->GetPoints();
	vtkFloatArray* updateVectors = vtkFloatArray::SafeDownCast(this->updatesData->GetPointData()->GetVectors());
	TransferWarpUpdatesToVtkStructuresFunctor<ITMVoxelCanonical> transferWarpUpdatesToVtkStructuresFunctor(
			updatePoints, updateVectors);
	VoxelPositionTraversalWithinBounds_CPU(this->scene, transferWarpUpdatesToVtkStructuresFunctor, this->bounds);

	this->updatesData->Modified();
	this->window->Update();

	this->warpUpdatePerformed = true;
	lock.unlock();
	conditionVariable.notify_all();
}

template<typename TIndex>
void ITMCanonicalScene3DSliceVisualizer<TIndex>::Run() {

	std::unique_lock<std::mutex> lock(mutex);
	this->Initialize();
	this->InitializeWarps();
	initialized = true;
	lock.unlock();
	conditionVariable.notify_all();
	this->window->RunInteractor();
}

template<typename TIndex>
void ITMCanonicalScene3DSliceVisualizer<TIndex>::InitializeWarps() {
	vtkSmartPointer<vtkPoints> updatePoints = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkFloatArray> updateVectors = vtkSmartPointer<vtkFloatArray>::New();
	updateVectors->SetName("Warp update vectors");
	updateVectors->SetNumberOfComponents(3);

	this->updatesData->SetPoints(updatePoints);
	this->updatesData->GetPointData()->SetVectors(updateVectors);

	vtkSmartPointer<vtkHedgeHog> hedgehog = vtkSmartPointer<vtkHedgeHog>::New();
	hedgehog->SetInputData(this->updatesData);
	updatesMapper->SetInputConnection(hedgehog->GetOutputPort());

	updatesActor->SetMapper(updatesMapper);
	updatesActor->GetProperty()->SetColor(0,0,0);

	this->window->AddLayer(Vector4d(1.0,1.0,1.0,0.0));
	this->window->AddActorToLayer(updatesActor, 1);

	threadCallback = vtkSmartPointer<ThreadInteropCommand<TIndex>>::New();
	threadCallback->parent = this;
	this->window->AddLoopCallback(threadCallback);
}

template<typename TIndex>
void ITMCanonicalScene3DSliceVisualizer<TIndex>::TriggerDrawWarpUpdates() {
	this->threadCallback->operation = DRAW_WARP_UPDATES;
	std::unique_lock<std::mutex> lock(mutex);
	conditionVariable.wait(lock, [this]{return this->warpUpdatePerformed;});
	this->warpUpdatePerformed = false;
}
