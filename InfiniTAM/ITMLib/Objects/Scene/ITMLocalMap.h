// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <map>

#include "../../Engines/Visualisation/Interface/ITMVisualisationEngine.h"
#include "../../Objects/RenderStates/ITMRenderState.h"
#include "ITMVoxelVolume.h"
#include "../../Objects/Tracking/ITMTrackingState.h"
#include "../../Utils/Configuration.h"

namespace ITMLib {
	struct ITMPoseConstraint
	{
	public:
		ITMPoseConstraint(void)
		{
			accu_num = 0;
		}

		void AddObservation(const ORUtils::SE3Pose & relative_pose, int weight = 1)
		{
			Matrix4f tmp = accu_poses.GetM() * (float)accu_num + relative_pose.GetM() * (float)weight;
			accu_num += weight;
			accu_poses.SetM(tmp / (float)accu_num);
			accu_poses.Coerce();
			//	accu_poses = (accu_poses * (float)accu_num + relative_pose)/(float)(accu_num+1);
			accu_num++;
		}

		ORUtils::SE3Pose GetAccumulatedObservations(void) const { return accu_poses; }
		int GetNumAccumulatedObservations(void) const { return accu_num; }

	private:
		ORUtils::SE3Pose accu_poses;
		int accu_num;
	};

	typedef std::map<int, ITMPoseConstraint> ConstraintList;

	template<class TVoxel, class TIndex>
	class ITMLocalMap
	{
	public:
		ITMVoxelVolume<TVoxel, TIndex> *scene;
		ITMRenderState *renderState;
		ITMTrackingState *trackingState;
		ConstraintList relations;
		ORUtils::SE3Pose estimatedGlobalPose;

		ITMLocalMap(const ITMVisualisationEngine<TVoxel, TIndex>* visualisationEngine, const Vector2i& trackedImageSize)
		{
			auto& settings = Configuration::Instance();
			MemoryDeviceType memoryType = settings.deviceType == MEMORYDEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
			scene = new ITMVoxelVolume<TVoxel, TIndex>(&settings.scene_parameters, settings.swappingMode == Configuration::SWAPPINGMODE_ENABLED, memoryType);
			renderState = visualisationEngine->CreateRenderState(scene, trackedImageSize);
			trackingState = new ITMTrackingState(trackedImageSize, memoryType);
		}
		~ITMLocalMap()
		{
			delete scene;
			delete renderState;
			delete trackingState;
		}
	};
}

