// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ExtendedTracker.h"

namespace ITMLib
{
	//N.B. public virtual inheritance here to avoid the deadly-diamond problem in ITMKillingTracker_CPU
	class ExtendedTracker_CPU : public virtual ExtendedTracker
	{
	protected:
		int ComputeGandH_Depth(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);
		int ComputeGandH_RGB(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);
		void ProjectCurrentIntensityFrame(ITMFloat4Image *points_out,
										  ITMFloatImage *intensity_out,
										  const ITMFloatImage *intensity_in,
										  const ITMFloatImage *depth_in,
										  const Vector4f &intrinsics_depth,
										  const Vector4f &intrinsics_rgb,
										  const Matrix4f &scenePose);

	public:
		ExtendedTracker_CPU(Vector2i imgSize_d,
		                    Vector2i imgSize_rgb,
		                    bool useDepth,
		                    bool useColour,
		                    float colourWeight,
		                    TrackerIterationType *trackingRegime,
		                    int noHierarchyLevels,
		                    float terminationThreshold,
		                    float failureDetectorThreshold,
		                    float viewFrustum_min,
		                    float viewFrustum_max,
		                    float minColourGradient,
		                    float tukeyCutOff,
		                    int framesToSkip,
		                    int framesToWeight,
		                    const ITMLowLevelEngine *lowLevelEngine);
		~ExtendedTracker_CPU(void);
	};
}
