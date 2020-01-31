// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/LowLevelEngine.h"

namespace ITMLib
{
	class LowLevelEngine_CPU : public LowLevelEngine
	{
	public:
		void CopyImage(ITMUChar4Image *image_out, const ITMUChar4Image *image_in) const;
		void CopyImage(ITMFloatImage *image_out, const ITMFloatImage *image_in) const;
		void CopyImage(ITMFloat4Image *image_out, const ITMFloat4Image *image_in) const;

		void ConvertColourToIntensity(ITMFloatImage *image_out, const ITMUChar4Image *image_in) const;

		void FilterIntensity(ITMFloatImage *image_out, const ITMFloatImage *image_in) const;

		void FilterSubsample(ITMUChar4Image *image_out, const ITMUChar4Image *image_in) const;
		void FilterSubsample(ITMFloatImage *image_out, const ITMFloatImage *image_in) const;
		void FilterSubsampleWithHoles(ITMFloatImage *image_out, const ITMFloatImage *image_in) const;
		void FilterSubsampleWithHoles(ITMFloat4Image *image_out, const ITMFloat4Image *image_in) const;

		void GradientX(ITMShort4Image *grad_out, const ITMUChar4Image *image_in) const;
		void GradientY(ITMShort4Image *grad_out, const ITMUChar4Image *image_in) const;
		void GradientXY(ITMFloat2Image *grad_out, const ITMFloatImage *image_in) const;

		int CountValidDepths(const ITMFloatImage *image_in) const;

		LowLevelEngine_CPU(void);
		~LowLevelEngine_CPU(void);
	};
}
