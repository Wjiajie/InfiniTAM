// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMView.h"
#include "../Misc/IMUMeasurement.h"

namespace ITMLib
{
	/** \brief
	    Represents a single "view", i.e. RGB and depth images along
	    with all intrinsic and relative calibration information
	*/
	class ITMViewIMU : public ITMView
	{
	public:
		IMUMeasurement *imu;

		ITMViewIMU(const RGBDCalib& calibration, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU)
		 : ITMView(calibration, imgSize_rgb, imgSize_d, useGPU)
		{
			imu = new IMUMeasurement();
		}

		~ITMViewIMU(void) { delete imu; }

		// Suppress the default copy constructor and assignment operator
		ITMViewIMU(const ITMViewIMU&);
		ITMViewIMU& operator=(const ITMViewIMU&);
	};
}
