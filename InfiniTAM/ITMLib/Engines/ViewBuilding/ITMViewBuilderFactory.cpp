// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMViewBuilderFactory.h"

#include "CPU/ITMViewBuilder_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/ITMViewBuilder_CUDA.h"
#endif

namespace ITMLib
{

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

ITMViewBuilder *ITMViewBuilderFactory::MakeViewBuilder(const ITMRGBDCalib& calib, MemoryDeviceType deviceType)
{
  ITMViewBuilder *viewBuilder = nullptr;

  switch(deviceType)
  {
    case MEMORYDEVICE_CPU:
      viewBuilder = new ITMViewBuilder_CPU(calib);
      break;
    case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
      viewBuilder = new ITMViewBuilder_CUDA(calib);
#endif
      break;
    case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
      viewBuilder = new ITMViewBuilder_CPU(calib);
#endif
      break;
  }

  return viewBuilder;
}

ITMViewBuilder *ITMViewBuilderFactory::MakeViewBuilder(const std::string& calibration_path, MemoryDeviceType deviceType)
{
	ITMViewBuilder *viewBuilder = nullptr;

	ITMRGBDCalib calibrationData;
	readRGBDCalib(calibration_path.c_str(), calibrationData);

	switch(deviceType)
	{
		case MEMORYDEVICE_CPU:
			viewBuilder = new ITMViewBuilder_CPU(calibrationData);
			break;
		case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			viewBuilder = new ITMViewBuilder_CUDA(calibrationData);
#endif
			break;
		case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
			viewBuilder = new ITMViewBuilder_CPU(calibrationData);
#endif
			break;
	}

	return viewBuilder;
}

}
