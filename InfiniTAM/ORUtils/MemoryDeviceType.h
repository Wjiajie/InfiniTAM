// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

//TODO: instead of having a separate "DeviceType" in the ITMLibSettings and a "MemoryDeviceType" here, just have a single,
// unified enum for DeviceType here.

/**
 * \brief The values of this enumeration denote the different types of memory device on which code may be running.
 */
enum MemoryDeviceType
{
  MEMORYDEVICE_CPU,
  MEMORYDEVICE_CUDA
};
