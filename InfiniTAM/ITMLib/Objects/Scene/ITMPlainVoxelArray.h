// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__

#include "../../Utils/ITMMath.h"
#include "../../../ORUtils/MemoryBlock.h"

namespace ITMLib
{
/** \brief
This is the central class for the original fixed size volume
representation. It contains the data needed on the CPU and
a pointer to the data structure on the GPU.
*/
class ITMPlainVoxelArray
{
public:
	struct ITMVoxelArrayInfo {
		/// Size in voxels
		Vector3i size;
		/// offset of the lower left front corner of the volume in voxels
		Vector3i offset;

		ITMVoxelArrayInfo()
		{
			size.x = size.y = size.z = 512;
			offset.x = -256;
			offset.y = -256;
			offset.z = 0;
		}

		ITMVoxelArrayInfo(Vector3i size, Vector3i offset) : size(size), offset(offset){}
	};

	typedef ITMVoxelArrayInfo IndexData;
	struct IndexCache {};

private:
	ORUtils::MemoryBlock<IndexData> *indexData;

	MemoryDeviceType memoryType;

	void Initialize(MemoryDeviceType memoryType, Vector3i size = Vector3i(512), Vector3i offset = Vector3i(-256,-256,0)){
		this->memoryType = memoryType;

		if (memoryType == MEMORYDEVICE_CUDA) indexData = new ORUtils::MemoryBlock<IndexData>(1, true, true);
		else indexData = new ORUtils::MemoryBlock<IndexData>(1, true, false);

		indexData->GetData(MEMORYDEVICE_CPU)[0] = IndexData(size,offset);
		indexData->UpdateDeviceFromHost();
	}

public:
	ITMPlainVoxelArray(MemoryDeviceType memoryType)
	{
		Initialize(memoryType);
	}

	ITMPlainVoxelArray(MemoryDeviceType memoryType, Vector3i size, Vector3i offset)
	{
		Initialize(memoryType, size, offset);
	}

	ITMPlainVoxelArray(const ITMPlainVoxelArray& other, MemoryDeviceType memoryType){
		Initialize(memoryType, other.getVolumeSize(), other.getVolumeOffset());
		this->SetFrom(other);
	}

	void SetFrom(const ITMPlainVoxelArray& other){
		MemoryCopyDirection memoryCopyDirection = determineMemoryCopyDirection(this->memoryType, other.memoryType);
		this->indexData->SetFrom(other.indexData,memoryCopyDirection);
	}

	~ITMPlainVoxelArray()
	{
		delete indexData;
	}

	/** Maximum number of total entries. */
	int getNumAllocatedVoxelBlocks(void) { return 1; }
	int getVoxelBlockSize(void)
	{
		return indexData->GetData(MEMORYDEVICE_CPU)->size.x *
		       indexData->GetData(MEMORYDEVICE_CPU)->size.y *
		       indexData->GetData(MEMORYDEVICE_CPU)->size.z;
	}

	const Vector3i getVolumeSize(void) const { return indexData->GetData(MEMORYDEVICE_CPU)->size; }
	const Vector3i getVolumeOffset(void) const { return indexData->GetData(MEMORYDEVICE_CPU)->offset; }

	/**Get the memory type used for storage.**/
	MemoryDeviceType GetMemoryType() const{
		return this->memoryType;
	}

	const IndexData* getIndexData(void) const { return indexData->GetData(memoryType); }
	IndexData* getIndexData(void) { return indexData->GetData(memoryType); }

	void SaveToDirectory(const std::string &outputDirectory) const
	{
	}

	void LoadFromDirectory(const std::string &outputDirectory)
	{
	}

#ifdef COMPILE_WITH_METAL
	const void *getIndexData_MB() const { return indexData->GetMetalBuffer(); }
#endif

	// Suppress the default copy constructor and assignment operator
	ITMPlainVoxelArray(const ITMPlainVoxelArray&);
	ITMPlainVoxelArray& operator=(const ITMPlainVoxelArray&);
};
}

#endif