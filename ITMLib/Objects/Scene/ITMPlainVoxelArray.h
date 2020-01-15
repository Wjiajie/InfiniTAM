// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__

#include "../../Utils/ITMMath.h"
#include "../../Utils/Serialization/Serialization.h"
#include "../../../ORUtils/MemoryBlock.h"

namespace ITMLib {
/** \brief
This is the central class for the original fixed size volume
representation. It contains the data needed on the CPU and
a pointer to the data structure on the GPU.
*/
class ITMPlainVoxelArray {
public:

	GENERATE_SERIALIZABLE_STRUCT(
			GridAlignedBox,
			(Vector3i, size, Vector3i(512), VECTOR, "Size, in voxels."),
			(Vector3i, offset, Vector3i(-256, -256, 0), VECTOR, "Offset of the lower left front corner of the volume, in voxels")
	);

	typedef GridAlignedBox IndexData;
	typedef GridAlignedBox InitializationParameters;
	struct IndexCache {
	};

private:
	ORUtils::MemoryBlock<IndexData>* indexData;

public:
	const MemoryDeviceType memoryType;

	ITMPlainVoxelArray(ITMPlainVoxelArray::InitializationParameters info, MemoryDeviceType memoryType) :
			memoryType(memoryType),
			indexData(new ORUtils::MemoryBlock<IndexData>(1, true, true)) {
		indexData->GetData(MEMORYDEVICE_CPU)[0] = info;
		indexData->UpdateDeviceFromHost();
	}

	explicit ITMPlainVoxelArray(MemoryDeviceType memoryType, Vector3i size = Vector3i(512),
	                            Vector3i offset = Vector3i(-256, -256, 0)) : ITMPlainVoxelArray({size, offset},
	                                                                                            memoryType) {}

	ITMPlainVoxelArray(const ITMPlainVoxelArray& other, MemoryDeviceType memoryType) :
			ITMPlainVoxelArray({other.GetVolumeSize(), other.GetVolumeOffset()}, memoryType) {
		this->SetFrom(other);
	}

	void SetFrom(const ITMPlainVoxelArray& other) {
		MemoryCopyDirection memoryCopyDirection = determineMemoryCopyDirection(this->memoryType, other.memoryType);
		this->indexData->SetFrom(other.indexData, memoryCopyDirection);
	}

	~ITMPlainVoxelArray() {
		delete indexData;
	}

	/** Maximum number of total entries. */
	int GetAllocatedBlockCount() { return 1; }

	int GetVoxelBlockSize() {
		return indexData->GetData(MEMORYDEVICE_CPU)->size.x *
		       indexData->GetData(MEMORYDEVICE_CPU)->size.y *
		       indexData->GetData(MEMORYDEVICE_CPU)->size.z;
	}

	unsigned int GetMaxVoxelCount() const {
		return static_cast<unsigned int>(indexData->GetData(MEMORYDEVICE_CPU)->size.x) *
		       static_cast<unsigned int>(indexData->GetData(MEMORYDEVICE_CPU)->size.y) *
		       static_cast<unsigned int>(indexData->GetData(MEMORYDEVICE_CPU)->size.z);
	}

	Vector3i GetVolumeSize() const { return indexData->GetData(MEMORYDEVICE_CPU)->size; }

	Vector3i GetVolumeOffset() const { return indexData->GetData(MEMORYDEVICE_CPU)->offset; }

	/**Get the memory type used for storage.**/
	MemoryDeviceType GetMemoryType() const {
		return this->memoryType;
	}

	const IndexData* GetIndexData() const { return indexData->GetData(memoryType); }

	IndexData* GetIndexData() { return indexData->GetData(memoryType); }

	void SaveToDirectory(const std::string& outputDirectory) const {
	}

#pragma clang diagnostic push
#pragma ide diagnostic ignored "MemberFunctionCanBeStatic"

	void LoadFromDirectory(const std::string& outputDirectory) {
	}

#pragma clang diagnostic pop

#ifdef COMPILE_WITH_METAL
	const void *getIndexData_MB() const { return indexData->GetMetalBuffer(); }
#endif

	// Suppress the default copy constructor and assignment operator
	ITMPlainVoxelArray(const ITMPlainVoxelArray&) = delete;
	ITMPlainVoxelArray& operator=(const ITMPlainVoxelArray&) = delete;
};
}

#endif