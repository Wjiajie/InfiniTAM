// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMLocalVBA.h"
#include "ITMGlobalCache.h"
#include "../../Utils/VoxelVolumeParameters.h"

namespace ITMLib
{
/** \brief
Represents the 3D world model as collection of voxel blocks, i.e. regular 3D grid
*/
template<class TVoxel, class TIndex>
class ITMVoxelVolume
{
public:
	/** Scene parameters like voxel size etc. */
	const VoxelVolumeParameters* sceneParams;

	/**
	 * \brief An indexing method for access to the volume's voxels.
	 * \details For instance, if VoxelBlockHash is used as TIndex, this is a hash table to reference the 8x8x8
	 * blocks. If it's an PlainVoxelArray, it's just a dense regular 3D array. */
	TIndex index;

	/** Current local content of the 8x8x8 voxel blocks -- stored host or device */
	ITMLocalVBA<TVoxel> localVBA;

	/** "Global" content -- stored on in host memory only */
	ITMGlobalCache<TVoxel, TIndex>* globalCache;

	ITMVoxelVolume(const VoxelVolumeParameters *_sceneParams, bool _useSwapping, MemoryDeviceType _memoryType,
	               typename TIndex::InitializationParameters indexParameters = typename TIndex::InitializationParameters());
	ITMVoxelVolume(MemoryDeviceType memoryDeviceType, typename TIndex::InitializationParameters indexParameters = typename TIndex::InitializationParameters());
	ITMVoxelVolume(const ITMVoxelVolume& other, MemoryDeviceType _memoryType);
	~ITMVoxelVolume()
	{
		if (globalCache != nullptr) delete globalCache;
	}

	void Reset();
	void SetFrom(const ITMVoxelVolume& other);
	void SaveToDirectory(const std::string &outputDirectory) const;
	void LoadFromDirectory(const std::string &outputDirectory);
	TVoxel GetValueAt(const Vector3i& pos);
	TVoxel GetValueAt(int x, int y, int z){
		Vector3i pos(x,y,z);
		return GetValueAt(pos);
	}

	/** Return whether this scene is using swapping mechanism or not. **/
	bool Swapping() const{
		return this->globalCache != nullptr;
	}

	// Suppress the default copy constructor and assignment operator (C++11 way)
	ITMVoxelVolume(const ITMVoxelVolume&) = delete;
	//ITMVoxelVolume(ITMVoxelVolume&&) noexcept = default;
	ITMVoxelVolume& operator=(const ITMVoxelVolume&) = delete;
};

}//end namespace ITMLib