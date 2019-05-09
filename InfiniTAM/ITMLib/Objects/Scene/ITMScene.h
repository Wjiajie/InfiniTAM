// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMLocalVBA.h"
#include "ITMGlobalCache.h"
#include "../../Utils/ITMSceneParams.h"

namespace ITMLib
{
/** \brief
Represents the 3D world model as a hash of small voxel
blocks
*/
template<class TVoxel, class TIndex>
class ITMScene
{
public:
	/** Scene parameters like voxel size etc. */
	const ITMSceneParams *sceneParams;

	/** Hash table to reference the 8x8x8 blocks */
	TIndex index;

	/** Current local content of the 8x8x8 voxel blocks -- stored host or device */
	ITMLocalVBA<TVoxel> localVBA;

	/** Global content of the 8x8x8 voxel blocks -- stored on host only */
	ITMGlobalCache<TVoxel> *globalCache;

	void SaveToDirectory(const std::string &outputDirectory) const
	{
		localVBA.SaveToDirectory(outputDirectory);
		index.SaveToDirectory(outputDirectory);
	}

	void LoadFromDirectory(const std::string &outputDirectory)
	{
		localVBA.LoadFromDirectory(outputDirectory);
		index.LoadFromDirectory(outputDirectory);
	}

	ITMScene(const ITMSceneParams *_sceneParams, bool _useSwapping, MemoryDeviceType _memoryType,
	         Vector3i size = Vector3i(512), Vector3i offset = Vector3i(-256,-256,0));

	~ITMScene(void)
	{
		if (globalCache != nullptr) delete globalCache;
	}

	/** Return whether this scene is using swapping mechanism or not. **/
	bool Swapping() const{
		return this->globalCache != nullptr;
	}

	// Suppress the default copy constructor and assignment operator (C++11 way)
	ITMScene(const ITMScene&) = delete;
	ITMScene& operator=(const ITMScene&) = delete;
};
}//end namespace ITMLib