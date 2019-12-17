// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdlib.h>

#include "ITMRenderState.h"
#include "../Scene/ITMVoxelBlockHash.h"
#include "../../../ORUtils/MemoryBlock.h"

namespace ITMLib
{
	/** \brief
	    Stores the render state used by the SceneReconstruction 
	    and visualisation engines, as used by voxel hashing.
	*/
	class ITMRenderState_VH : public ITMRenderState
	{
	private:
		MemoryDeviceType memoryType;

		/** A list of "visible entries", that are currently
		being processed by the tracker.
		*/
		ORUtils::MemoryBlock<int> *visibleBlockHashCodes;

		/** A list of "visible entries", that are
		currently being processed by integration
		and tracker.
		*/
		ORUtils::MemoryBlock<HashBlockVisibility> *blockVisibilityTypes;
           
	public:
		/** Number of entries in the live list. */
		int visibleHashBlockCount;
		const int hashEntryCount;
           
		ITMRenderState_VH(const int hashEntryCount, const int voxelBlockCount, const Vector2i & imgSize, float vf_min, float vf_max, MemoryDeviceType memoryType = MEMORYDEVICE_CPU)
			: ITMRenderState(imgSize, vf_min, vf_max, memoryType), hashEntryCount(hashEntryCount)
		{
			this->memoryType = memoryType;

			visibleBlockHashCodes = new ORUtils::MemoryBlock<int>(voxelBlockCount, memoryType);
			blockVisibilityTypes = new ORUtils::MemoryBlock<HashBlockVisibility>(hashEntryCount, memoryType);

			visibleHashBlockCount = 0;
		}
		~ITMRenderState_VH()
		{
			delete visibleBlockHashCodes;
			delete blockVisibilityTypes;
		}
		/** Get the list of "visible entries", that are currently
		processed by the tracker.
		*/
		const int *GetVisibleBlockHashCodes(void) const { return visibleBlockHashCodes->GetData(memoryType); }
		int *GetVisibleBlockHashCodes(void) { return visibleBlockHashCodes->GetData(memoryType); }

		/** Get the list of "visible entries", that are
		currently processed by integration and tracker.
		*/
		HashBlockVisibility *GetBlockVisibilityTypes(void) { return blockVisibilityTypes->GetData(memoryType); }

#ifdef COMPILE_WITH_METAL
		const void* GetVisibleEntryIDs_MB(void) { return visibleBlockHashCodes->GetMetalBuffer(); }
		const void* GetEntriesVisibleType_MB(void) { return blockVisibilityTypes->GetMetalBuffer(); }
#endif
	};
} 
