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
		ORUtils::MemoryBlock<int> *visibleEntryIDs;

		/** A list of "visible entries", that are
		currently being processed by integration
		and tracker.
		*/
		ORUtils::MemoryBlock<uchar> *entriesVisibleType;
           
	public:
		/** Number of entries in the live list. */
		int noVisibleEntries;
		const int hashEntryCount;
           
		ITMRenderState_VH(const int hashEntryCount, const int voxelBlockCount, const Vector2i & imgSize, float vf_min, float vf_max, MemoryDeviceType memoryType = MEMORYDEVICE_CPU)
			: ITMRenderState(imgSize, vf_min, vf_max, memoryType), hashEntryCount(hashEntryCount)
		{
			this->memoryType = memoryType;

			visibleEntryIDs = new ORUtils::MemoryBlock<int>(voxelBlockCount, memoryType);
			entriesVisibleType = new ORUtils::MemoryBlock<uchar>(hashEntryCount, memoryType);

			noVisibleEntries = 0;
		}
		~ITMRenderState_VH()
		{
			delete visibleEntryIDs;
			delete entriesVisibleType;
		}
		/** Get the list of "visible entries", that are currently
		processed by the tracker.
		*/
		const int *GetVisibleEntryIDs(void) const { return visibleEntryIDs->GetData(memoryType); }
		int *GetVisibleEntryIDs(void) { return visibleEntryIDs->GetData(memoryType); }

		/** Get the list of "visible entries", that are
		currently processed by integration and tracker.
		*/
		uchar *GetEntriesVisibleType(void) { return entriesVisibleType->GetData(memoryType); }

#ifdef COMPILE_WITH_METAL
		const void* GetVisibleEntryIDs_MB(void) { return visibleEntryIDs->GetMetalBuffer(); }
		const void* GetEntriesVisibleType_MB(void) { return entriesVisibleType->GetMetalBuffer(); }
#endif
	};
} 
