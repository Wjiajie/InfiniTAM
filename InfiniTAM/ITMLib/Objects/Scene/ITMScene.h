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

		//TODO: provide GPU versions of these functions and make the public functions agnostic (i.e. choose appropriate version based on GPU/CPU setting -Greg (GitHub: Algomorph)
		void SaveToDirectoryCompact_CPU(const std::string &outputDirectory) const
		{

			std::string path = outputDirectory + "compact.dat";
			std::ofstream ofStream = std::ofstream(path.c_str(),std::ios_base::binary | std::ios_base::out);
			if (!ofStream) throw std::runtime_error("Could not open '" + path + "' for writing.");

			const TVoxel* voxelBlocks = localVBA.GetVoxelBlocks();
			const ITMHashEntry* canonicalHashTable = index.GetEntries();
			int noTotalEntries = index.noTotalEntries;
			//count filled entries
			int allocatedHashBlockCount = 0;
#ifdef WITH_OPENMP
#pragma omp parallel for reduction(+:allocatedHashBlockCount)
#endif
			for (int entryId = 0; entryId < noTotalEntries; entryId++) {
				const ITMHashEntry& currentHashEntry = canonicalHashTable[entryId];
				//skip unfilled hash
				if (currentHashEntry.ptr < 0) continue;
				allocatedHashBlockCount++;
			}
			ofStream.write(reinterpret_cast<const char* >(&allocatedHashBlockCount), sizeof(int));
			for (int entryId = 0; entryId < noTotalEntries; entryId++) {
				const ITMHashEntry& currentHashEntry = canonicalHashTable[entryId];
				//skip unfilled hash
				if (currentHashEntry.ptr < 0) continue;
				ofStream.write(reinterpret_cast<const char* >(&entryId), sizeof(int));
				ofStream.write(reinterpret_cast<const char* >(&currentHashEntry), sizeof(ITMHashEntry));
				const TVoxel* localVoxelBlock = &(voxelBlocks[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
				ofStream.write(reinterpret_cast<const char* >(localVoxelBlock), sizeof(TVoxel)*SDF_BLOCK_SIZE3);
			}
		}

		void LoadFromDirectoryCompact_CPU(const std::string &outputDirectory)
		{
			std::string path = outputDirectory + "compact.dat";
			std::ifstream ifStream = std::ifstream(path.c_str(),std::ios_base::binary | std::ios_base::in);
			if (!ifStream) throw std::runtime_error("Could not open '" + path + "' for reading.");

			TVoxel* voxelBlocks = localVBA.GetVoxelBlocks();
			ITMHashEntry* canonicalHashTable = index.GetEntries();
			int noTotalEntries = index.noTotalEntries;
			//count filled entries
			int allocatedHashBlockCount;
			ifStream.read(reinterpret_cast<char* >(&allocatedHashBlockCount), sizeof(int));
			for (int iEntry = 0; iEntry < allocatedHashBlockCount; iEntry++) {
				int entryId;
				ifStream.read(reinterpret_cast<char* >(&entryId), sizeof(int));
				ITMHashEntry& currentHashEntry = canonicalHashTable[entryId];
				ifStream.read(reinterpret_cast<char* >(&currentHashEntry), sizeof(ITMHashEntry));
				TVoxel* localVoxelBlock = &(voxelBlocks[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
				ifStream.read(reinterpret_cast<char* >(localVoxelBlock), sizeof(TVoxel)*SDF_BLOCK_SIZE3);
			}
		}

		ITMScene(const ITMSceneParams *_sceneParams, bool _useSwapping, MemoryDeviceType _memoryType)
			: sceneParams(_sceneParams), index(_memoryType), localVBA(_memoryType, index.getNumAllocatedVoxelBlocks(), index.getVoxelBlockSize())
		{
			if (_useSwapping) globalCache = new ITMGlobalCache<TVoxel>();
			else globalCache = NULL;
		}

		~ITMScene(void)
		{
			if (globalCache != NULL) delete globalCache;
		}

		// Suppress the default copy constructor and assignment operator
		ITMScene(const ITMScene&);
		ITMScene& operator=(const ITMScene&);
	};
}
