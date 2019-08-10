//  ================================================================
//  Created by Gregory Kramida on 7/10/18.
//  Copyright (c) 2018-2025 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================
#include "ITMSceneFileIOEngine.h"

using namespace ITMLib;

const std::string compactFilePostfixAndExtension = "compact.dat";
//TODO: revise member functions & their usages to accept the full path as argument instead of the directory

template<typename TVoxel>
void ITMSceneFileIOEngine<TVoxel, ITMVoxelBlockHash>::SaveToDirectoryCompact(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene,
                                                                             const std::string& outputDirectory) {
	std::string path = outputDirectory + compactFilePostfixAndExtension;
	std::ofstream ofStream = std::ofstream(path.c_str(),std::ios_base::binary | std::ios_base::out);
	if (!ofStream) throw std::runtime_error("Could not open '" + path + "' for writing.");

	const TVoxel* voxels = scene->localVBA.GetVoxelBlocks();
	const ITMHashEntry* hashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noTotalEntries;

	int lastExcessListId = scene->index.GetLastFreeExcessListId();
	ofStream.write(reinterpret_cast<const char* >(&scene->localVBA.lastFreeBlockId), sizeof(int));
	ofStream.write(reinterpret_cast<const char* >(&scene->localVBA.allocatedSize), sizeof(int));
	ofStream.write(reinterpret_cast<const char* >(&lastExcessListId), sizeof(int));
	//count filled entries
	int allocatedHashBlockCount = 0;
#ifdef WITH_OPENMP
#pragma omp parallel for reduction(+:allocatedHashBlockCount)
#endif

	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		const ITMHashEntry& currentHashEntry = hashTable[entryId];
		//skip unfilled hash
		if (currentHashEntry.ptr < 0) continue;
		allocatedHashBlockCount++;
	}

	ofStream.write(reinterpret_cast<const char* >(&allocatedHashBlockCount), sizeof(int));
	for (int entryId = 0; entryId < noTotalEntries; entryId++) {
		const ITMHashEntry& currentHashEntry = hashTable[entryId];
		//skip unfilled hash
		if (currentHashEntry.ptr < 0) continue;
		ofStream.write(reinterpret_cast<const char* >(&entryId), sizeof(int));
		ofStream.write(reinterpret_cast<const char* >(&currentHashEntry), sizeof(ITMHashEntry));
		const TVoxel* localVoxelBlock = &(voxels[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		ofStream.write(reinterpret_cast<const char* >(localVoxelBlock), sizeof(TVoxel)*SDF_BLOCK_SIZE3);
	}
}

template<typename TVoxel>
void
ITMSceneFileIOEngine<TVoxel, ITMVoxelBlockHash>::LoadFromDirectoryCompact(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash>* scene,
                                                                          const std::string& outputDirectory) {
	std::string path = outputDirectory + "compact.dat";
	std::ifstream ifStream = std::ifstream(path.c_str(),std::ios_base::binary | std::ios_base::in);
	if (!ifStream) throw std::runtime_error("Could not open '" + path + "' for reading.");

	TVoxel* voxelBlocks = scene->localVBA.GetVoxelBlocks();
	ITMHashEntry* hashTable = scene->index.GetEntries();
	int noTotalEntries = scene->index.noTotalEntries;
	int lastExcessListId;
	int lastOrderedListId;
	ifStream.read(reinterpret_cast<char* >(&lastOrderedListId), sizeof(int));
	ifStream.read(reinterpret_cast<char* >(&scene->localVBA.allocatedSize), sizeof(int));
	ifStream.read(reinterpret_cast<char* >(&lastExcessListId), sizeof(int));
	scene->index.SetLastFreeExcessListId(lastExcessListId);
	scene->localVBA.lastFreeBlockId = lastOrderedListId;
	//count filled entries
	int allocatedHashBlockCount;
	ifStream.read(reinterpret_cast<char* >(&allocatedHashBlockCount), sizeof(int));
	for (int iEntry = 0; iEntry < allocatedHashBlockCount; iEntry++) {
		int entryId;
		ifStream.read(reinterpret_cast<char* >(&entryId), sizeof(int));
		ITMHashEntry& currentHashEntry = hashTable[entryId];
		ifStream.read(reinterpret_cast<char* >(&currentHashEntry), sizeof(ITMHashEntry));
		TVoxel* localVoxelBlock = &(voxelBlocks[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
		ifStream.read(reinterpret_cast<char* >(localVoxelBlock), sizeof(TVoxel)*SDF_BLOCK_SIZE3);
	}
}


template<typename TVoxel>
void
ITMSceneFileIOEngine<TVoxel, ITMPlainVoxelArray>::SaveToDirectoryCompact(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene,
                                                                         const std::string& outputDirectory) {
	scene->SaveToDirectory(outputDirectory);
}


template<typename TVoxel>
void
ITMSceneFileIOEngine<TVoxel, ITMPlainVoxelArray>::LoadFromDirectoryCompact(ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* scene,
                                                                           const std::string& outputDirectory) {
	scene->LoadFromDirectory(outputDirectory);
}