//  ================================================================
//  Created by Gregory Kramida on 1/16/18.
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
#include "ITMIntArrayMap3D.h"

//stdlib
#include <iostream>
#include <fstream>

using namespace ITMLib;

bool ITMIntArrayMap3D::InsertOrdered(int keyLevel3, int keyLevel2, int keyLevel1, int valueLevel0) {

	if (this->internalMap.find(keyLevel3) == (this->internalMap).end()) {
		internalMap[keyLevel3] = std::map<int, std::map<int, std::vector<int>>>();
	}
	auto& valueLevel3 = internalMap[keyLevel3];
	if (valueLevel3.find(keyLevel2) == valueLevel3.end()) {
		valueLevel3[keyLevel2] = std::map<int, std::vector<int>>();
	}
	auto& valueLevel2 = valueLevel3[keyLevel2];
	if (valueLevel2.find(keyLevel1) == valueLevel2.end()) {
		valueLevel2[keyLevel1] = std::vector<int>();
	}
	std::vector<int>& valueLevel1 = valueLevel2[keyLevel1];
	if (valueLevel1.empty() ||
	    valueLevel1[valueLevel1.size() - 1] != valueLevel0) {
		valueLevel1.push_back(valueLevel0);
		return true;
	}
	return false;
}

ITMIntArrayMap3D::ITMIntArrayMap3D(const char* prefixLevel3, const char* prefixLevel2, const char* prefixLevel1,
                                   const char* prefixLevel0) :
		internalMap(),
		prefixLevel3(prefixLevel3),
		prefixLevel2(prefixLevel2),
		prefixLevel1(prefixLevel1),
		prefixLevel0(prefixLevel0) {}

bool ITMIntArrayMap3D::SaveToFile(const char* path) {
	std::ofstream file = std::ofstream(path, std::ios::binary | std::ios::out);
	if (!file) {
		std::cerr << ("Could not open " + std::string(path) + " for writing") << std::endl;
		return false;
	}

	size_t sizeLevel3 = internalMap.size();
	file.write(reinterpret_cast<const char* >(&sizeLevel3), sizeof(size_t));
	for (std::pair<int, std::map<int, std::map<int, std::vector<int>>>> elementLevel3 : internalMap) {
		file.write(reinterpret_cast<const char* >(&elementLevel3.first), sizeof(int));
		size_t sizeLevel2 = elementLevel3.second.size();
		file.write(reinterpret_cast<const char* >(&sizeLevel2), sizeof(size_t));
		for (std::pair<int, std::map<int, std::vector<int>>> elementLevel2 : elementLevel3.second) {
			file.write(reinterpret_cast<const char* >(&elementLevel2.first), sizeof(int));
			size_t sizeLevel1 = elementLevel2.second.size();
			file.write(reinterpret_cast<const char* >(&sizeLevel1), sizeof(size_t));
			for (std::pair<int, std::vector<int>> elementLevel1 : elementLevel2.second) {
				file.write(reinterpret_cast<const char* >(&elementLevel1.first), sizeof(int));
				size_t sizeLevel0 = elementLevel1.second.size();
				file.write(reinterpret_cast<const char* >(&sizeLevel0), sizeof(size_t));
				for (int elementLevel0 : elementLevel1.second) {
					file.write(reinterpret_cast<const char* >(&elementLevel0), sizeof(int));
				}
			}
		}
	}
	file.close();
	return true;
}


bool ITMIntArrayMap3D::LoadFromFile(const char* path) {
	internalMap.clear();
	std::ifstream file = std::ifstream(path, std::ios::binary | std::ios::in);
	if (!file) {
		std::cerr << ("Could not open " + std::string(path) + " for writing") << std::endl;
		return false;
	}
	size_t level3ElementCount, level2ElementCount, level1ElementCount, level0ElementCount;
	int level3Element, level2Element, level1Element, level0Element;
	file.read(reinterpret_cast<char*>(&level3ElementCount), sizeof(size_t));
	for (int iLevel3Element = 0; iLevel3Element < level3ElementCount; iLevel3Element++) {
		file.read(reinterpret_cast<char*>(&level3Element), sizeof(int));
		internalMap[level3Element] = std::map<int, std::map<int, std::vector<int>>>();
		file.read(reinterpret_cast<char*>(&level2ElementCount), sizeof(size_t));
		for (int iLevel2Element = 0; iLevel2Element < level2ElementCount; iLevel2Element++) {
			file.read(reinterpret_cast<char*>(&level2Element), sizeof(int));
			internalMap[level3Element][level2Element] = std::map<int, std::vector<int>>();
			file.read(reinterpret_cast<char*>(&level1ElementCount), sizeof(size_t));
			for (int iLevel1Element = 0; iLevel1Element < level1ElementCount; iLevel1Element++) {
				file.read(reinterpret_cast<char*>(&level1Element), sizeof(int));
				internalMap[level3Element][level2Element][level1Element] = std::vector<int>();
				std::vector<int>& array = internalMap[level3Element][level2Element][level1Element];
				file.read(reinterpret_cast<char*>(&level0ElementCount), sizeof(size_t));
				for (int iLevel0Element = 0; iLevel0Element < level1ElementCount; iLevel0Element++) {
					file.read(reinterpret_cast<char*>(&level0Element), sizeof(int));
					array.push_back(level0Element);
				}
			}
		}
	}
	file.close();
	return true;
}

namespace ITMLib {
std::ostream& operator<<(std::ostream& stream, const ITMIntArrayMap3D& intArrayMap3D) {
	for (std::pair<int, std::map<int, std::map<int, std::vector<int>>>> elementLevel3 : intArrayMap3D.internalMap) {
		stream << intArrayMap3D.prefixLevel3 << ": " << elementLevel3.first << std::endl;
		for (std::pair<int, std::map<int, std::vector<int>>> elementLevel2 : elementLevel3.second) {
			stream << "  " << intArrayMap3D.prefixLevel2 << ": " << elementLevel2.first << std::endl;
			for (std::pair<int, std::vector<int>> elementLevel1 : elementLevel2.second) {
				stream << "    " << intArrayMap3D.prefixLevel1 << ": " << elementLevel1.first << std::endl;
				for (int elementLevel0 : elementLevel1.second) {
					stream << "      " << intArrayMap3D.prefixLevel0 << ": " << elementLevel0 << std::endl;
				}
			}
		}
	}
	return stream;
}
}//namespace ITMLib

bool ITMIntArrayMap3D::LoadFromTextFile(const char* path) {
	internalMap.clear();
	std::ifstream file = std::ifstream(path, std::ios::in);
	if (!file) {
		std::cerr << ("Could not open " + std::string(path) + " for writing") << std::endl;
		return false;
	}
	int level3ElementCount, level2ElementCount, level1ElementCount, level0ElementCount;
	int level3Element, level2Element, level1Element, level0Element;
	file.read(reinterpret_cast<char*>(&level3ElementCount), sizeof(int));
	for (int iLevel3Element = 0; iLevel3Element < level3ElementCount; iLevel3Element++) {
		file.read(reinterpret_cast<char*>(&level3Element), sizeof(int));
		internalMap[level3Element] = std::map<int, std::map<int, std::vector<int>>>();
		file.read(reinterpret_cast<char*>(&level2ElementCount), sizeof(int));
		for (int iLevel2Element = 0; iLevel2Element < level2ElementCount; iLevel2Element++) {
			file.read(reinterpret_cast<char*>(&level2Element), sizeof(int));
			internalMap[level3Element][level2Element] = std::map<int, std::vector<int>>();
			file.read(reinterpret_cast<char*>(&level1ElementCount), sizeof(int));
			for (int iLevel1Element = 0; iLevel1Element < level1ElementCount; iLevel1Element++) {
				file.read(reinterpret_cast<char*>(&level1Element), sizeof(int));
				internalMap[level3Element][level2Element][level1Element] = std::vector<int>();
				std::vector<int>& array = internalMap[level3Element][level2Element][level1Element];
				file.read(reinterpret_cast<char*>(&level0ElementCount), sizeof(int));
				for (int iLevel0Element = 0; iLevel0Element < level1ElementCount; iLevel0Element++) {
					file.read(reinterpret_cast<char*>(&level0Element), sizeof(int));
					array.push_back(level0Element);
				}
			}
		}
	}
	file.close();
	return true;
}

bool ITMIntArrayMap3D::operator==(const ITMIntArrayMap3D& other) const {
	if(internalMap.size() != other.internalMap.size()){
		return false;
	}
	auto itThisLevel3 = internalMap.begin();
	auto itOtherLevel3 = other.internalMap.begin();

	while(itThisLevel3 != internalMap.end()){
		if(itThisLevel3->first != itOtherLevel3->first || itThisLevel3->second.size() != itOtherLevel3->second.size()){
			return false;
		}
		auto itThisLevel2 = itThisLevel3->second.begin();
		auto itOtherLevel2 = itOtherLevel3->second.begin();
		while(itThisLevel2 != itThisLevel3->second.end()){
			if(itThisLevel2->first != itOtherLevel2->first || itThisLevel2->second.size() != itOtherLevel2->second.size()){
				return false;
			}
			auto itThisLevel1 = itThisLevel2->second.begin();
			auto itOtherLevel1 = itOtherLevel2->second.begin();
			while(itThisLevel1 != itThisLevel2->second.end()){
				if(itThisLevel1->first != itOtherLevel1->first || itThisLevel1->second.size() != itOtherLevel1->second.size()){
					return false;
				}
				auto itThisLevel0 = itThisLevel1->second.begin();
				auto itOtherLevel0 = itOtherLevel1->second.begin();
				while (itThisLevel0 != itThisLevel1->second.end()){
					if(*itThisLevel0 != *itOtherLevel0){
						return false;
					}
					itThisLevel0++; itOtherLevel0++;
				}
				itThisLevel1++; itOtherLevel1++;
			}
			itThisLevel2++; itOtherLevel2++;
		}
		itThisLevel3++; itOtherLevel3++;
	}
	return true;
}
