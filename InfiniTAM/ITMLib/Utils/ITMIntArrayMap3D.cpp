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

	if(this->internalMap.find(keyLevel3) == (this->internalMap).end()){
		internalMap[keyLevel3] = std::map<int, std::map<int, std::vector<int>>>();
	}
	auto& valueLevel3 = internalMap[keyLevel3];
	if(valueLevel3.find(keyLevel2) == valueLevel3.end()){
		valueLevel3[keyLevel2] = std::map<int, std::vector<int>>();
	}
	auto& valueLevel2 = valueLevel3[keyLevel2];
	if(valueLevel2.find(keyLevel1) == valueLevel2.end()){
		valueLevel2[keyLevel1] = std::vector<int>();
	}
	std::vector<int>& valueLevel1 = valueLevel2[keyLevel1];
	if(valueLevel1.empty() ||
			valueLevel1[valueLevel1.size()-1] != valueLevel0){
		valueLevel1.push_back(valueLevel0);
		return true;
	}
	return false;
}

ITMIntArrayMap3D::ITMIntArrayMap3D() : internalMap() {}

bool ITMIntArrayMap3D::SaveToFile(const char* path) {
	std::ofstream file = std::ofstream(path,std::ofstream::binary | std::ofstream::out);
	if (!file){
		std::cerr << ("Could not open " + std::string(path) + " for writing") <<std::endl;
		return false;
	}

	file.write(reinterpret_cast<const char* >(internalMap.size()), sizeof(int));
	for(std::pair<int,std::map<int, std::map<int, std::vector<int>>>> elementLevel3 : internalMap ){
		file.write(reinterpret_cast<const char* >(elementLevel3.first), sizeof(int));
		file.write(reinterpret_cast<const char* >(elementLevel3.second.size()), sizeof(int));
		for(std::pair<int, std::map<int,std::vector<int>>> elementLevel2 : elementLevel3.second){
			file.write(reinterpret_cast<const char* >(elementLevel2.first), sizeof(int));
			file.write(reinterpret_cast<const char* >(elementLevel2.second.size()), sizeof(int));
			for(std::pair<int, std::vector<int>> elementLevel1 : elementLevel2.second ){
				file.write(reinterpret_cast<const char* >(elementLevel1.first), sizeof(int));
				file.write(reinterpret_cast<const char* >(elementLevel1.second.size()), sizeof(int));
				for(int elementLevel0 : elementLevel1.second){
					file.write(reinterpret_cast<const char* >(elementLevel0), sizeof(int));
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
	if (!file){
		std::cerr << ("Could not open " + std::string(path) + " for writing") <<std::endl;
		return false;
	}
	int level3ElementCount, level2ElementCount, level1ElementCount, level0ElementCount;
	int level3Element, level2Element, level1Element, level0Element;
	file.read(reinterpret_cast<char*>(&level3ElementCount), sizeof(int));
	for(int iLevel3Element = 0; iLevel3Element < level3ElementCount; iLevel3Element++){
		file.read(reinterpret_cast<char*>(&level3Element), sizeof(int));
		internalMap[level3Element] = std::map<int, std::map<int, std::vector<int>>>();
		file.read(reinterpret_cast<char*>(&level2ElementCount), sizeof(int));
		for(int iLevel2Element = 0; iLevel2Element < level2ElementCount; iLevel2Element++){
			file.read(reinterpret_cast<char*>(&level2Element), sizeof(int));
			internalMap[level3Element][level2Element] = std::map<int, std::vector<int>>();
			file.read(reinterpret_cast<char*>(&level1ElementCount), sizeof(int));
			for(int iLevel1Element = 0; iLevel1Element < level1ElementCount; iLevel1Element++){
				file.read(reinterpret_cast<char*>(&level1Element), sizeof(int));
				internalMap[level3Element][level2Element][level1Element] = std::vector<int>();
				std::vector<int>& array = internalMap[level3Element][level2Element][level1Element];
				file.read(reinterpret_cast<char*>(&level0ElementCount), sizeof(int));
				for(int iLevel0Element = 0; iLevel0Element < level1ElementCount; iLevel0Element++){
					file.read(reinterpret_cast<char*>(&level0Element), sizeof(int));
					array.push_back(level0Element);
				}
			}
		}
	}
	file.close();
	return true;
}
