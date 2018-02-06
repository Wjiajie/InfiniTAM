//  ================================================================
//  Created by Gregory Kramida on 2/2/18.
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
#pragma once

//stdlib
#include <vector>
#include <ostream>
#include <map>

namespace ITMLib{

template<typename T>
class ITM3DNestedMap {
public:
	ITM3DNestedMap();
	ITM3DNestedMap(const char* prefixLevel3, const char* prefixLevel2, const char* prefixLevel1,
	                 const char* prefixLevel0);
	ITM3DNestedMap(const ITM3DNestedMap& nestedMap3D);
	ITM3DNestedMap& operator=(ITM3DNestedMap& other);
	ITM3DNestedMap& operator=(ITM3DNestedMap&& other) noexcept;
	bool operator==(const ITM3DNestedMap &other) const;

	~ITM3DNestedMap();

	bool InsertOrdered(int keyLevel3, int keyLevel2, int keyLevel1, T valueLevel0);
	bool SaveToFile(const char* path);
	bool SaveToTextFile(const char* path);
	bool LoadFromFile(const char* path);
	ITM3DNestedMap FilterBasedOnLevel0Lengths(int minThreshold);


	std::vector<int> GetLevel3Keys();
	std::vector<int> GetOuterLevelKeys(){
		return GetLevel3Keys();
	};
	bool Contains(int keyLevel3, int keyLevel2, int keyLevel1, T valueLevel0);
	bool Contains(int keyLevel3, int keyLevel2);


	template<typename FT>
	friend std::ostream& operator<<(std::ostream& stream, const ITM3DNestedMap<FT>& nestedMap3D);

private:
	std::map<int, std::map<int, std::map<int, std::vector<T>>>> internalMap;
	const char* prefixLevel3;
	const char* prefixLevel2;
	const char* prefixLevel1;
	const char* prefixLevel0;

};

}//namespace ITMLib


