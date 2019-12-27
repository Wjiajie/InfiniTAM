//  ================================================================
//  Created by Gregory Kramida on 11/12/19.
//  Copyright (c) 2019 Gregory Kramida
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

#include <string>

#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
namespace po = boost::program_options;
namespace pt = boost::property_tree;

struct InputPaths{

	static InputPaths FromVariablesMap(const po::variables_map& vm, const std::function<void()>& printHelp);
	static InputPaths FromJsonFile(std::string);
	static InputPaths FromPropertyTree(const pt::ptree& pt);

	std::string calibFilePath;
	std::string openniFilePath;
	std::string rgbVideoFilePath;
	std::string depthVideoFilePath;
	std::string rgbImageFileMask;
	std::string depthImageFileMask;
	std::string maskImageFileMask;
	std::string imuInputPath;
};
