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

//stdlib
#include <iostream>
#include <exception>
// boost
#include <boost/property_tree/json_parser.hpp>

//local
#include "InputPaths.h"


bool isPathMask(const std::string& arg) {
	return arg.find('%') != std::string::npos;
}

InputPaths InputPaths::FromVariablesMap(const po::variables_map& vm, const std::function<void()>& printHelp) {

	std::string calibFilePath;
	if (vm.count("calib_file")) {
		calibFilePath = vm["calib_file"].as<std::string>();
	}

	//all initialized to empty string by default
	std::string openniFilePath, rgbVideoFilePath, depthVideoFilePath, rgbImageFileMask, depthImageFileMask,
			maskImageFileMask, imuInputPath;

	std::vector<std::string> inputPaths;
	if (vm.count("input_path")) {
		inputPaths = vm["input_path"].as<std::vector<std::string>>();
	}
	auto inputFileCount = inputPaths.size();
	switch (inputFileCount) {
		case 0:
			//no input files
			break;
		case 1:
			//a single OpenNI file
			openniFilePath = inputPaths[0];
			break;
		case 3:
		default:
			if (isPathMask(inputPaths[2])) { maskImageFileMask = inputPaths[2]; }
			else { imuInputPath = inputPaths[2]; }
		case 2:
			if (isPathMask(inputPaths[0]) && isPathMask(inputPaths[1])) {
				rgbImageFileMask = inputPaths[0];
				depthImageFileMask = inputPaths[1];
			} else if (!isPathMask(inputPaths[0]) && !isPathMask(inputPaths[1])) {
				rgbVideoFilePath = inputPaths[0];
				depthVideoFilePath = inputPaths[1];
			} else {
				std::cerr << "The first & second input_path arguments need to either both be masks or both be"
				             " paths to video files." << std::endl;
				printHelp();
				throw std::runtime_error("Could not parse command-line arguments");
			}
			break;
	}
	return {calibFilePath, openniFilePath, rgbVideoFilePath,
	        depthVideoFilePath, rgbImageFileMask, depthImageFileMask, maskImageFileMask,
	        imuInputPath};
}

InputPaths InputPaths::FromPropertyTree(const pt::ptree& pt) {

	//all initialized to empty string by default
	std::string calibFilePath, openniFilePath, rgbVideoFilePath, depthVideoFilePath,
			rgbImageFileMask, depthImageFileMask, maskImageFileMask, imuInputPath;

	auto value_or_empty_string = [](boost::optional<std::string> optional) {
		return optional ? optional.get() : "";
	};

	calibFilePath = value_or_empty_string(pt.get_optional<std::string>("input.calibration_file_path"));
	openniFilePath = value_or_empty_string(pt.get_optional<std::string>("input.openni_file_path"));
	rgbVideoFilePath = value_or_empty_string(pt.get_optional<std::string>("input.rgb_video_file_path"));
	depthVideoFilePath = value_or_empty_string(pt.get_optional<std::string>("input.depth_video_file_path"));
	rgbImageFileMask = value_or_empty_string(pt.get_optional<std::string>("input.rgb_image_path_mask"));
	depthImageFileMask = value_or_empty_string(pt.get_optional<std::string>("input.depth_image_path_mask"));
	maskImageFileMask = value_or_empty_string(pt.get_optional<std::string>("input.mask_image_path_mask"));
	imuInputPath = value_or_empty_string(pt.get_optional<std::string>("input.imp_input_path"));

	return {calibFilePath,
	        openniFilePath,
	        rgbVideoFilePath,
	        depthVideoFilePath,
	        rgbImageFileMask,
	        depthImageFileMask, maskImageFileMask,
	        imuInputPath};

}

InputPaths InputPaths::FromJsonFile(std::string path) {
	pt::ptree tree;
	pt::read_json(path, tree);
	return FromPropertyTree(tree);
}

