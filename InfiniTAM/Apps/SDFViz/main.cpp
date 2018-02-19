//  ================================================================
//  Created by Gregory Kramida on 12/21/17.
//  Copyright (c) 2017-2025 Gregory Kramida
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
#include "SDFViz.h"
#include <boost/program_options.hpp>

namespace bpo = boost::program_options;

int main(int argc, const char* argv[]) {
	try{
		//visibility boolean flags
		bool showNonInterestCanonicalVoxels = false;
		bool showLiveVoxels = false;
		bool hideInterestCanonicalRegions = false;
		bpo::options_description description{"Options"};
		description.add_options()
				("help,h", "Help screen")
				("directory,d",bpo::value<std::string>()->default_value("/media/algomorph/Data/Reconstruction/debug_output/scene"),
				 "Directory where to load the SDF scene from.")
				("show_non_interest, sni", bpo::bool_switch(&showNonInterestCanonicalVoxels), "Show non-interest canonical voxels on startup.")
				("show_live, sl", bpo::bool_switch(&showLiveVoxels), "Show live voxels on startup.")
				("hide_interest, hcr", bpo::bool_switch(&hideInterestCanonicalRegions), "Hide interest canonical voxels on startup.")
				;

		bpo::variables_map vm;
		bpo::store(bpo::parse_command_line(argc, argv, description), vm);
		bpo::notify(vm);

		if (vm.count("help")){
			std::cout << description << std::endl;
		}else{
			SDFViz application(vm["directory"].as<std::string>(),
			                   showNonInterestCanonicalVoxels,
			                   showLiveVoxels,
			                   hideInterestCanonicalRegions);
			application.Run();
		}
	}catch(const bpo::error &ex){
		std::cerr << ex.what() << std::endl;
	}
}