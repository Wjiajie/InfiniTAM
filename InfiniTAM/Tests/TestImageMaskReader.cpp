//  ================================================================
//  Created by Gregory Kramida on 9/3/19.
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

#define BOOST_TEST_MODULE SceneConstruction
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//stdlib
#include <iostream>

//boost
#include <boost/test/unit_test.hpp>

//test targets
#include "../ORUtils/FileUtils.h"
#include "../InputSource/ImageSourceEngine.h"


BOOST_AUTO_TEST_CASE(testImageMaskReader) {

	using namespace InputSource;
	auto* rgb = new ITMUChar4Image(true, false);
	auto* depth = new ITMShortImage(true, false);
	auto* gtMaskedRgb = new ITMUChar4Image(true, false);
	auto* gtMaskedDepth = new ITMShortImage(true, false);

	ITMUCharImage* mask = new ITMUCharImage(true, false);

	InputSource::ImageMaskPathGenerator pathGenerator("frames/color_%06i.png", "frames/depth_%06i.png",
	                                                  "frames/omask_%06i.png");
	InputSource::ImageSourceEngine* imageSource = new InputSource::ImageFileReader<InputSource::ImageMaskPathGenerator>(
			"frames/snoopy_calib.txt", pathGenerator);
	imageSource->getImages(rgb, depth);

//	BOOST_REQUIRE(ReadImageFromFile(rgb, "frames/color_000000.png"));
//	BOOST_REQUIRE(ReadImageFromFile(depth, "frames/depth_000000.png"));
	BOOST_REQUIRE(ReadImageFromFile(mask, "frames/omask_000000.png"));

//	rgb->ApplyMask(*mask,Vector4u((unsigned char)0));
//	depth->ApplyMask(*mask,0);

//	SaveImageToFile(rgb, "frames/color_000000_masked2.pnm");
//	SaveImageToFile(depth, "frames/depth_000000_masked2.pnm");

	ReadImageFromFile(gtMaskedRgb, "frames/color_000000.png");
	gtMaskedRgb->ApplyMask(*mask, Vector4u((unsigned char) 0));
	ReadImageFromFile(gtMaskedDepth, "frames/depth_000000_masked.pnm");

	BOOST_REQUIRE(*rgb == *gtMaskedRgb);
	BOOST_REQUIRE(*depth == *gtMaskedDepth);

	delete rgb;
	delete depth;
	//delete mask;
	delete imageSource;
	delete gtMaskedDepth;
	delete gtMaskedRgb;
}