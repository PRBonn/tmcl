/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: NDLUnitTests.cpp    	                                                #
# ##############################################################################
**/

#include "gtest/gtest.h"
#include "TextSpotting.h"
#include "ImageUtils.h"
#include <algorithm>

std::string textDataPath = PROJECT_TEST_DATA_DIR + std::string("/text/");



TEST(TextSpotting, test1) {

    std::string imgPath = textDataPath + "test.jpg";
    cv::Mat frame = cv::imread(imgPath);
    TextSpotting ts = TextSpotting(textDataPath + "textspotting.config");
     std::vector<std::string> texts  = ts.Infer(frame);
    /* for(int i = 0; i < texts.size(); ++i)
    {
    	std::cout << texts[i] << std::endl;
    }*/
     ASSERT_EQ("JAMES", texts[1]);
  }
  

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


