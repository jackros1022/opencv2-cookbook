/*------------------------------------------------------------------------------------------*\
 This file contains material supporting chapter 8 of the cookbook:  
 Computer Vision Programming using the OpenCV Library. 
 by Robert Laganiere, Packt Publishing, 2011.
 
 This program is free software; permission is hereby granted to use, copy, modify, 
 and distribute this source code, or portions thereof, for any purpose, without fee, 
 subject to the restriction that the copyright notice may not be removed 
 or altered from any source or altered source distribution. 
 The software is released on an as-is basis and without any warranties of any kind. 
 In particular, the software is not guaranteed to be fault-tolerant or free from failure. 
 The author disclaims all warranties with regard to this software, any use, 
 and any consequent failure, is purely the responsibility of the user.
 
 Copyright (C) 2010-2011 Robert Laganiere, www.laganiere.name
 \*------------------------------------------------------------------------------------------*/

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

const int iwidth=640;
const int iheight=480;

void drawMatchesRelative (cv::Mat& imgInput, 
                          std::vector<cv::KeyPoint>& keypoints1, 
                          std::vector<cv::KeyPoint>& keypoints2, 
                          std::vector<cv::DMatch>& matches, 
                          cv::Mat& out)
{
    imgInput.copyTo (out);
    for (int i=0; i<matches.size(); i++)
        {
        cv::Point2f pt_new = keypoints1[matches[i].queryIdx].pt;
        cv::Point2f pt_old = keypoints2[matches[i].trainIdx].pt;
        
        cv::line(out, pt_new, pt_old, cv::Scalar(125, 255, 125), 1);

        cv::circle (out, pt_new, 4, cv::Scalar(255,0,0), 1);
        cv::circle (out, pt_old, 2, cv::Scalar(0,0,225), 1);
        }
}

int main()
{
	// Construction of the SURF feature detector 
	cv::SurfFeatureDetector surf(3000);
	// Construction of the SURF descriptor extractor 
	cv::SurfDescriptorExtractor surfDesc;
	// Construction of the matcher 
	cv::BruteForceMatcher< cv::L2<float> > matcher;
    
    cv::VideoCapture capture;
    capture.open (0);
    
    if (!capture.isOpened())
        {
        std::cout << "capture device failed to open!" << std::endl;
        return 1;
        }
    
    capture.set(CV_CAP_PROP_FRAME_WIDTH, iwidth);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, iheight);
    
    std::cerr << "Video " <<
    ": width=" << capture.get(CV_CAP_PROP_FRAME_WIDTH) <<
    ", height=" << capture.get(CV_CAP_PROP_FRAME_HEIGHT) <<
    ", nframes=" << capture.get(CV_CAP_PROP_FRAME_COUNT) << std::endl;
    

    
	// Read input images
	cv::Mat image1, aframe;
    // vector of keypoints
    std::vector<cv::KeyPoint> keypoints1;
    cv::Mat descriptors1;

    do {
        capture >> aframe;
        if (!aframe.data) continue;
        
        // Detection of the SURF features
        surf.detect(aframe,keypoints1);

        surfDesc.compute(aframe,keypoints1,descriptors1);
        cv::waitKey(300);
    } while (keypoints1.size() < 10);

    aframe.copyTo(image1);
    imwrite ("frame1.png", image1);

    cv::namedWindow("Image 1");
    cv::imshow("Image 1",image1);

    cv::namedWindow("Matches");
    std::vector<char> mask;
    
    for (;;)
        {
        cv::Mat image2;
        capture >> image2;
        if (!image2.data) continue; 

        //cv::namedWindow("Image 2");
        //cv::imshow("Image 2",image2);
        
        std::vector<cv::KeyPoint> keypoints2;
        surf.detect(image2,keypoints2);
    
        // Extraction of the SURF descriptors
        cv::Mat descriptors2;
        surfDesc.compute(image2,keypoints2,descriptors2);
    
    
        // Match the two image descriptors
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1,descriptors2, matches);
    
        std::cout << "Number of matched points: " << matches.size() << std::endl;
    
        int nMatch=200;
        std::nth_element(matches.begin(),    // initial position
                         matches.begin()+nMatch, // position of the sorted element
                         matches.end());     // end position
        // remove all elements after the 25th
        matches.erase(matches.begin()+nMatch+1, matches.end()); 
    
        cv::Mat imageMatches;
        /***
        cv::drawMatches(image1, keypoints1,  // 1st image and its keypoints
                        image2, keypoints2,  // 2nd image and its keypoints
                        matches,			// the matches
                        imageMatches);//,		// the image produced
         ***/
        /***
                        cv::Scalar::all(-1),
                        cv::Scalar::all(-1),
                        mask,
                        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
                        ); // color of the lines
         ***/
        
        drawMatchesRelative (image2, keypoints1, keypoints2, matches, imageMatches);
        
        cv::imshow("Matches",imageMatches);
        cv::imshow("Image 1",image1);
        cv::waitKey (1000);
        }
	
    return 0;
}

// EOF //