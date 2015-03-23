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
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "harrisDetector.h"

int main()
{
	// Read input image
	cv::Mat image= cv::imread("../images/church01.jpg",0);
	if (!image.data)
		return 0; 
    
    // Display the image
	cv::namedWindow("Original Image");
	cv::imshow("Original Image",image);
    
	// Detect Harris Corners
	cv::Mat cornerStrength;
	cv::cornerHarris(image,cornerStrength,
                     3,     // neighborhood size
					 3,     // aperture size
					 0.01); // Harris parameter
    
    // threshold the corner strengths
	cv::Mat harrisCorners;
	double threshold= 0.0001; 
	cv::threshold(cornerStrength,harrisCorners,
                  threshold,255,cv::THRESH_BINARY_INV);
    
    // Display the corners
	cv::namedWindow("Harris Corner Map");
	cv::imshow("Harris Corner Map",harrisCorners);
    
    cv::waitKey ();
    
	// Create Harris detector instance
	HarrisDetector harris;
    // Compute Harris values
	harris.detect(image);
    // Detect Harris corners
	std::vector<cv::Point> pts;
	harris.getCorners(pts,0.01);
	// Draw Harris corners
	harris.drawOnImage(image,pts);
    
    // Display the corners
	cv::namedWindow("Harris Corners");
	cv::imshow("Harris Corners",image);

    cv::waitKey ();

	// Read input image
	image= cv::imread("../images/church01.jpg",0);
    if (!image.data) return 0;
    
	// Compute good features to track
	std::vector<cv::Point2f> corners;
    
	cv::goodFeaturesToTrack(image,corners,
                            500,	// maximum number of corners to be returned
                            0.01,	// quality level
                            10);	// minimum allowed distance between points

    std::cerr << "gftt.corners() size()= " << corners.size() << std::endl;
    
	// for all corners
	std::vector<cv::Point2f>::iterator it= corners.begin();
	while (it!=corners.end()) {       
		// draw a circle at each corner location
		cv::circle(image,*it,3,cv::Scalar(5,255,5),2);
		++it;
	}
    
    // Display the corners
	cv::namedWindow("Good Features to Track");
	cv::imshow("Good Features to Track",image);
    
    cv::waitKey ();

	// Read input image
	image= cv::imread("../images/church01.jpg",0);
    
	// vector of keypoints
	std::vector<cv::KeyPoint> keypoints;
	// Construction of the Good Feature to Track detector 
	cv::GoodFeaturesToTrackDetector gftt(
                                         500,	// maximum number of corners to be returned
                                         0.01,	// quality level
                                         10);	// minimum allowed distance between points
	// point detection using FeatureDetector method
	gftt.detect(image,keypoints);
	
    std::cerr<< "GFTT Features Detected: " << keypoints.size() << std::endl;
    
	cv::drawKeypoints(image,		// original image
                      keypoints,					// vector of keypoints
                      image,						// the resulting image
                      cv::Scalar(255,255,5),	// color of the points
                      cv::DrawMatchesFlags::DRAW_OVER_OUTIMG); //drawing flag
    
    // Display the corners
	cv::namedWindow("Good Features to Track Detector");
	cv::imshow("Good Features to Track Detector",image);
    
    cv::waitKey ();

	// Read input image
	image= cv::imread("../images/church01.jpg",0);
    
	keypoints.clear();
	cv::FastFeatureDetector fast(40);
	fast.detect(image,keypoints);
	
    std::cerr<< "FAST Features Detected: " << keypoints.size() << std::endl;
    
	cv::drawKeypoints(image,keypoints,image,
                      cv::Scalar(255,5,255),
                      cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);
    
    // Display the corners
	cv::namedWindow("FAST Features");
	cv::imshow("FAST Features",image);
    
    cv::waitKey ();

	// Read input image
	image= cv::imread("../images/church03.jpg",0);
    
	keypoints.clear();
	// Construct the SURF feature detector object
	cv::SurfFeatureDetector surf(2500);
	// Detect the SURF features
	surf.detect(image,keypoints);
	
    std::cerr<< "SURF Features Detected: " << keypoints.size() << std::endl;
    
	cv::Mat featureImage;
	cv::drawKeypoints(image,keypoints,featureImage,
                      //cv::Scalar(5,255,255),
                      cv::Scalar(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    
    // Display the corners
	cv::namedWindow("SURF Features");
	cv::imshow("SURF Features",featureImage);
    
    cv::waitKey ();

	// Read input image
	image= cv::imread("../images/church01.jpg",0);
    
	keypoints.clear();
	// Construct the SURF feature detector object
	cv::SiftFeatureDetector 
    sift(
         0.03,  // feature threshold
         10.);  // threshold to reduce sensitivity to lines
    
	// Detect the SURF features
	sift.detect(image,keypoints);

    std::cerr<< "SIFT Features Detected: " << keypoints.size() << std::endl;
    
	cv::drawKeypoints(image,keypoints,featureImage,
                      cv::Scalar(255,5,255),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    
    // Display the corners
	cv::namedWindow("SIFT Features");
	cv::imshow("SIFT Features",featureImage);
    
    cv::waitKey ();

	// Read input image
	image= cv::imread("../images/church01.jpg",0);
    
	keypoints.clear();
    
	cv::MserFeatureDetector mser;
	mser.detect(image,keypoints);
    
    std::cerr<< "MSER Features Detected: " << keypoints.size() << std::endl;
    
	// Draw the keypoints with scale and orientation information
	cv::drawKeypoints(image,		// original image
                      keypoints,					// vector of keypoints
                      featureImage,				// the resulting image
                      cv::Scalar(255,255,255),	// color of the points
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS); //drawing flag
    
    // Display the corners
	cv::namedWindow("MSER Features");
	cv::imshow("MSER Features",featureImage);
    
	cv::waitKey();
	return 0;
}