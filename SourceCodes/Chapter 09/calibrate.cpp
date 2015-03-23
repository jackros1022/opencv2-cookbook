/*------------------------------------------------------------------------------------------*\
 This file contains material supporting chapter 9 of the cookbook:
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
#include <iomanip>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "CameraCalibrator.h"

int main()
{
    
    cv::namedWindow("Image");
    cv::Mat image;
    std::vector<std::string> filelist;
    cv::Size boardSize;
    
    // generate list of chessboard image filename
    for (int i=1; i<=20; i++) {
        if (1) {
            boardSize = cv::Size(6,4);
            std::stringstream str;
            str << "./chessboards/chessboard" << std::setw(2) << std::setfill('0') << i << ".jpg";
            std::cout << str.str() << std::endl;
            
            filelist.push_back(str.str());
            image= cv::imread(str.str(),0);
        } else {
            boardSize = cv::Size(9,6);
            char fname[1024];
            sprintf (fname, "chessboards2/left%02d.jpg", i);
            image = cv::imread (fname, 0);
            if (image.empty()) continue;
            std::cerr << "reading: " << fname << std::endl;
            filelist.push_back (std::string(fname));
        }
        cv::imshow("Image",image);
        
        cv::waitKey(100);
    }
    
    // Create calibrator object
    CameraCalibrator cameraCalibrator;
    // add the corners from the chessboard
    
    std::cerr << "addChessboardPoints()" << std::endl;
    
    cameraCalibrator.addChessboardPoints(
                                         filelist,	// filenames of chessboard image
                                         boardSize);	// size of chessboard
                                                        // calibrate the camera
                                                        //	cameraCalibrator.setCalibrationFlag(true,true);
    std::cerr << "now run calibration" << std::endl;
    
    cameraCalibrator.calibrate();
    
    // display camera matrix
    cv::Mat cameraMatrix= cameraCalibrator.getCameraMatrix();
    std::cout << " Camera intrinsic: " << cameraMatrix.rows << "x" << cameraMatrix.cols << std::endl;
    std::cout << cameraMatrix.at<double>(0,0) << " " << cameraMatrix.at<double>(0,1) << " " << cameraMatrix.at<double>(0,2) << std::endl;
    std::cout << cameraMatrix.at<double>(1,0) << " " << cameraMatrix.at<double>(1,1) << " " << cameraMatrix.at<double>(1,2) << std::endl;
    std::cout << cameraMatrix.at<double>(2,0) << " " << cameraMatrix.at<double>(2,1) << " " << cameraMatrix.at<double>(2,2) << std::endl;

    
    cameraCalibrator.testReprojection();
    
    
    // Image Undistortion
    for (std::vector<std::string>::iterator it=filelist.begin(); it!=filelist.end(); ++it) {
        std::cerr << "reading: " << *it << std::endl;
        image = cv::imread(*it);
        cv::Mat uImage= cameraCalibrator.undistort(image);
    
        imshow("Original Image", image);
        imshow("Undistorted Image", uImage);
        
        cv::waitKey(500);
    }
    
    cv::waitKey();
    return 0;
}
