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

#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H

#include <vector>
#include <iostream>
using namespace std;

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>

class CameraCalibrator {
    
    std::vector<cv::Mat> srcImages;
    cv::Size imageSize;
    cv::Size boardSize;
    // input points
    std::vector<std::vector<cv::Point3f> > objectPoints;
    std::vector<std::vector<cv::Point2f> > imagePoints;
    // output Matrices
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    //Output rotations and translations
    std::vector<cv::Mat> rvecs, tvecs;
    // flag to specify how calibration is done
    int flag;
    // used in image undistortion
    cv::Mat map1,map2;
    bool mustInitUndistort;
    
public:
    CameraCalibrator() : flag(0), mustInitUndistort(true) {};
    
    // Open the chessboard images and extract corner points
    int addChessboardPoints(const std::vector<std::string>& filelist, cv::Size & boardSize);
    // Add scene points and corresponding image points
    void addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners);
    // Calibrate the camera
    double calibrate();
    // test the projection
    void  testReprojection();
    // Set the calibration flag
    void setCalibrationFlag(bool radial8CoeffEnabled=false, bool tangentialParamEnabled=false);
    // Remove distortion in an image (after calibration)
    cv::Mat undistort(const cv::Mat &image);
    //
    
    // Getters
    cv::Mat getCameraMatrix() { return cameraMatrix; }
    cv::Mat getDistCoeffs()   { return distCoeffs; }
    
    void getObjectPoints(std::vector<cv::Point3f>& obj, int i) { obj = objectPoints[i]; }
    std::vector<cv::Point3f> getObjectPoints(int i) { return objectPoints[i]; }
    cv::Size getBoardSize() { return boardSize; }
    
};

#endif // CAMERACALIBRATOR_H
