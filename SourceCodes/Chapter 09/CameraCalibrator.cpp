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

#include "CameraCalibrator.h"

// Open chessboard images and extract corner points
int CameraCalibrator::addChessboardPoints(
                                          const std::vector<std::string>& filelist,
                                          cv::Size & boardSize) {
    
    // the points on the chessboard
    std::vector<cv::Point2f> imageCorners;
    std::vector<cv::Point3f> objectCorners;
    
    // 3D Scene Points:
    // Initialize the chessboard corners
    // in the chessboard reference frame
    // The corners are at 3D location (X,Y,Z)= (i,j,0)
    for (int i=0; i<boardSize.height; i++) {
        for (int j=0; j<boardSize.width; j++) {
            
            objectCorners.push_back(cv::Point3f(i, j, 0.0f));
        }
    }
    
    // 2D Image points:
    int successes = 0;
    // for all viewpoints
    for (int i=0; i<filelist.size(); i++) {
        
        // Open the image
        cv::Mat image = cv::imread(filelist[i],0);
        if (image.empty()) {
            cerr << "! image read error: " << filelist[i] << endl;
            
        }
        // Get the chessboard corners
        bool found = cv::findChessboardCorners(
                                               image, boardSize, imageCorners);
        
        if (!found) {
            cerr << "! Error in findChesboardCorners: " << filelist[i] << endl;
        }
        // Get subpixel accuracy on the corners
        cv::cornerSubPix(image, imageCorners,
                         cv::Size(5,5),
                         cv::Size(-1,-1),
                         cv::TermCriteria(cv::TermCriteria::MAX_ITER +
                                          cv::TermCriteria::EPS,
                                          30,		// max number of iterations
                                          0.1));     // min accuracy
        
        // If we have a good board, add it to our data
        if (imageCorners.size() == boardSize.area()) {
            this->imageSize = image.size();
            // Add image and scene points from one view
            addPoints(imageCorners, objectCorners);
            successes++;
        }
    
        this->boardSize = boardSize;
        
        //Draw the corners
        cv::Mat cimage = cv::imread(filelist[i]);
        srcImages.push_back (cimage.clone());
        cv::drawChessboardCorners(cimage, boardSize, imageCorners, found);
        cv::imshow("Corners on Chessboard", cimage);
        cv::waitKey(200);
    }
    
    return successes;
}

// Add scene points and corresponding image points
void CameraCalibrator::addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners) {
    
    // 2D image points from one view
    imagePoints.push_back(imageCorners);
    // corresponding 3D scene points
    objectPoints.push_back(objectCorners);
}

// Calibrate the camera
// returns the re-projection error
double CameraCalibrator::calibrate()
{
    // undistorter must be reinitialized
    mustInitUndistort= true;
    
    // start calibration

    flag = flag | CV_CALIB_FIX_ASPECT_RATIO | CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_FIX_K3;
    double reprojE = calibrateCamera(objectPoints, // the 3D points
                                     imagePoints,  // the image points
                                     imageSize,    // image size
                                     cameraMatrix, // output camera matrix
                                     distCoeffs,   // output distortion matrix
                                     rvecs, tvecs, // Rs, Ts
                                     flag);        // set options
    
    std::cerr << "calibration finished with reproj. error: " << reprojE << std::endl;
    cerr << "distCoeffs: " << distCoeffs << endl;
    cerr << "K: " << cameraMatrix << endl;
    for (int i=0; i<rvecs.size(); i++)
        cerr << "rv:" << rvecs[i] << endl << "t: " << tvecs[i] << endl;
    cerr << "-----------" << endl;

    return reprojE;
}

// remove distortion in an image (after calibration)
cv::Mat CameraCalibrator::undistort(const cv::Mat &image) {
    
    cv::Mat undistorted;
    
    if (mustInitUndistort) { // called once per calibration
        
        cv::initUndistortRectifyMap(
                                    cameraMatrix,  // computed camera matrix
                                    distCoeffs,    // computed distortion matrix
                                    cv::Mat(),     // optional rectification (none)
                                    cameraMatrix,     // camera matrix to generate undistorted
                                    image.size(),  // size of undistorted
                                    CV_32FC1,      // type of output map
                                    map1, map2);   // the x and y mapping functions
        
        mustInitUndistort= false;
    }
    
    // Apply mapping functions
    cv::remap(image, undistorted, map1, map2, cv::INTER_LINEAR); // interpolation type
    
    return undistorted;
}

// reproject the obj points through the calibration parameters obtained,
// to see the appropriateness of the mathematical calibration model.
void  CameraCalibrator::testReprojection() {
    cv::Mat_<double> dcoeff = this->distCoeffs;
    cv::Mat_<double> K = this->cameraMatrix;
    
    for (int i=0; i<srcImages.size(); i++) {
        cv::Mat undistortedImage = this->undistort (srcImages[i]);
        
        std::vector<cv::Point3f> &Xw = objectPoints[i];
        std::vector<cv::Point2f> &p  = imagePoints[i];
        cv::Mat_<double> Rmat(3,3);
        cv::Rodrigues (rvecs[i], Rmat);
        
        double mean = 0.;
        for (int k=0; k<p.size(); k++) {
            // rotation
            // (3x1) = (3x3) * (3x1)
            cv::Mat_<double> Xc = Rmat * cv::Mat_<double>(Xw[k]);
            // translation
            Xc = Xc + tvecs[i];
            // perspective projection
            Xc /= Xc(2);
            // radial distortion
            double r2 = pow(Xc(0),2) + pow(Xc(1),2);
            double r4 = r2*r2;
            double xd = Xc(0) * (1. + dcoeff(0)*r2 + dcoeff(1)*r4);
            double yd = Xc(1) * (1. + dcoeff(0)*r2 + dcoeff(1)*r4);
            double u = K(0,0)*xd + K(0,2);
            double v = K(1,1)*yd + K(1,2);
            
            float xud = K(0,2) + K(0,0)*Xc(0);
            float yud = K(1,2) + K(1,1)*Xc(1);
            
            double eu = u - p[k].x;
            double ev = v - p[k].y;
            double err = sqrt( eu*eu + ev*ev );
//        cerr << "Xc = " << Xc << endl;
//cerr << "err = " << err << endl;
            mean += err;
        
            cv::circle (undistortedImage, cv::Point2f(xud, yud), 3, cv::Scalar(0,0,255), 2);
        }
        cerr << " --- " << mean/p.size() << endl;
        cv::imshow ("undistorted+reprojection", undistortedImage);
        cv::waitKey(300);
    }
}


// Set the calibration options
// 8radialCoeffEnabled should be true if 8 radial coefficients are required (5 is default)
// tangentialParamEnabled should be true if tangeantial distortion is present
void CameraCalibrator::setCalibrationFlag(bool radial8CoeffEnabled, bool tangentialParamEnabled) {
    
    // Set the flag used in cv::calibrateCamera()
    flag = 0;
    if (!tangentialParamEnabled) flag += CV_CALIB_ZERO_TANGENT_DIST;
    if (radial8CoeffEnabled) flag += CV_CALIB_RATIONAL_MODEL;
}

