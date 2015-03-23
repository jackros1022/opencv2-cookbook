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

#define IMAGE_DATA 1


#define POINT(x1)  (cv::Point(x1(0)/x1(2), x1(1)/x1(2)))
void drawCube (cv::Mat& img, cv::Mat_<double>& K, cv::Mat& Rmat, cv::Mat& tvec) {
    cv::Point3f p0(0,0,0);
    cv::Point3f p1(3,0,0);
    cv::Point3f p2(0,3,0);
    cv::Point3f p3(0,0,3);
    
    cv::Point3f p4(3,3,0);
    cv::Point3f p5(3,0,3);
    cv::Point3f p6(0,3,3);
    cv::Point3f p7(3,3,3);
    
    cv::Mat_<double> x0 = K * (Rmat * cv::Mat_<double>(p0) + tvec);
    cv::Mat_<double> x1 = K * (Rmat * cv::Mat_<double>(p1) + tvec);
    cv::Mat_<double> x2 = K * (Rmat * cv::Mat_<double>(p2) + tvec);
    cv::Mat_<double> x3 = K * (Rmat * cv::Mat_<double>(p3) + tvec);
    cv::Mat_<double> x4 = K * (Rmat * cv::Mat_<double>(p4) + tvec);
    cv::Mat_<double> x5 = K * (Rmat * cv::Mat_<double>(p5) + tvec);
    cv::Mat_<double> x6 = K * (Rmat * cv::Mat_<double>(p6) + tvec);
    cv::Mat_<double> x7 = K * (Rmat * cv::Mat_<double>(p7) + tvec);
    
    cv::line (img, POINT(x0), POINT(x1), cv::Scalar(0,0,255), 2); // X-axis
    cv::line (img, POINT(x0), POINT(x2), cv::Scalar(0,255,0), 2); // Y-axis
    cv::line (img, POINT(x0), POINT(x3), cv::Scalar(255,0,0), 2); // Z-axis
    
    cv::line (img, POINT(x1), POINT(x4), cv::Scalar(155,255,255), 2); // Z-axis
    cv::line (img, POINT(x1), POINT(x5), cv::Scalar(155,255,255), 2); // Z-axis
    
    cv::line (img, POINT(x2), POINT(x4), cv::Scalar(155,255,255), 2); // Z-axis
    cv::line (img, POINT(x2), POINT(x6), cv::Scalar(155,255,255), 2); // Z-axis
    
    cv::line (img, POINT(x3), POINT(x5), cv::Scalar(155,255,255), 2); // Z-axis
    cv::line (img, POINT(x3), POINT(x6), cv::Scalar(155,255,255), 2); // Z-axis
    
    cv::line (img, POINT(x7), POINT(x5), cv::Scalar(155,255,255), 2); // Z-axis
    cv::line (img, POINT(x7), POINT(x6), cv::Scalar(155,255,255), 2); // Z-axis
    cv::line (img, POINT(x7), POINT(x4), cv::Scalar(155,255,255), 2); // Z-axis
}
#undef POINT

int main()
{
    
    cv::namedWindow("Image");
    cv::Mat image;
    std::vector<std::string> filelist;
    cv::Size boardSize;
    
    // generate list of chessboard image filename
    for (int i=1; i<=20; i++) {
        if (0) {
            boardSize = cv::Size(6,4);
            std::stringstream str;
            str << "./chessboards/chessboard" << std::setw(2) << std::setfill('0') << i << ".jpg";
            std::cout << str.str() << std::endl;
            
            filelist.push_back(str.str());
            image= cv::imread(str.str(),0);
        } else if (IMAGE_DATA==1) {
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
        
        cv::waitKey(200);
    }
    
    //
    // pose estimation test
    //
    if (IMAGE_DATA) {
        cv::Mat srcI = cv::imread ("chessboards2/test.jpg");
        cv::Mat undistortedSrc = cameraCalibrator.undistort (srcI); // distCoeff = 0, from now on.
        cv::Mat gray;
        cv::cvtColor (undistortedSrc, gray, CV_BGR2GRAY);
        
        std::vector<cv::Point3f> objectCorners = cameraCalibrator.getObjectPoints(0); // obj coord. are all the same 2D pattern
        std::vector<cv::Point2f> imagePoints;
        bool found = cv::findChessboardCorners (gray, cameraCalibrator.getBoardSize(), imagePoints);
        if (!found) {
            cerr << "corners not found" << endl;
        }
        cv::cornerSubPix(gray, imagePoints, cv::Size(5,5), cv::Size(-1,-1),
                         cv::TermCriteria(cv::TermCriteria::MAX_ITER +
                                          cv::TermCriteria::EPS,
                                          30,		// max number of iterations
                                          0.1)      // min accuracy
                         );
        if (imagePoints.size() != boardSize.area()) {
            cerr << "the number of corners not match" << endl;
        }
        cv::Mat disp=undistortedSrc.clone();
        cv::drawChessboardCorners (disp, cameraCalibrator.getBoardSize(), imagePoints, found);
        cv::imshow ("Pose Estimation: Corner Detection Result", disp);
        
        // now compute the pose: R, T
        cv::Mat_<double> K = cameraCalibrator.getCameraMatrix();
        cv::Mat rvec, tvec;
        cv::solvePnP (objectCorners, imagePoints,
                      K, cv::Mat() /* distcoeff */,
                      rvec, tvec, false, CV_ITERATIVE);
        cv::Mat Rmat;
        cv::Rodrigues (rvec, Rmat);

        double mean = 0.; // mean of L2 distance
        for (int k=0; k<objectCorners.size(); k++) {
            // rotation
            // (3x1) = (3x3) * (3x1)
            cv::Mat_<double> Xc = Rmat * cv::Mat_<double>(objectCorners[k]);
            // translation
            Xc = Xc + tvec;
            // perspective projection
            Xc /= Xc(2);
            // no radial distortion
            double u = K(0,0)*Xc(0) + K(0,2);
            double v = K(1,1)*Xc(1) + K(1,2);
            
            double eu = u - imagePoints[k].x;
            double ev = v - imagePoints[k].y;
            double err = sqrt( eu*eu + ev*ev );
            //cerr << "err = " << err << endl;
            mean += err;
            
            cv::circle (undistortedSrc, cv::Point2f(u, v), 3, cv::Scalar(0,155,0), 1);
        }
        cerr << "- mean of L2 erros = " << mean / objectCorners.size() << endl;
        
        drawCube (undistortedSrc, K, Rmat, tvec);
        cv::imshow ("Pose estimation, reprojection", undistortedSrc);
    }
    cv::waitKey();
    return 0;
}
