/*------------------------------------------------------------------------------------------*\
   This file contains material supporting chapter 2 of the cookbook:  
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
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

void salt(cv::Mat &image, int n) {

	int i,j;
	for (int k=0; k<n; k++) {

		i= rand()%image.cols;
		j= rand()%image.rows;

		if (image.channels() == 1) { // gray-level image

			image.at<uchar>(j,i)= 255; 

		} else if (image.channels() == 3) 
		{ // color image

			image.at<cv::Vec3b>(j,i)[0]= 255; 
			image.at<cv::Vec3b>(j,i)[1]= 255; 
			image.at<cv::Vec3b>(j,i)[2]= 255; 
		}
	}
}

void salt2 (cv::Mat_<cv::Vec3b>& img, int N)
{
	for (int n=0; n<N; n++)
	{
		int c = rand()%img.cols;
		int r = rand()%img.rows;
		img(r,c) = cv::Vec3b(255,0,0);
	}
}

int main()
{
	srand(cv::getTickCount()); // init random number generator

	cv::Mat image= cv::imread("boldt.jpg");
	
	cv::Mat_<cv::Vec3b> im2 = image;
	
	cerr << "nchannel= " << image.channels() << endl
	<< "dim = " << image.dims << endl
	<< "rows= " << image.rows << " cols= " << image.cols << endl;
	
	salt(image,3000);
	salt2(im2,3000);
	
	cv::namedWindow("Image");
	//cv::imshow("Image",image);
	cv::imshow("Image",im2);

	cv::imwrite("salted.bmp",image);

	
	cv::waitKey(5000);

	return 0;
}


