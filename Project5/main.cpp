//C
#include <stdio.h>
//C++
#include <iostream>
#include <sstream>

#ifdef WINDOWS
#include<conio.h>           // it may be necessary to change or remove this line if not using Windows
#endif

#include "VehicleDetection.h"
#include "Blob.h"




void help();


void help()
{
	std::cout
		<< "--------------------------------------------------------------------------" << std::endl
		<< "This program shows how to use background subtraction methods provided by " << std::endl
		<< " OpenCV. You can process both videos (-vid) and images (-img)." << std::endl
		<< std::endl
		<< "Usage:" << std::endl
		<< "./bs {-vid <video filename>|-img <image filename>}" << std::endl
		<< "for example: ./bs -vid video.avi" << std::endl
		<< "or: ./bs -img /data/images/1.png" << std::endl
		<< "--------------------------------------------------------------------------" << std::endl
		<< std::endl;
}
int main(int argc, char* argv[])
{
	//print help information
	help();
	//check for the input parameter correctness
	if (argc != 3) {
		std::cerr << "Incorret input list" << std::endl;
		std::cerr << "exiting..." << std::endl;
		return EXIT_FAILURE;
	}

	std::unique_ptr<VehicleDetection> vehicleDetection = std::make_unique<VehicleDetection>();


	if (strcmp(argv[1], "-vid") == 0) {
		//input data coming from a video
		//processVideo(argv[2]);
		vehicleDetection->processVideo(argv[2]);
	}
	else if (strcmp(argv[1], "-img") == 0) {
		//input data coming from a sequence of images
		//processImages(argv[2]);
	}
	else {
		//error in reading input parameters
		std::cerr << "Please, check the input parameters." << std::endl;
		std::cerr << "Exiting..." << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
////////////////////////////////////////////////

//OLD CODE
// 
//void processVideo(char* videoFilename) {
//
//	//create the capture object
//	VideoCapture capture(videoFilename);
//	if (!capture.isOpened()) {
//		//error in opening the video input
//		cerr << "Unable to open video file: " << videoFilename << endl;
//		exit(EXIT_FAILURE);
//	}
//
//	///new
//	bool blnFirstFrame = true;
//
//	///KF
//	// >>>> Kalman Filter
//	int stateSize = 6;
//	int measSize = 4;
//	int contrSize = 0;
//
//	unsigned int type = CV_32F;
//	cv::KalmanFilter kf(stateSize, measSize, contrSize, type);
//	cv::KalmanFilter kf2(stateSize, measSize, contrSize, type);
//	cv::KalmanFilter kf3(stateSize, measSize, contrSize, type);
//	cv::KalmanFilter kf4(stateSize, measSize, contrSize, type);
//	cv::Mat state2(stateSize, 1, type);
//	cv::Mat meas2(measSize, 1, type);
//	cv::Mat state3(stateSize, 1, type);
//	cv::Mat meas3(measSize, 1, type);
//	cv::Mat state4(stateSize, 1, type);
//	cv::Mat meas4(measSize, 1, type);
//	
//	cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
//	cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
//										//cv::Mat procNoise(stateSize, 1, type)
//										// [E_x,E_y,E_v_x,E_v_y,E_w,E_h]
//
//										// Transition State Matrix A
//										// Note: set dT at each processing step!
//										// [ 1 0 dT 0  0 0 ]
//										// [ 0 1 0  dT 0 0 ]
//										// [ 0 0 1  0  0 0 ]
//										// [ 0 0 0  1  0 0 ]
//										// [ 0 0 0  0  1 0 ]
//										// [ 0 0 0  0  0 1 ]
//	
//	cv::setIdentity(kf.transitionMatrix);
//	cv::setIdentity(kf2.transitionMatrix);
//	cv::setIdentity(kf3.transitionMatrix);
//	cv::setIdentity(kf4.transitionMatrix);
//
//	// Measure Matrix H
//	// [ 1 0 0 0 0 0 ]
//	// [ 0 1 0 0 0 0 ]
//	// [ 0 0 0 0 1 0 ]
//	// [ 0 0 0 0 0 1 ]
//	kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
//	kf.measurementMatrix.at<float>(0) = 1.0f;
//	kf.measurementMatrix.at<float>(7) = 1.0f;
//	kf.measurementMatrix.at<float>(16) = 1.0f;
//	kf.measurementMatrix.at<float>(23) = 1.0f;
//
//	kf2.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
//	kf2.measurementMatrix.at<float>(0) = 1.0f;
//	kf2.measurementMatrix.at<float>(7) = 1.0f;
//	kf2.measurementMatrix.at<float>(16) = 1.0f;
//	kf2.measurementMatrix.at<float>(23) = 1.0f;
//
//	kf3.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
//	kf3.measurementMatrix.at<float>(0) = 1.0f;
//	kf3.measurementMatrix.at<float>(7) = 1.0f;
//	kf3.measurementMatrix.at<float>(16) = 1.0f;
//	kf3.measurementMatrix.at<float>(23) = 1.0f;
//
//	kf4.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
//	kf4.measurementMatrix.at<float>(0) = 1.0f;
//	kf4.measurementMatrix.at<float>(7) = 1.0f;
//	kf4.measurementMatrix.at<float>(16) = 1.0f;
//	kf4.measurementMatrix.at<float>(23) = 1.0f;
//
//	// Process Noise Covariance Matrix Q
//	// [ Ex   0   0     0     0    0  ]
//	// [ 0    Ey  0     0     0    0  ]
//	// [ 0    0   Ev_x  0     0    0  ]
//	// [ 0    0   0     Ev_y  0    0  ]
//	// [ 0    0   0     0     Ew   0  ]
//	// [ 0    0   0     0     0    Eh ]
//	//cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
//	kf.processNoiseCov.at<float>(0) = 1e-2;
//	kf.processNoiseCov.at<float>(7) = 1e-2;
//	kf.processNoiseCov.at<float>(14) = 5.0f;
//	kf.processNoiseCov.at<float>(21) = 5.0f;
//	kf.processNoiseCov.at<float>(28) = 1e-2;
//	kf.processNoiseCov.at<float>(35) = 1e-2;
//
//	kf2.processNoiseCov.at<float>(0) = 1e-2;
//	kf2.processNoiseCov.at<float>(7) = 1e-2;
//	kf2.processNoiseCov.at<float>(14) = 5.0f;
//	kf2.processNoiseCov.at<float>(21) = 5.0f;
//	kf2.processNoiseCov.at<float>(28) = 1e-2;
//	kf2.processNoiseCov.at<float>(35) = 1e-2;
//
//	kf3.processNoiseCov.at<float>(0) = 1e-2;
//	kf3.processNoiseCov.at<float>(7) = 1e-2;
//	kf3.processNoiseCov.at<float>(14) = 5.0f;
//	kf3.processNoiseCov.at<float>(21) = 5.0f;
//	kf3.processNoiseCov.at<float>(28) = 1e-2;
//	kf3.processNoiseCov.at<float>(35) = 1e-2;
//
//	kf4.processNoiseCov.at<float>(0) = 1e-2;
//	kf4.processNoiseCov.at<float>(7) = 1e-2;
//	kf4.processNoiseCov.at<float>(14) = 5.0f;
//	kf4.processNoiseCov.at<float>(21) = 5.0f;
//	kf4.processNoiseCov.at<float>(28) = 1e-2;
//	kf4.processNoiseCov.at<float>(35) = 1e-2;
//
//	// Measures Noise Covariance Matrix R
//	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
//	cv::setIdentity(kf2.measurementNoiseCov, cv::Scalar(1e-1));
//	cv::setIdentity(kf3.measurementNoiseCov, cv::Scalar(1e-1));
//	cv::setIdentity(kf4.measurementNoiseCov, cv::Scalar(1e-1));
//	// <<<< Kalman Filter
//	char ch = 0;
//
//	double ticks = 0;
//	bool found = false;
//
//	int notFoundCount = 0;
//	///KF end
//
//	//read input data. ESC or 'q' for quitting
//	while ((char)keyboard != 'q' && (char)keyboard != 27) {
//		//read the current frame
//
//		//KF
//		double precTick = ticks;
//		ticks = (double)cv::getTickCount();
//
//		double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds
//			
//		///KF end
//
//		//Draw box video frame
//		capture.read(imgFrame2);
//
//		///Convex Hull defin blobs
//		
//		std::vector<Blob> currentFrameBlobs;
//		///
//
//		if (!capture.read(frame)) {
//			cerr << "Unable to read next frame." << endl;
//			cerr << "Exiting..." << endl;
//			exit(EXIT_FAILURE);
//		}
//		//update the background model
//		pMOG2->apply(frame, fgMaskMOG2,-1); //// MOG Learning rate
//		//get the frame number and write it on the current frame
//		stringstream ss;
//		rectangle(frame, cv::Point(10, 2), cv::Point(100, 20),
//			cv::Scalar(255, 255, 255), -1);
//		ss << capture.get(CAP_PROP_POS_FRAMES);
//		string frameNumberString = ss.str();
//		putText(frame, frameNumberString.c_str(), cv::Point(15, 15),
//			FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
//
//		//KF
//		cv::Mat res;
//		frame.copyTo(res);
//		//KF end
//
//		//show the current frame and the fg masks
//		imshow("Frame", frame);
//		imshow("FG Mask MOG 2", fgMaskMOG2);
//		cv::Rect predRect6;
//		cv::Rect predRect5;
//		//KF
//		if (found)
//		{
//			// >>>> Matrix A
//			kf.transitionMatrix.at<float>(2) = dT;
//			kf.transitionMatrix.at<float>(9) = dT;
//			// <<<< Matrix A
//
//			//cout << "dT:" << endl << dT << endl;
//
//			state = kf.predict();
//			//cout << "State post:" << endl << state << endl;
//
//			cv::Rect predRect;
//			predRect.width = state.at<float>(4);
//			predRect.height = state.at<float>(5);
//			predRect.x = state.at<float>(0) - predRect.width / 2;
//			predRect.y = state.at<float>(1) - predRect.height / 2;
//			predRect6 = predRect;
//
//			cv::Point center;
//			center.x = state.at<float>(0);
//			center.y = state.at<float>(1);
//			cv::circle(res, center, 2, SCALAR_BLUE, -1);
//
//			cv::rectangle(res, predRect, SCALAR_BLUE, 2);
//		}
//		if (found)
//		{
//			// >>>> Matrix A
//			kf2.transitionMatrix.at<float>(2) = dT;
//			kf2.transitionMatrix.at<float>(9) = dT;
//			// <<<< Matrix A
//
//			//cout << "dT:" << endl << dT << endl;
//
//			state2 = kf2.predict();
//			//cout << "State post:" << endl << state << endl;
//
//			cv::Rect predRect2;
//			predRect2.width = state2.at<float>(4);
//			predRect2.height = state2.at<float>(5);
//			predRect2.x = state2.at<float>(0) - predRect2.width / 2;
//			predRect2.y = state2.at<float>(1) - predRect2.height / 2;
//			predRect5 = predRect2;
//
//			cv::Point center2;
//			center2.x = state2.at<float>(0);
//			center2.y = state2.at<float>(1);
//			cv::circle(res, center2, 2, SCALAR_BLUE, -1);
//
//			cv::rectangle(res, predRect2, SCALAR_BLUE, 2);
//		}
//		
//		///Predictie
//		cv::Rect predRect3;
//		cv::Point center3;
//		cv::Rect predRect4;
//		cv::Point center4;
//		
//		if (found)
//		{
//			// >>>> Matrix A
//			kf3.transitionMatrix.at<float>(2) = dT;
//			kf3.transitionMatrix.at<float>(9) = dT;
//			// <<<< Matrix A
//
//			//cout << "dT:" << endl << dT << endl;
//
//			state3 = kf3.predict();
//
//			//cout << "State post:" << endl << state << endl;
//
//			
//			
//			predRect3.width = state3.at<float>(4);
//			predRect3.height = state3.at<float>(5);
//			predRect3.x = (state3.at<float>(0) - predRect3.width / 2);
//			predRect3.y = (state3.at<float>(1) - predRect3.height / 2);
//			for (int i = 0; i <= 6; i++) {
//				meas3.at<float>(0) = predRect3.x + predRect3.width / 2;
//				meas3.at<float>(1) = predRect3.y + predRect3.height / 2;
//				meas3.at<float>(2) = (float)predRect3.width;
//				meas3.at<float>(3) = (float)predRect3.height;
//				kf3.correct(meas3);
//				state3 = kf3.predict();			
//				predRect3.width = state3.at<float>(4);
//				predRect3.height = state3.at<float>(5);
//				predRect3.x = state3.at<float>(0) - predRect3.width / 2;
//				predRect3.y = state3.at<float>(1) - predRect3.height / 2;
//			}
//
//			
//			center3.x = state3.at<float>(0);
//			center3.y = state3.at<float>(1);
//
//			
//			
//		}
//		
//
//		
//		if (found)
//		{
//			// >>>> Matrix A
//			kf4.transitionMatrix.at<float>(2) = dT;
//			kf4.transitionMatrix.at<float>(9) = dT;
//			// <<<< Matrix A
//
//			//cout << "dT:" << endl << dT << endl;
//
//			state4 = kf4.predict();
//
//			//cout << "State post:" << endl << state << endl;
//
//			
//			predRect4.width = state4.at<float>(4);
//			predRect4.height = state4.at<float>(5);
//			predRect4.x = (state4.at<float>(0) - predRect4.width / 2);
//			predRect4.y = (state4.at<float>(1) - predRect4.height / 2);
//			for (int i = 0; i <= 6; i++) {
//				meas4.at<float>(0) = predRect4.x + predRect4.width / 2;
//				meas4.at<float>(1) = predRect4.y + predRect4.height / 2;
//				meas4.at<float>(2) = (float)predRect4.width;
//				meas4.at<float>(3) = (float)predRect4.height;
//				kf4.correct(meas4);
//				state4 = kf4.predict();
//				predRect4.width = state4.at<float>(4);
//				predRect4.height = state4.at<float>(5);
//				predRect4.x = state4.at<float>(0) - predRect4.width / 2;
//				predRect4.y = state4.at<float>(1) - predRect4.height / 2;
//			}
//
//			
//			center4.x = state4.at<float>(0);
//			center4.y = state4.at<float>(1);
//
//			
//		}
//		double colid = distanceBetweenPoints(center3, center4);
//		double colidmindist=120;
//		Scalar Coliz;
//		if (colid < colidmindist) {
//			Coliz = SCALAR_RED;
//		}
//		else {
//			Coliz = SCALAR_WHITE;
//		}
//		///pred out
//		cv::circle(res, center3, 2, Coliz, -1);
//		
//		cv::rectangle(res, predRect3, Coliz, 2);
//		
//		cv::circle(res, center4, 2, Coliz, -1);
//		
//		cv::rectangle(res, predRect4, Coliz, 2);
//
//		//KF end
//
//
//		////Image Conturs ->start
//		cv::Mat structuringElement2x2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
//		cv::Mat structuringElement3x3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
//		cv::Mat structuringElement5x5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
//		cv::Mat structuringElement7x7 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
//		cv::Mat structuringElement9x9 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
//
//
//		cv::erode(fgMaskMOG2, fgMaskMOG2, structuringElement3x3);
//		cv::dilate(fgMaskMOG2, fgMaskMOG2, structuringElement5x5);
//		cv::erode(fgMaskMOG2, fgMaskMOG2, structuringElement3x3);
//		cv::dilate(fgMaskMOG2, fgMaskMOG2, structuringElement5x5);
//		cv::dilate(fgMaskMOG2, fgMaskMOG2, structuringElement3x3);
//		cv::erode(fgMaskMOG2, fgMaskMOG2, structuringElement5x5);
//
//
//
//
//
//		cv::Mat fgMaskMOG2Copy = fgMaskMOG2.clone();
//
//		std::vector<std::vector<cv::Point> > contours;
//
//		cv::findContours(fgMaskMOG2Copy, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//
//		drawAndShowContours(fgMaskMOG2.size(), contours, "imgContours");
//		
//	
//		////Image Conturs ->fin
//
//		////Convex_Hull ->start
//		std::vector<std::vector<cv::Point> > convexHulls(contours.size());
//		//vector<vector<cv::Point> > balls;
//		//vector<cv::Rect> ballsBox;
//
//		for (unsigned int i = 0; i < contours.size(); i++) {
//			cv::convexHull(contours[i], convexHulls[i]);
//
//		}
//
//		drawAndShowContours(fgMaskMOG2.size(), convexHulls, "imgConvexHulls");
//		vector<vector<cv::Point> > CurrentBlobs;
//		vector<cv::Rect> Box;//KF
//		
//
//		for (auto &convexHull : convexHulls) {
//			Blob possibleBlob(convexHull);
//			
//			
//			if (possibleBlob.currentBoundingRect.area() > 70 && /// Blob filtre param
//																 //possibleBlob.dblCurrentAspectRatio >= 0.2 &&
//																 //possibleBlob.dblCurrentAspectRatio <= 1.25 &&
//				possibleBlob.currentBoundingRect.width > 20 &&
//				possibleBlob.currentBoundingRect.height > 20 &&
//				possibleBlob.dblCurrentDiagonalSize > 15.0 &&
//				(cv::contourArea(possibleBlob.currentContour) / (double)possibleBlob.currentBoundingRect.area()) > 0.20) {
//				currentFrameBlobs.push_back(possibleBlob);
//				
//			}
//		}
//
//		drawAndShowContours(fgMaskMOG2.size(), currentFrameBlobs, "imgCurrentFrameBlobs");
//
//
//		///KF
//		cv::Mat rangeRes = cv::Mat::zeros(fgMaskMOG2Copy.size(), CV_8UC1);
//		cv::inRange(fgMaskMOG2Copy, cv::Scalar(MIN_H_BLUE / 255, 255, 255),
//			cv::Scalar(MAX_H_BLUE / 255, 255, 255), rangeRes);
//		cv::Mat invSrc = cv::Scalar::all(255) - rangeRes; /// invert rangeRes
//		vector<vector<cv::Point> > contours2;
//		cv::findContours(invSrc, contours2, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//		imshow("rangeRes", invSrc); 
//
//		// >>>>>> Filtering 
//		vector<vector<cv::Point> > cars;
//		vector<cv::Rect> carsBox;
//		cv::Rect bBox;
//		for (size_t i = 0; i < contours2.size(); i++)
//		{
//			
//			bBox = cv::boundingRect(contours2[i]);
//
//			float ratio = (float)bBox.width / (float)bBox.height;
//			float DiagonalSize = sqrt(pow(bBox.width, 2) + pow(bBox.height, 2));
//
//			// Searching for a bBox almost square
//			if (bBox.area() > 70 && /// Blob filtre param
//																//ratio >= 0.2 &&
//																//ratio <= 1.25 &&
//				bBox.width > 20 &&
//				bBox.height > 20 &&
//				DiagonalSize > 15.0 &&
//				(cv::contourArea(contours2[i]) / (double)bBox.area()) > 0.20)
//			{
//				cars.push_back(contours2[i]);
//				carsBox.push_back(bBox);
//			}
//		}
//		// <<<<< Filtering
//
//		// >>>>> Detection result
//		for (size_t i = 0; i < cars.size(); i++)
//		{
//			cv::drawContours(res, contours2, 0, CV_RGB(20, 150, 20), 1);
//			cv::rectangle(res, carsBox[i], CV_RGB(0, 255, 0), 2);
//
//			cv::Point center;
//			center.x = carsBox[i].x + carsBox[i].width / 2;
//			center.y = carsBox[i].y + carsBox[i].height / 2;
//			cv::circle(res, center, 2, CV_RGB(20, 150, 20), -1);
//
//			stringstream sstr;
//			//sstr << "(" << center.x << "," << center.y << ")";
//			cv::putText(res, sstr.str(),
//				cv::Point(center.x + 3, center.y - 3),
//				cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20, 150, 20), 2);
//			float DiagonalSize = sqrt(pow(bBox.width, 2) + pow(bBox.height, 2));
//			int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
//			double dblFontScale = DiagonalSize / 60.0;
//			int intFontThickness = (int)std::round(dblFontScale * 2.0);
//			
//			cv::putText(res, std::to_string(i), cv::Point(center.x + 3, center.y - 3), intFontFace, dblFontScale, SCALAR_GREEN, intFontThickness);
//		}
//		// <<<<< Detection result
//
//		// >>>>> Kalman Update
//		if (cars.size() == 1) {
//			for (size_t i = 0; i < cars.size(); i++) {
//				if (cars.size() == 0)
//				{
//					notFoundCount++;
//					//cout << "notFoundCount:" << notFoundCount << endl;
//					if (notFoundCount >= 100)
//					{
//						found = false;
//					}
//					/*else
//					kf.statePost = state;*/
//				}
//				else
//				{
//					notFoundCount = 0;
//
//					meas.at<float>(0) = carsBox[0].x + carsBox[0].width / 2;
//					meas.at<float>(1) = carsBox[0].y + carsBox[0].height / 2;
//					meas.at<float>(2) = (float)carsBox[0].width;
//					meas.at<float>(3) = (float)carsBox[0].height;
//
//
//					meas3.at<float>(0) = predRect6.x + predRect6.width / 2;
//					meas3.at<float>(1) = predRect6.y + predRect6.height / 2;
//					meas3.at<float>(2) = (float)predRect6.width;
//					meas3.at<float>(3) = (float)predRect6.height;
//
//					meas4.at<float>(0) = predRect5.x + predRect5.width / 2;
//					meas4.at<float>(1) = predRect5.y + predRect5.height / 2;
//					meas4.at<float>(2) = (float)predRect5.width;
//					meas4.at<float>(3) = (float)predRect5.height;
//
//					if (!found) // First detection!
//					{
//						// >>>> Initialization
//						kf.errorCovPre.at<float>(0) = 1; // px
//						kf.errorCovPre.at<float>(7) = 1; // px
//						kf.errorCovPre.at<float>(14) = 1;
//						kf.errorCovPre.at<float>(21) = 1;
//						kf.errorCovPre.at<float>(28) = 1; // px
//						kf.errorCovPre.at<float>(35) = 1; // px
//
//						state.at<float>(0) = meas.at<float>(0);
//						state.at<float>(1) = meas.at<float>(1);
//						state.at<float>(2) = 0;
//						state.at<float>(3) = 0;
//						state.at<float>(4) = meas.at<float>(2);
//						state.at<float>(5) = meas.at<float>(3);
//						// <<<< Initialization
//
//						// >>>> Initialization
//						kf3.errorCovPre.at<float>(0) = 1; // px
//						kf3.errorCovPre.at<float>(7) = 1; // px
//						kf3.errorCovPre.at<float>(14) = 1;
//						kf3.errorCovPre.at<float>(21) = 1;
//						kf3.errorCovPre.at<float>(28) = 1; // px
//						kf3.errorCovPre.at<float>(35) = 1; // px
//
//						state3.at<float>(0) = meas3.at<float>(0);
//						state3.at<float>(1) = meas3.at<float>(1);
//						state3.at<float>(2) = 0;
//						state3.at<float>(3) = 0;
//						state3.at<float>(4) = meas3.at<float>(2);
//						state3.at<float>(5) = meas3.at<float>(3);
//						// <<<< Initialization
//
//						// >>>> Initialization
//						kf4.errorCovPre.at<float>(0) = 1; // px
//						kf4.errorCovPre.at<float>(7) = 1; // px
//						kf4.errorCovPre.at<float>(14) = 1;
//						kf4.errorCovPre.at<float>(21) = 1;
//						kf4.errorCovPre.at<float>(28) = 1; // px
//						kf4.errorCovPre.at<float>(35) = 1; // px
//						
//						state4.at<float>(0) = meas4.at<float>(0);
//						state4.at<float>(1) = meas4.at<float>(1);
//						state4.at<float>(2) = 0;
//						state4.at<float>(3) = 0;
//						state4.at<float>(4) = meas4.at<float>(2);
//						state4.at<float>(5) = meas4.at<float>(3);
//						// <<<< Initialization
//
//						kf.statePost = state;
//						kf3.statePost = state3;
//						kf4.statePost = state4;
//
//						found = true;
//					}
//					else
//						kf.correct(meas); // Kalman Correction
//					kf3.correct(meas3);
//					kf4.correct(meas4);
//
//					//cout << "Measure matrix:" << endl << meas << endl;
//				}
//			}
//			////
//		}
//		else {
//			for (size_t i = 0; i < cars.size(); i++) {
//				if (cars.size() == 0)
//				{
//					notFoundCount++;
//					//cout << "notFoundCount:" << notFoundCount << endl;
//					if (notFoundCount >= 100)
//					{
//						found = false;
//					}
//					/*else
//					kf.statePost = state;*/
//				}
//				else
//				{
//					notFoundCount = 0;
//
//					meas.at<float>(0) = carsBox[0].x + carsBox[0].width / 2;
//					meas.at<float>(1) = carsBox[0].y + carsBox[0].height / 2;
//					meas.at<float>(2) = (float)carsBox[0].width;
//					meas.at<float>(3) = (float)carsBox[0].height;
//
//
//					meas2.at<float>(0) = carsBox[1].x + carsBox[1].width / 2;
//					meas2.at<float>(1) = carsBox[1].y + carsBox[1].height / 2;
//					meas2.at<float>(2) = (float)carsBox[1].width;
//					meas2.at<float>(3) = (float)carsBox[1].height;
//
//					meas3.at<float>(0) = predRect6.x + predRect6.width / 2;
//					meas3.at<float>(1) = predRect6.y + predRect6.height / 2;
//					meas3.at<float>(2) = (float)predRect6.width;
//					meas3.at<float>(3) = (float)predRect6.height;
//
//					meas4.at<float>(0) = predRect5.x + predRect5.width / 2;
//					meas4.at<float>(1) = predRect5.y + predRect5.height / 2;
//					meas4.at<float>(2) = (float)predRect5.width;
//					meas4.at<float>(3) = (float)predRect5.height;
//
//					if (!found) // First detection!
//					{
//						// >>>> Initialization
//						kf.errorCovPre.at<float>(0) = 1; // px
//						kf.errorCovPre.at<float>(7) = 1; // px
//						kf.errorCovPre.at<float>(14) = 1;
//						kf.errorCovPre.at<float>(21) = 1;
//						kf.errorCovPre.at<float>(28) = 1; // px
//						kf.errorCovPre.at<float>(35) = 1; // px
//
//						state.at<float>(0) = meas.at<float>(0);
//						state.at<float>(1) = meas.at<float>(1);
//						state.at<float>(2) = 0;
//						state.at<float>(3) = 0;
//						state.at<float>(4) = meas.at<float>(2);
//						state.at<float>(5) = meas.at<float>(3);
//						// <<<< Initialization
//
//						// >>>> Initialization
//						kf2.errorCovPre.at<float>(0) = 1; // px
//						kf2.errorCovPre.at<float>(7) = 1; // px
//						kf2.errorCovPre.at<float>(14) = 1;
//						kf2.errorCovPre.at<float>(21) = 1;
//						kf2.errorCovPre.at<float>(28) = 1; // px
//						kf2.errorCovPre.at<float>(35) = 1; // px
//
//						state2.at<float>(0) = meas2.at<float>(0);
//						state2.at<float>(1) = meas2.at<float>(1);
//						state2.at<float>(2) = 0;
//						state2.at<float>(3) = 0;
//						state2.at<float>(4) = meas2.at<float>(2);
//						state2.at<float>(5) = meas2.at<float>(3);
//						// <<<< Initialization
//
//						// >>>> Initialization
//						kf3.errorCovPre.at<float>(0) = 1; // px
//						kf3.errorCovPre.at<float>(7) = 1; // px
//						kf3.errorCovPre.at<float>(14) = 1;
//						kf3.errorCovPre.at<float>(21) = 1;
//						kf3.errorCovPre.at<float>(28) = 1; // px
//						kf3.errorCovPre.at<float>(35) = 1; // px
//
//						state3.at<float>(0) = meas3.at<float>(0);
//						state3.at<float>(1) = meas3.at<float>(1);
//						state3.at<float>(2) = 0;
//						state3.at<float>(3) = 0;
//						state3.at<float>(4) = meas3.at<float>(2);
//						state3.at<float>(5) = meas3.at<float>(3);
//						// <<<< Initialization
//
//
//						// >>>> Initialization
//						kf4.errorCovPre.at<float>(0) = 1; // px
//						kf4.errorCovPre.at<float>(7) = 1; // px
//						kf4.errorCovPre.at<float>(14) = 1;
//						kf4.errorCovPre.at<float>(21) = 1;
//						kf4.errorCovPre.at<float>(28) = 1; // px
//						kf4.errorCovPre.at<float>(35) = 1; // px
//
//						state4.at<float>(0) = meas4.at<float>(0);
//						state4.at<float>(1) = meas4.at<float>(1);
//						state4.at<float>(2) = 0;
//						state4.at<float>(3) = 0;
//						state4.at<float>(4) = meas4.at<float>(2);
//						state4.at<float>(5) = meas4.at<float>(3);
//						// <<<< Initialization
//
//						kf.statePost = state;
//						kf2.statePost = state2;
//						kf3.statePost = state3;
//						kf4.statePost = state4;
//
//						found = true;
//					}
//					else
//						kf.correct(meas); // Kalman Correction
//					kf2.correct(meas2);
//					kf3.correct(meas3);
//					kf4.correct(meas4);
//
//					//cout << "Measure matrix:" << endl << meas << endl;
//				}
//			}
//		}
//		// <<<<< Kalman Update
//		//KF end
//		imshow("Tracking", res);
//		//KF end
//
//		if (blnFirstFrame == true) {
//			for (auto &currentFrameBlob : currentFrameBlobs) {
//				blobs.push_back(currentFrameBlob);
//			}
//		}
//		else {
//			matchCurrentFrameBlobsToExistingBlobs(blobs, currentFrameBlobs);
//		}
//
//		drawAndShowContours(fgMaskMOG2.size(), blobs, "imgBlobs");
//
//		cv::Mat imgFrame2Copy = imgFrame2.clone();
//		imgFrame2Copy = imgFrame2.clone();          // get another copy of frame 2 since we changed the previous frame 2 copy in the processing above
//
//		drawBlobInfoOnImage(blobs, imgFrame2Copy);
//
//		cv::imshow("imgFrame2Copy", imgFrame2Copy);
//
//		currentFrameBlobs.clear();
//
//		
//
//		////Convex_Hull ->fin
//		blnFirstFrame = false;
//
//		//get the input from the keyboard
//		keyboard = waitKey(30);
//	}
//	//delete capture object
//	capture.release();
//}
//void processImages(char* fistFrameFilename) {
//	//read the first file of the sequence
//	frame = imread(fistFrameFilename);
//	if (frame.empty()) {
//		//error in opening the first image
//		cerr << "Unable to open first image frame: " << fistFrameFilename << endl;
//		exit(EXIT_FAILURE);
//	}
//	//current image filename
//	string fn(fistFrameFilename);
//	//read input data. ESC or 'q' for quitting
//	while ((char)keyboard != 'q' && (char)keyboard != 27) {
//		//update the background model
//		pMOG2->apply(frame, fgMaskMOG2);
//		//get the frame number and write it on the current frame
//		size_t index = fn.find_last_of("/");
//		if (index == string::npos) {
//			index = fn.find_last_of("\\");
//		}
//		size_t index2 = fn.find_last_of(".");
//		string prefix = fn.substr(0, index + 1);
//		string suffix = fn.substr(index2);
//		string frameNumberString = fn.substr(index + 1, index2 - index - 1);
//		istringstream iss(frameNumberString);
//		int frameNumber = 0;
//		iss >> frameNumber;
//		rectangle(frame, cv::Point(10, 2), cv::Point(100, 20),
//			cv::Scalar(255, 255, 255), -1);
//		putText(frame, frameNumberString.c_str(), cv::Point(15, 15),
//			FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
//		//show the current frame and the fg masks
//		imshow("Frame", frame);
//		imshow("FG Mask MOG 2", fgMaskMOG2);
//		//get the input from the keyboard
//		keyboard = waitKey(30);
//		//search for the next image in the sequence
//		ostringstream oss;
//		oss << (frameNumber + 1);
//		string nextFrameNumberString = oss.str();
//		string nextFrameFilename = prefix + nextFrameNumberString + suffix;
//		//read the next frame
//		frame = imread(nextFrameFilename);
//		if (frame.empty()) {
//			//error in opening the next image in the sequence
//			cerr << "Unable to open image frame: " << nextFrameFilename << endl;
//			exit(EXIT_FAILURE);
//		}
//		//update the path of the current frame
//		fn.assign(nextFrameFilename);
//	}
//}
//
//
/////Functions:
/////////////////////////////////////////////////////////////////////////////////////////////////////
//void matchCurrentFrameBlobsToExistingBlobs(std::vector<Blob> &existingBlobs, std::vector<Blob> &currentFrameBlobs) {
//
//	for (auto &existingBlob : existingBlobs) {
//
//		existingBlob.blnCurrentMatchFoundOrNewBlob = false;
//
//		existingBlob.predictNextPosition();
//	}
//
//	for (auto &currentFrameBlob : currentFrameBlobs) {
//
//		int intIndexOfLeastDistance = 0;
//		double dblLeastDistance = 100000.0;
//
//		for (unsigned int i = 0; i < existingBlobs.size(); i++) {
//			if (existingBlobs[i].blnStillBeingTracked == true) {
//				double dblDistance = distanceBetweenPoints(currentFrameBlob.centerPositions.back(), existingBlobs[i].predictedNextPosition);
//
//				if (dblDistance < dblLeastDistance) {
//					dblLeastDistance = dblDistance;
//					intIndexOfLeastDistance = i;
//				}
//			}
//		}
//
//		if (dblLeastDistance < currentFrameBlob.dblCurrentDiagonalSize * 1.15) {
//			addBlobToExistingBlobs(currentFrameBlob, existingBlobs, intIndexOfLeastDistance);
//		}
//		else {
//			addNewBlob(currentFrameBlob, existingBlobs);
//		}
//
//	}
//
//	for (auto &existingBlob : existingBlobs) {
//
//		if (existingBlob.blnCurrentMatchFoundOrNewBlob == false) {
//			existingBlob.intNumOfConsecutiveFramesWithoutAMatch++;
//		}
//
//		if (existingBlob.intNumOfConsecutiveFramesWithoutAMatch >= 5) {
//			existingBlob.blnStillBeingTracked = false;
//		}
//
//	}
//
//}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
//void addBlobToExistingBlobs(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs, int &intIndex) {
//
//	existingBlobs[intIndex].currentContour = currentFrameBlob.currentContour;
//	existingBlobs[intIndex].currentBoundingRect = currentFrameBlob.currentBoundingRect;
//
//	existingBlobs[intIndex].centerPositions.push_back(currentFrameBlob.centerPositions.back());
//
//	existingBlobs[intIndex].dblCurrentDiagonalSize = currentFrameBlob.dblCurrentDiagonalSize;
//	existingBlobs[intIndex].dblCurrentAspectRatio = currentFrameBlob.dblCurrentAspectRatio;
//
//	existingBlobs[intIndex].blnStillBeingTracked = true;
//	existingBlobs[intIndex].blnCurrentMatchFoundOrNewBlob = true;
//}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
//void addNewBlob(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs) {
//
//	currentFrameBlob.blnCurrentMatchFoundOrNewBlob = true;
//
//	existingBlobs.push_back(currentFrameBlob);
//}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
//double distanceBetweenPoints(cv::Point point1, cv::Point point2) {
//
//	int intX = abs(point1.x - point2.x);
//	int intY = abs(point1.y - point2.y);
//
//	return(sqrt(pow(intX, 2) + pow(intY, 2)));
//}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
//void drawAndShowContours(cv::Size imageSize, std::vector<std::vector<cv::Point> > contours, std::string strImageName) {
//	cv::Mat image(imageSize, CV_8UC3, SCALAR_BLACK);
//
//	cv::drawContours(image, contours, -1, SCALAR_WHITE, -1);
//
//	cv::imshow(strImageName, image);
//}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
//void drawAndShowContours(cv::Size imageSize, std::vector<Blob> blobs, std::string strImageName) {
//
//	cv::Mat image(imageSize, CV_8UC3, SCALAR_BLACK);
//
//	std::vector<std::vector<cv::Point> > contours;
//
//	for (auto &blob : blobs) {
//		if (blob.blnStillBeingTracked == true) {
//			contours.push_back(blob.currentContour);
//		}
//	}
//
//	cv::drawContours(image, contours, -1, SCALAR_WHITE, -1);
//
//	cv::imshow(strImageName, image);
//}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
//void drawBlobInfoOnImage(std::vector<Blob> &blobs, cv::Mat &imgFrame2Copy) {
//
//	for (unsigned int i = 0; i < blobs.size(); i++) {
//
//		if (blobs[i].blnStillBeingTracked == true) {
//			cv::rectangle(imgFrame2Copy, blobs[i].currentBoundingRect, SCALAR_RED, 2);
//
//			int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
//			double dblFontScale = blobs[i].dblCurrentDiagonalSize / 60.0;
//			int intFontThickness = (int)std::round(dblFontScale * 1.0);
//
//			cv::putText(imgFrame2Copy, std::to_string(i), blobs[i].centerPositions.back(), intFontFace, dblFontScale, SCALAR_GREEN, intFontThickness);
//		}
//	}
//}
//
