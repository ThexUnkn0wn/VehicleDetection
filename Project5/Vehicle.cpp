#include "vehicle.h"

#include <iostream>


Vehicle::Vehicle(std::vector<cv::Point> _contour):boundingRects(5), centerPositions(5)
{
	currentContour = _contour;
	cv::Rect currentBoundingRect = cv::boundingRect(currentContour);

	boundingRects.insert(currentBoundingRect);

	cv::Point currentCenter;

	currentCenter.x = currentBoundingRect.x + currentBoundingRect.width / 2;
	currentCenter.y = currentBoundingRect.y + currentBoundingRect.height / 2;

	currentDiagonalSize = sqrt(pow(currentBoundingRect.width, 2) + pow(currentBoundingRect.height, 2));

	centerPositions.insert(currentCenter);

	blnStillBeingTracked = true;
	blnCurrentMatchFoundOrNewVehicle = true;

	// Kalman Filter
	initKF();
	
}

Vehicle::Vehicle(Blob blob) :boundingRects(10), centerPositions(10)
{
	boundingRects.insert(blob.currentBoundingRect);

	currentDiagonalSize = blob.dblCurrentDiagonalSize;

	centerPositions.insert(blob.centerPositions.back());

	blnStillBeingTracked = true;
	blnCurrentMatchFoundOrNewVehicle = true;
	intNumOfConsecutiveFramesWithoutAMatch = 0;

	// Kalman Filter
	initKF();
}


void Vehicle::predictNextPosition(double& dt, cv::Mat& display,size_t &index)
{
	cv::Mat predState =cv::Mat(this->state);
	
	size_t head = centerPositions.getHead();
			
	if (found || blnStillBeingTracked)
	{
		kf.transitionMatrix.at<float>(2) = dt;
		kf.transitionMatrix.at<float>(9) = dt;

		state = kf.predict();
	}

	if (intNumOfConsecutiveFramesWithoutAMatch!=0)
	{
		notFoundCount++;
		std::cout << index;
		//cout << "notFoundCount:" << notFoundCount << endl;
		if (notFoundCount >= 50)
		{
			found = false;
			blnStillBeingTracked = false;
		}
			
	}
	else 
	{
		notFoundCount = 0;

		meas.at<float>(0) = centerPositions.back().x;
		meas.at<float>(1) = centerPositions.back().y;
		meas.at<float>(2) = boundingRects.back().width;
		meas.at<float>(3) = boundingRects.back().height;

		if (!found) // First detection!
		{
			state.at<float>(0) = centerPositions.back().x;
			state.at<float>(1) = centerPositions.back().y;
			state.at<float>(2) = 0;
			state.at<float>(3) = 0;
			state.at<float>(4) = boundingRects.back().width;
			state.at<float>(5) = boundingRects.back().height;
			kf.statePost = state;
			found = true;
		}
		else
		{
			// Kalman Correction
			kf.correct(meas);
		}

		predState = kf.statePost;
	}

	// Set prediction
	this->predPosition.x = predState.at<float>(0);
	this->predPosition.y = predState.at<float>(1);
	this->predRect = cv::Rect(this->predPosition.x - predState.at<float>(4)/2,		// x coord (top left)
								this->predPosition.y - predState.at<float>(5)/2,	// y coord (top left)
								predState.at<float>(4),								// width
								predState.at<float>(5));							// hight


	if (blnStillBeingTracked) {
		cv::circle(display, predPosition, 2, vd::SCALAR_BLUE, -1);
		cv::rectangle(display, predRect, vd::SCALAR_BLUE, 2);
	}
	

}

void Vehicle::initKF()
{
	int stateSize = 6;
	int measSize = 4;
	int contrSize = 0;
	unsigned int type = CV_32F;

	kf = cv::KalmanFilter(stateSize, measSize, contrSize, type);
	state = cv::Mat(stateSize, 1, type);		// [x,y,v_x,v_y,w,h]
	meas = cv::Mat(measSize, 1, type);			// [z_x,z_y,z_w,z_h]
	procNoise = cv::Mat(stateSize, 1, type);	// [Ex,Ey,Ev_x,Ev_y,Ew,Eh]

	// Transition State Matrix A
	// Note: set dT at each processing step!
	// [ 1 0 dT 0  0 0 ]
	// [ 0 1 0  dT 0 0 ]
	// [ 0 0 1  0  0 0 ]
	// [ 0 0 0  1  0 0 ]
	// [ 0 0 0  0  1 0 ]
	// [ 0 0 0  0  0 1 ]	
	//cv::setIdentity(kf.transitionMatrix);

	// Measure Matrix H
	// [ 1 0 0 0 0 0 ]
	// [ 0 1 0 0 0 0 ]
	// [ 0 0 0 0 1 0 ]
	// [ 0 0 0 0 0 1 ]
	
	kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kf.measurementMatrix.at<float>(0) = 1.0f;
	kf.measurementMatrix.at<float>(7) = 1.0f;
	kf.measurementMatrix.at<float>(16) = 1.0f;
	kf.measurementMatrix.at<float>(23) = 1.0f;

	
	// Process Noise Covariance Matrix Q
	// [ Ex   0   0     0     0    0  ]
	// [ 0    Ey  0     0     0    0  ]
	// [ 0    0   Ev_x  0     0    0  ]
	// [ 0    0   0     Ev_y  0    0  ]
	// [ 0    0   0     0     Ew   0  ]
	// [ 0    0   0     0     0    Eh ]
	cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-0));

	// Measures Noise Covariance Matrix R
	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(5 * 1e-0));
	cv::setIdentity(kf.errorCovPost, cv::Scalar::all(.2));
	cv::setIdentity(kf.errorCovPre, cv::Scalar::all(.1));
}

