#ifndef MY_VEHICLE
#define MY_VEHICLE

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/video/tracking.hpp>

#include "VehicleDetectionConstants.h"
#include "CircularBuffer.h"
#include "Blob.h"


class Vehicle {
	// member variables ///////////////////////////////////////////////////////////////////////////
private:


	double m_diagonalSize;
	double m_aspectRatio;

	void initKF();

public:
	
	std::vector<cv::Point> currentContour;

	CircularBuffer<cv::Rect> boundingRects;
	CircularBuffer<cv::Point> centerPositions;

	cv::Rect predRect;
	cv::Point predPosition;
	//std::vector<cv::Point> predPositions;//new

	double currentDiagonalSize;

	bool blnCurrentMatchFoundOrNewVehicle;
	bool blnStillBeingTracked;
	int intNumOfConsecutiveFramesWithoutAMatch;
		

	// Kalman Filter
	cv::KalmanFilter kf;
	cv::Mat state;
	cv::Mat meas;
	cv::Mat procNoise;
	bool found = false;
	int notFoundCount = 0;


	// function prototypes ////////////////////////////////////////////////////////////////////////
	Vehicle(std::vector<cv::Point> _contour);
	Vehicle(Blob blob);
	void predictNextPosition(double& dt, cv::Mat& display, size_t& index);

};

#endif    // MY_VEHICLE

