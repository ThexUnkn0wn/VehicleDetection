// Blob.h

#ifndef MY_BLOB
#define MY_BLOB

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/video/tracking.hpp>

///////////////////////////////////////////////////////////////////////////////////////////////////
class Blob {
public:
	// member variables ///////////////////////////////////////////////////////////////////////////
	std::vector<cv::Point> currentContour;

	cv::Rect currentBoundingRect;
	cv::Rect predBoundingRect;//new

	std::vector<cv::Point> centerPositions;
	//std::vector<cv::Point> predPositions;//new

	double dblCurrentDiagonalSize;
	double dblCurrentAspectRatio;

	bool blnCurrentMatchFoundOrNewBlob;

	bool blnStillBeingTracked;

	int intNumOfConsecutiveFramesWithoutAMatch;

	cv::Point predictedNextPosition;

	//Kalman Filter
	cv::KalmanFilter kf;
	cv::Mat state;
	cv::Mat meas;
	cv::Mat procNoise;
	bool found = false;
	int notFoundCount = 0;


	// function prototypes ////////////////////////////////////////////////////////////////////////
	Blob(std::vector<cv::Point> _contour);
	void predictNextPosition(void);

};

#endif    // MY_BLOB
