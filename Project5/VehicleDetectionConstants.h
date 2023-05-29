#ifndef VD_CONSTANTS_H
#define VD_CONSTANTS_H
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>


//  File Name : LibConstants.hpp    Purpose : Global Constants for Lib Utils
namespace vd
{
	// >>>>> Color to be tracked
#define MIN_H_BLUE 200
#define MAX_H_BLUE 300
// <<<<< Color to be tracked
	//////collors
	const cv::Scalar SCALAR_BLACK = cv::Scalar(0.0, 0.0, 0.0);					
	const cv::Scalar SCALAR_WHITE = cv::Scalar(255.0, 255.0, 255.0);
	const cv::Scalar SCALAR_BLUE = cv::Scalar(255.0, 0.0, 0.0);
	const cv::Scalar SCALAR_GREEN = cv::Scalar(0.0, 200.0, 0.0);
	const cv::Scalar SCALAR_RED = cv::Scalar(0.0, 0.0, 255.0);

	const cv::Mat STRUCTURING_ELEMENT2x2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
	const cv::Mat STRUCTURING_ELEMENT3x3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	const cv::Mat STRUCTURING_ELEMENT4x4 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4));
	const cv::Mat STRUCTURING_ELEMENT5x5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	const cv::Mat STRUCTURING_ELEMENT7x7 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
	const cv::Mat STRUCTURING_ELEMENT9x9 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));


}
#endif