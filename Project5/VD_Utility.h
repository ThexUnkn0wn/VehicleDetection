#ifndef VD_UTILITY_H
#define VD_UTILITY_H

#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include "VehicleDetectionConstants.h"
#include "Blob.h"

namespace vd {

	static double distanceBetweenPoints(cv::Point point1, cv::Point point2)
	{
		int intX = abs(point1.x - point2.x);
		int intY = abs(point1.y - point2.y);

		return(sqrt(pow(intX, 2) + pow(intY, 2)));
	}


	static void drawAndShowContours(cv::Size imageSize, const std::vector<std::vector<cv::Point> >& contours, std::string strImageName)
	{
		cv::Mat image(imageSize, CV_8UC3, vd::SCALAR_BLACK);

		cv::drawContours(image, contours, -1, vd::SCALAR_WHITE, -1);

		cv::imshow(strImageName, image);
	}


	static void drawAndShowContours(cv::Size imageSize, const std::vector<Blob>& blobs, std::string strImageName)
	{
		cv::Mat image(imageSize, CV_8UC3, vd::SCALAR_BLACK);

		std::vector<std::vector<cv::Point> > contours;

		for (auto& blob : blobs) {
			if (blob.blnStillBeingTracked == true) {
				contours.push_back(blob.currentContour);
			}
		}

		cv::drawContours(image, contours, -1, vd::SCALAR_WHITE, -1);

		cv::imshow(strImageName, image);
	}



}


#endif