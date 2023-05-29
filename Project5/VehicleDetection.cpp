#include "VehicleDetection.h"


///////////////////////////////////////////////////////////////////////////////////////////////////
//Public
VehicleDetection::VehicleDetection()
{
	//BG substract
	m_pMOG2 = cv::createBackgroundSubtractorMOG2(500, 100, false);
	if (m_pMOG2.dynamicCast<cv::BackgroundSubtractorMOG2>())
		m_pMOG2.dynamicCast<cv::BackgroundSubtractorMOG2>()->setShadowValue(0);//MOG2 approach




}

void VehicleDetection::processVideo(char* videoFilename)
{
	//create GUI windows
	cv::namedWindow("0.Frame");
	cv::namedWindow("1.FG Mask MOG 2");


	//create the capture object
	cv::VideoCapture capture(videoFilename);
	if (!capture.isOpened())
	{
		//error in opening the video input
		std::cerr << "Unable to open video file: " << videoFilename << std::endl;
		exit(EXIT_FAILURE);
	}

	bool blnFirstFrame = true;

	while ((char)m_keyboard != 'q' && (char)m_keyboard != 27)
	{
		//read the current frame
		if (!capture.read(m_frame))
		{
			std::cerr << "Unable to read next frame." << std::endl;
			std::cerr << "Exiting..." << std::endl;
			exit(EXIT_FAILURE);
		}

		double precTick = m_ticks;
		m_ticks = (double)cv::getTickCount();

		double dT = (m_ticks - precTick) / cv::getTickFrequency(); //seconds

		//update the background model
		m_pMOG2->apply(m_frame, m_fgMaskMOG2, -1); //// MOG Learning rate

		//get the frame number and write it on the current frame
		std::stringstream ss;
		ss << capture.get(cv::CAP_PROP_POS_FRAMES);
		std::string frameNumberString = ss.str();

		cv::rectangle(m_frame, cv::Point(10, 2), cv::Point(100, 20), vd::SCALAR_WHITE, -1);
		cv::putText(m_frame, frameNumberString.c_str(), cv::Point(15, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, vd::SCALAR_BLACK);


		//show the current frame and the fg mask
		imshow("0.Frame", m_frame);
		imshow("1.FG Mask MOG 2", m_fgMaskMOG2);

		//filter out the noise from the fg mask
		cv::dilate(m_fgMaskMOG2, m_fgMaskMOG2, vd::STRUCTURING_ELEMENT7x7);
		cv::erode(m_fgMaskMOG2, m_fgMaskMOG2, vd::STRUCTURING_ELEMENT7x7);

		cv::erode(m_fgMaskMOG2, m_fgMaskMOG2, vd::STRUCTURING_ELEMENT3x3);
		cv::dilate(m_fgMaskMOG2, m_fgMaskMOG2, vd::STRUCTURING_ELEMENT3x3);
		cv::erode(m_fgMaskMOG2, m_fgMaskMOG2, vd::STRUCTURING_ELEMENT7x7);
		cv::dilate(m_fgMaskMOG2, m_fgMaskMOG2, vd::STRUCTURING_ELEMENT7x7);;


		////Image Conturs
		std::vector<std::vector<cv::Point> > contours;
		cv::findContours(m_fgMaskMOG2, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		vd::drawAndShowContours(m_fgMaskMOG2.size(), contours, "2.imgContours");

		////Convex Hulls
		std::vector<std::vector<cv::Point> > convexHulls(contours.size());
		for (unsigned int i = 0; i < contours.size(); i++) {
			cv::convexHull(contours[i], convexHulls[i]);

		}
		vd::drawAndShowContours(m_fgMaskMOG2.size(), convexHulls, "3.imgConvexHulls");


		///Convex Hull defin blobs
		std::vector<Blob> currentFrameBlobs;
		for (auto& convexHull : convexHulls)
		{
			Blob possibleBlob(convexHull);
			/// Blob filtre param
			if (possibleBlob.currentBoundingRect.area() > 50 &&
				//possibleBlob.dblCurrentAspectRatio >= 0.2 &&
				//possibleBlob.dblCurrentAspectRatio <= 1.25 &&
				possibleBlob.currentBoundingRect.width > 15 &&
				possibleBlob.currentBoundingRect.height > 15 &&
				(cv::contourArea(possibleBlob.currentContour) / (double)possibleBlob.currentBoundingRect.area()) > 0.20)
			{
				currentFrameBlobs.push_back(possibleBlob);
			}
		}
		vd::drawAndShowContours(m_fgMaskMOG2.size(), currentFrameBlobs, "4.imgCurrentFrameBlobs");

		cv::Mat vehicles;
		m_frame.copyTo(vehicles);

		if (blnFirstFrame == true) {
			blnFirstFrame = false;
			for (auto& currentFrameBlob : currentFrameBlobs) {
				m_blobs.push_back(currentFrameBlob);
				m_vehicles.push_back(currentFrameBlob);
			}
		}
		else {
			matchCurrentFrameBlobsToExistingBlobs(m_blobs, currentFrameBlobs);
			matchCurrentFrameBlobsToExistingVehicles(m_vehicles, currentFrameBlobs, dT, vehicles);
		}



		// >>>>> Detection result
		cv::Mat res;
		m_frame.copyTo(res);


		for (size_t i = 0; i < m_blobs.size(); i++)
		{
			kfTraking(res, m_blobs[i], i, dT);
			float x = m_blobs[i].state.at<float>(0);
			float y = m_blobs[i].state.at<float>(1);
			m_blobs[i].centerPositions.push_back(cv::Point(x, y));
		}

		// blobs traking
		for (size_t i = 0; i < m_blobs.size(); i++)
		{
			if (m_blobs[i].blnStillBeingTracked) {
				cv::rectangle(res, m_blobs[i].currentBoundingRect, vd::SCALAR_GREEN, 1);

				cv::Point center;
				center.x = m_blobs[i].centerPositions.back().x;
				center.y = m_blobs[i].centerPositions.back().y;
				cv::circle(res, center, 2, CV_RGB(20, 150, 20), -1);

				double dblFontScale = m_blobs[i].dblCurrentDiagonalSize / 60.0;
				int intFontThickness = (int)std::round(dblFontScale * 2.0);

				cv::putText(res, std::to_string(i), cv::Point(center.x + 2, center.y - 2), cv::FONT_HERSHEY_SIMPLEX, dblFontScale, vd::SCALAR_WHITE, intFontThickness);
			}

		}



		cv::imshow("5.Tracking blobs", res);
		// <<<<< Detection result

		// Vehicle trakking

		for (size_t i = 0; i < m_vehicles.size(); i++)
		{
			if (m_vehicles[i].blnStillBeingTracked) {

				cv::rectangle(vehicles, m_vehicles[i].boundingRects.back(), vd::SCALAR_GREEN, 1);

				cv::Point center = m_vehicles[i].centerPositions.back();

				cv::circle(vehicles, center, 2, CV_RGB(20, 150, 20), -1);

				double dblFontScale = m_vehicles[i].currentDiagonalSize / 60.0;
				int intFontThickness = (int)std::round(dblFontScale * 2.0);

				cv::putText(vehicles, std::to_string(i), cv::Point(center.x + 2, center.y - 2), cv::FONT_HERSHEY_SIMPLEX, dblFontScale, vd::SCALAR_WHITE, intFontThickness);
			}
		}


		cv::imshow("6.Tracking vehicles", vehicles);

		//get the input from the keyboard
		m_keyboard = cv::waitKey(30);
	}
	//delete capture object
	capture.release();
	cv::destroyAllWindows();
}

//Private
void VehicleDetection::matchCurrentFrameBlobsToExistingBlobs(std::vector<Blob>& existingBlobs, std::vector<Blob>& currentFrameBlobs)
{
	for (auto &existingBlob : existingBlobs) {

		existingBlob.blnCurrentMatchFoundOrNewBlob = false;

		existingBlob.predictNextPosition();
	}

	for (auto &currentFrameBlob : currentFrameBlobs) {

		int intIndexOfLeastDistance = 0;
		double dblLeastDistance = 100000.0;

		for (unsigned int i = 0; i < existingBlobs.size(); i++) {
			if (existingBlobs[i].blnStillBeingTracked == true) {
				double dblDistance = vd::distanceBetweenPoints(currentFrameBlob.centerPositions.back(), existingBlobs[i].predictedNextPosition);

				if (dblDistance < dblLeastDistance) {
					dblLeastDistance = dblDistance;
					intIndexOfLeastDistance = i;
				}
			}
		}

		if (dblLeastDistance < currentFrameBlob.dblCurrentDiagonalSize * 1.30) {
			addBlobToExistingBlobs(currentFrameBlob, existingBlobs, intIndexOfLeastDistance);
		}
		else {
			addNewBlob(currentFrameBlob, existingBlobs);
		}

	}

	for (auto &existingBlob : existingBlobs) {

		if (existingBlob.blnCurrentMatchFoundOrNewBlob == false) {
			existingBlob.intNumOfConsecutiveFramesWithoutAMatch++;
		}

		if (existingBlob.intNumOfConsecutiveFramesWithoutAMatch >= 5) {
			existingBlob.blnStillBeingTracked = false;
		}

	}

}

void VehicleDetection::addBlobToExistingBlobs(Blob& currentFrameBlob, std::vector<Blob>& existingBlobs, int& intIndex)
{
	existingBlobs[intIndex].currentContour = currentFrameBlob.currentContour;
	existingBlobs[intIndex].currentBoundingRect = currentFrameBlob.currentBoundingRect;

	existingBlobs[intIndex].centerPositions.push_back(currentFrameBlob.centerPositions.back());

	existingBlobs[intIndex].dblCurrentDiagonalSize = currentFrameBlob.dblCurrentDiagonalSize;
	existingBlobs[intIndex].dblCurrentAspectRatio = currentFrameBlob.dblCurrentAspectRatio;

	existingBlobs[intIndex].blnStillBeingTracked = true;
	existingBlobs[intIndex].blnCurrentMatchFoundOrNewBlob = true;
}

void VehicleDetection::addNewBlob(Blob& currentFrameBlob, std::vector<Blob>& existingBlobs)
{
	currentFrameBlob.blnCurrentMatchFoundOrNewBlob = true;

	existingBlobs.push_back(currentFrameBlob);
}

void VehicleDetection::addBlobToExistingVehicles(Blob& currentFrameBlob, std::deque<Vehicle>& existingVehicles, size_t& intIndex)
{
	existingVehicles[intIndex].currentContour = currentFrameBlob.currentContour;
	existingVehicles[intIndex].currentDiagonalSize = currentFrameBlob.dblCurrentDiagonalSize;

	existingVehicles[intIndex].centerPositions.insert(currentFrameBlob.centerPositions.back());
	existingVehicles[intIndex].boundingRects.insert(currentFrameBlob.currentBoundingRect);

	existingVehicles[intIndex].intNumOfConsecutiveFramesWithoutAMatch = 0;
	existingVehicles[intIndex].blnStillBeingTracked = true;
	existingVehicles[intIndex].blnCurrentMatchFoundOrNewVehicle = true;
}

void VehicleDetection::addBlobToExistingVehicles(cv::Point& predPosition, cv::Rect predRect, std::deque<Vehicle>& existingVehicles, size_t& intIndex)
{
	//existingVehicles[intIndex].currentContour = currentFrameBlob.currentContour;
	double dblCurrentDiagonalSize = sqrt(pow(predRect.width, 2) + pow(predRect.height, 2));
	existingVehicles[intIndex].currentDiagonalSize = dblCurrentDiagonalSize;

	existingVehicles[intIndex].centerPositions.insert(predPosition);
	existingVehicles[intIndex].boundingRects.insert(predRect);

	existingVehicles[intIndex].blnCurrentMatchFoundOrNewVehicle = false;
	existingVehicles[intIndex].blnStillBeingTracked = true;
}

void VehicleDetection::addNewVehicle(Blob& currentFrameBlob, std::deque<Vehicle>& existingVehicles)
{

	existingVehicles.push_back(currentFrameBlob);
}

void VehicleDetection::matchCurrentFrameBlobsToExistingVehicles(std::deque<Vehicle>& existingVehicles, std::vector<Blob>& currentFrameBlobs,double& dt,cv::Mat& display )
{
	for (size_t i=0;i<existingVehicles.size();i++)
	{
		existingVehicles[i].blnCurrentMatchFoundOrNewVehicle = false;
		existingVehicles[i].predictNextPosition(dt, display,i);

	}

	for (auto& currentFrameBlob : currentFrameBlobs)
	{

		size_t intIndexOfLeastDistance = 0;
		double dblLeastDistance = 100000.0;
		double dArea= 1000000.0;
		int dx;
		int dy;

		for (unsigned int i = 0; i < existingVehicles.size(); i++)
		{
			if (existingVehicles[i].blnStillBeingTracked == true)
			{

				double dblDistance = vd::distanceBetweenPoints(currentFrameBlob.centerPositions.back(), existingVehicles[i].predPosition);

				if (dblDistance < dblLeastDistance)
				{
					dblLeastDistance = dblDistance;
					dArea = abs(currentFrameBlob.currentBoundingRect.area() - existingVehicles[i].predRect.area());
					intIndexOfLeastDistance = i;
				}
			}
		}

		if (dblLeastDistance < currentFrameBlob.dblCurrentDiagonalSize * 1.30)
		{
			double posibleAreaChange = currentFrameBlob.currentBoundingRect.area() * 0.50;
			if(dArea< posibleAreaChange)
			{
				addBlobToExistingVehicles(currentFrameBlob, existingVehicles, intIndexOfLeastDistance);
			}
			else
			{
				dx = abs(currentFrameBlob.centerPositions.back().x - existingVehicles[intIndexOfLeastDistance].predPosition.x);
				dy = abs(currentFrameBlob.centerPositions.back().y - existingVehicles[intIndexOfLeastDistance].predPosition.y);

				if (dx < dy) {
					existingVehicles[intIndexOfLeastDistance].predPosition.y = currentFrameBlob.centerPositions.back().y;
				}
				else {
					existingVehicles[intIndexOfLeastDistance].predPosition.x = currentFrameBlob.centerPositions.back().x;
				}

				addBlobToExistingVehicles(existingVehicles[intIndexOfLeastDistance].predPosition, existingVehicles[intIndexOfLeastDistance].predRect, existingVehicles, intIndexOfLeastDistance);
			}

		}
		else
		{
			addNewVehicle(currentFrameBlob, m_vehicles);
		}
	}


	for (auto& vehicle : existingVehicles) {

		if (vehicle.blnCurrentMatchFoundOrNewVehicle == false) {
			vehicle.intNumOfConsecutiveFramesWithoutAMatch++;
		}

		if (!vehicle.found && vehicle.intNumOfConsecutiveFramesWithoutAMatch>5) {
			vehicle.blnStillBeingTracked = false;
		}

	}

}


void VehicleDetection::kfTraking(cv::Mat &res,Blob &blob,int index,const double & dT)
{

	if (blob.found)
	{
		// >>>> Matrix A
		blob.kf.transitionMatrix.at<float>(2) = dT;
		blob.kf.transitionMatrix.at<float>(9) = dT;
		// <<<< Matrix A

		std::cout << "dT: " << dT << std::endl;

		blob.state = blob.kf.predict();
		//cout << "State post:" << endl << state << endl;

		cv::Rect predRect;
		predRect.width = blob.state.at<float>(4);
		predRect.height = blob.state.at<float>(5);
		predRect.x = blob.state.at<float>(0) - predRect.width / 2;
		predRect.y = blob.state.at<float>(1) - predRect.height / 2;

		cv::Point center;
		center.x = blob.state.at<float>(0);
		center.y = blob.state.at<float>(1);
		cv::circle(res, center, 2, vd::SCALAR_BLUE, -1);

		cv::rectangle(res, predRect, vd::SCALAR_BLUE, 2);
	}

	 //>>>>> Kalman Update
	if (!blob.blnStillBeingTracked)
	{
		blob.notFoundCount++;
		//cout << "notFoundCount:" << notFoundCount << endl;
		if (blob.notFoundCount >= 10)
		{
			blob.found = false;
		}
		else
			blob.kf.statePost = blob.state;
	}
	else
	{
			blob.notFoundCount = 0;

			blob.meas.at<float>(0) = blob.centerPositions.back().x;
			blob.meas.at<float>(1) = blob.centerPositions.back().y;
			blob.meas.at<float>(2) = (float)blob.currentBoundingRect.width;
			blob.meas.at<float>(3) = (float)blob.currentBoundingRect.height;


			if (!blob.found) // First detection!
			{
				// >>>> Initialization
				blob.kf.errorCovPre.at<float>(0) = 1; // px
				blob.kf.errorCovPre.at<float>(7) = 1; // px
				blob.kf.errorCovPre.at<float>(14) = 1;
				blob.kf.errorCovPre.at<float>(21) = 1;
				blob.kf.errorCovPre.at<float>(28) = 1; // px
				blob.kf.errorCovPre.at<float>(35) = 1; // px

				blob.state.at<float>(0) = blob.meas.at<float>(0);
				blob.state.at<float>(1) = blob.meas.at<float>(1);
				blob.state.at<float>(2) = 0;
				blob.state.at<float>(3) = 0;
				blob.state.at<float>(4) = blob.meas.at<float>(2);
				blob.state.at<float>(5) = blob.meas.at<float>(3);
				// <<<< Initialization

				blob.kf.statePost = blob.state;


				blob.found = true;
			}
			else 
			{
				// Kalman Correction
				blob.kf.correct(blob.meas); 
			}

		//cout << "Measure matrix:" << endl << meas << endl;
	}
	// <<<<< Kalman Update
	if (blob.blnStillBeingTracked) {
		cv::Mat state = blob.kf.statePost;

		cv::Rect improuvedPred;
		improuvedPred.width = blob.kf.statePost.at<float>(4);
		improuvedPred.height = blob.kf.statePost.at<float>(5);
		improuvedPred.x = blob.kf.statePost.at<float>(0) - improuvedPred.width / 2;
		improuvedPred.y = blob.kf.statePost.at<float>(1) - improuvedPred.height / 2;

		cv::Point center;
		center.x = blob.kf.statePost.at<float>(0);
		center.y = blob.kf.statePost.at<float>(1);
		cv::circle(res, center, 2, vd::SCALAR_RED, -1);

		cv::rectangle(res, improuvedPred, vd::SCALAR_RED, 2);
	
	}

	



}

