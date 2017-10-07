#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <iostream>
#include <stdlib.h>
#include <ctime>

/**
*/
static bool readCameraParameters(std::string filename, cv::Mat &camMatrix, cv::Mat &distCoeffs) {
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	return true;
}

/**
*/
static bool saveCameraParams(const std::string &filename, cv::Size imageSize, float aspectRatio, int flags,
	const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, double totalAvgErr) {
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	if (!fs.isOpened())
		return false;

	time_t tt;
	time(&tt);
	struct tm *t2 = localtime(&tt);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", t2);

	fs << "calibration_time" << buf;

	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;

	if (flags & cv::CALIB_FIX_ASPECT_RATIO) fs << "aspectRatio" << aspectRatio;

	if (flags != 0) {
		sprintf(buf, "flags: %s%s%s%s",
			flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
			flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
			flags & cv::CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
			flags & cv::CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
	}

	fs << "flags" << flags;

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;

	fs << "avg_reprojection_error" << totalAvgErr;

	return true;
}

/**
*/
static bool readDetectorParameters(std::string filename, cv:: Ptr<cv::aruco::DetectorParameters> &params) {
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
	fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
	fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
	fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
	fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
	fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
	fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
	fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
	fs["minDistanceToBorder"] >> params->minDistanceToBorder;
	fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
	fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
	fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
	fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
	fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
	fs["markerBorderBits"] >> params->markerBorderBits;
	fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
	fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
	fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
	fs["minOtsuStdDev"] >> params->minOtsuStdDev;
	fs["errorCorrectionRate"] >> params->errorCorrectionRate;
	return true;
}


int main(int argc, char *argv[]) {
	cv::Mat markerImage;
	std::string markerPath = "C:\\Users\\wben7\\Documents\\cvtutorial\\AR Boardgame\\markers\\marker";
	std::string imagePath = "C:\\Users\\wben7\\Pictures\\Camera Roll\\cal_2.jpg";

	cv::Mat inputImage;
	cv::Mat imageCopy;

	cv::Mat camMatrix;
	cv::Mat distCoeffs;

	int markersX = 10;
	int markersY = 5;
	float markerLength = 0.019177; // 1.9177 cm, .755 inches
	float markerSeparation = 0.0048; // 0.48 cm, 0.189 inches

	std::string cameraParamsPath = "C:\\Users\\wben7\\Documents\\cvtutorial\\AR Boardgame\\cameraParams.yml";

	bool readOk = readCameraParameters(cameraParamsPath, camMatrix, distCoeffs);
	if (!readOk) {
		std::cerr << "Invalid camera file" << std::endl;
		return 0;
	}

	float axisLength = 0.5f * ((float) std::min(markersX, markersY) * (markerLength + markerSeparation) +
		markerSeparation);

	cv::namedWindow("Image", cv::WINDOW_AUTOSIZE); // Create a window for display.

	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	/*
	for (int i = 0; i < 50; i++) {
		cv::aruco::drawMarker(dictionary, i, 500, markerImage, 1);
		cv::imshow("Marker", markerImage);
		std::stringstream markerFilePath;
		markerFilePath << markerPath;
		markerFilePath << i;
		markerFilePath << ".bmp";

		cv::imwrite(markerFilePath.str() , markerImage);
		cv::waitKey(30);
	}
	*/

	cv::Ptr<cv::aruco::GridBoard> gridBoard = cv::aruco::GridBoard::create(10, 5, .02f, .005f, dictionary);
	cv::Ptr<cv::aruco::Board> board = gridBoard.staticCast<cv::aruco::Board>();
	/* gridBoard->draw(cv::Size(2000, 1000), markerImage);
	cv::imshow("Marker", markerImage);
	cv::waitKey(0);
	std::stringstream markerFilePath;
	markerFilePath << markerPath;
	markerFilePath << "_grid";
	markerFilePath << ".bmp";
	cv::imwrite(markerFilePath.str(), markerImage);
	*/

	cv::Ptr<cv::aruco::CharucoBoard> chBoard = cv::aruco::CharucoBoard::create(5, 5, .02f, .015f, dictionary);

	cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();

	// Store single frame ids and corners
	std::vector<int> markerIds;
	std::vector<std::vector<cv::Point2f>> markerCorners;

	// Store all points for calibration

	// OpenCV video capture interface
	cv::VideoCapture cap;
	cap.open(1);

	while (true) {
		cv::Vec3d rvec, tvec;

		cap >> inputImage;
		cv::aruco::detectMarkers(inputImage, chBoard->dictionary, markerCorners, markerIds);

		// draw results
		inputImage.copyTo(imageCopy);

		if (markerIds.size() > 0) {
			cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
			// estimate board pose
			int markersOfBoardDetected = 0;
			if (markerIds.size() > 0) {
				markersOfBoardDetected =
					cv::aruco::estimatePoseBoard(markerCorners, markerIds, board, camMatrix, distCoeffs, rvec, tvec);
			}
				if (markersOfBoardDetected > 0)
					cv::aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);
		}

		cv::imshow("Image", imageCopy);

		char key = (char) cv::waitKey(1);

		if (key == 27)
			break;

		/* if (key == 'c' && markerIds.size() > 0) {
			std::cout << "Frame captured" << std::endl;
			allCorners.push_back(markerCorners);
			allIds.push_back(markerIds);
			imageSize = inputImage.size();
		}
		*/
	}
}

void chDiamonds() {
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	cv::Mat markerImage;
	cv::aruco::drawCharucoDiamond(dictionary, cv::Vec4i(0, 1, 2, 3), 200, 150, markerImage);
	cv::aruco::drawCharucoDiamond(dictionary, cv::Vec4i(4, 5, 6, 7), 200, 150, markerImage);
	cv::aruco::drawCharucoDiamond(dictionary, cv::Vec4i(8, 9, 10, 11), 200, 150, markerImage);
}

// http://docs.opencv.org/3.3.0/d4/d94/tutorial_camera_calibration.html
// TODO - Clean this up
void calibrateCamera(std::vector<std::vector<std::vector<cv::Point2f>>> allCorners, std::vector<std::vector<int>> allIds, cv::Ptr<cv::aruco::GridBoard> gridBoard) {

	cv::Ptr<cv::aruco::Board> board = gridBoard.staticCast<cv::aruco::Board>();
	int calibrationFlags = 0;
	float aspectRatio = 1.777777f;
	int markersX = 10;
	int markersY = 5;
	float markerLength = 0.019177; // 1.9177 cm, .755 inches
	float markerSeparation = 0.0048; // 0.48 cm, 0.189 inches
	std::string cameraParamsPath = "C:\\Users\\wben7\\Documents\\cvtutorial\\AR Boardgame\\cameraParams.yml";

	cv::Size imageSize;

	if (allIds.size() < 1) {
		std::cerr << "Not enough captures for calibration" << std::endl;
		return;
	}

	cv::Mat cameraMatrix, distCoeffs;
	std::vector<cv::Mat> rvecs, tvecs;
	double repError;

	if (calibrationFlags & cv::CALIB_FIX_ASPECT_RATIO) {
		cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
		cameraMatrix.at<double>(0, 0) = aspectRatio;
	}

	// prepare data for calibration
	std::vector<std::vector<cv::Point2f>> allCornersConcatenated;
	std::vector<int> allIdsConcatenated;
	std::vector<int> markerCounterPerFrame;
	markerCounterPerFrame.reserve(allCorners.size());
	for (unsigned int i = 0; i < allCorners.size(); i++) {
		markerCounterPerFrame.push_back((int)allCorners[i].size());
		for (unsigned int j = 0; j < allCorners[i].size(); j++) {
			allCornersConcatenated.push_back(allCorners[i][j]);
			allIdsConcatenated.push_back(allIds[i][j]);
		}
	}
	// calibrate camera
	repError = cv::aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
		markerCounterPerFrame, board, imageSize, cameraMatrix,
		distCoeffs, rvecs, tvecs, calibrationFlags);

	bool saveOk = saveCameraParams(cameraParamsPath, imageSize, aspectRatio, calibrationFlags, cameraMatrix,
		distCoeffs, repError);

	if (!saveOk) {
		std::cerr << "Cannot save output file" << std::endl;
		return;
	}

	std::cout << "Rep Error: " << repError << std::endl;
	std::cout << "Calibration saved to " << cameraParamsPath << std::endl;

	return;
}
// if at least one marker detected
/* if (markerIds.size() > 0) {
std::vector<cv::Point2f> charucoCorners;
std::vector<int> charucoIds;
cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, inputImage, chBoard, charucoCorners, charucoIds); // cameraMatrix, distCoeffs
if (charucoIds.size() > 0) {
cv::aruco::drawDetectedCornersCharuco(inputImage, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
/* cv::Vec3d rvec, tvec;
bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, chBoard, cameraMatrix, distortionParams, rvec, tvec);
// if charuco pose is valid
if (valid)
cv::aruco::drawAxis(inputImage, cameraMatrix, distortionParams, rvec, tvec, 0.1);
*/
// 	}
// }