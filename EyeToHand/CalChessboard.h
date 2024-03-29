#pragma once
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
class CalChessboard
{
public:
	CalChessboard(cv::Mat, cv::Mat distCoeffD, double &markerRealSize);
	~CalChessboard();

	void calprocess(std::string &imgpath, std::string &calibFile, int x);
	void calprocess(std::string &imgpath, std::string &calibFile);

private:

	void corner_detection(std::string &imgpath, int x);
	void calibration();
	void save_transMatrix(std::string &calibFile);
	cv::Mat process_transMatrix();

	cv::Matx33f camera_matrix;
	cv::Matx<float, 5, 1> distortion_coefficients;

	std::vector<cv::Point3f> m_markerCorners3d;
	std::vector<cv::Point2f> m_markerCorners2d;

	cv::Mat image_color;
	//cv::Mat Rvec;
	//cv::Mat_<float> Tvec;
	cv::Mat rvec, tvec;
};

