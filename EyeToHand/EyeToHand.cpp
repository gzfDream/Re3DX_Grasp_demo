// EyeToHand.cpp: 机械臂手眼标定
//

#include "stdafx.h"
#include "HandEye.h"
#include "util_cal.h"
using namespace std;
using namespace cv;

void printMenu() {
	printf("Commands:\n");
	printf("  i  标定相机内参\n");
	printf("  g  获得标定图片\n");
	printf("  c  计算相机外参\n");
	printf("  p  标定计算\n");
	printf("  q  退出\n");
}


int main()
{
	// 存储相机内参标定图片
	std::string img_internal_file = ".\\data\\Images_internal";
	// 存储相机内参标定结果
	std::string cam_internal_file = ".\\data\\internal_reference.txt";

	// 存储标定时图片
	std::string cam_cal_file = ".\\data\\Images\\";
	// 标定时机械臂末端的位姿
	std::string robot_pose_file = ".\\data\\robot.txt";
	// 相机外参
	std::string cam_external_file = ".\\data\\cam_cal_robot.txt";//calHcam
	// 存储标定结果
	std::string result_file = ".\\data\\result.txt";
	printMenu();
	string line;
	bool running = true;
	while (running)
	{
		printf("please input the command >>> \n");
		std::getline(std::cin, line);
		switch (line[0]) {
		case 'i': {
			cout << "-*-*-*-*-标定相机内参-*-*-*-*-" << endl;
			cout << "-*-*-*-*-是否需要获得标定图片（输入1/0）-*-*-*-*-" << endl;
			int ok = 0;
			cin >> ok;
			
			if (ok) {
				getImageStream(img_internal_file + "\\");
			}

			std::vector<std::string> files;
			getAllImages(img_internal_file, files);
			internal_reference_calibration(files, cam_internal_file);
			break;
		}
		case 'g':
			getImageStream(cam_cal_file);
			break;
		case 'c': {
			cout << "开始获得相机外参" << endl;
			
			cout << "-*-*-*-*-是否重新计算相机内参（输入1/0）-*-*-*-*-" << endl;
			int ok = 0;
			cin >> ok;

			double cam[9] = { 2180.158012978382, 0, 481.9677402667883,
							0, 2186.775601369109, 415.3799781615131,
							0, 0, 1 };
			cv::Mat camD = cv::Mat(3, 3, CV_64FC1, cam);
			double distCoeffD[5] = { -0.3766702758870145, 0.0336750411497953, 0.005424035460253849, 0.003744876142253783, -0.5389160914426057 };
			Mat distortion_coefficients = cv::Mat(5, 1, CV_64FC1, distCoeffD);

			if (ok) {
				std::vector<std::string> files;
				getAllImages(cam_cal_file, files);
				internal_reference_calibration(files, cam_internal_file, camD, distortion_coefficients);
			}

			// 罗技
			/*double cam[9] = { 1488.908016986866, 0, 730.8401584004299,
				0, 1491.706043620342, 395.8201021249665,
				0, 0, 1 };
			cv::Mat camD = cv::Mat(3, 3, CV_64FC1, cam);

			double distCoeffD[5] = { 0.06436, 2.278, -0.00197799, 0.02, -22.37558 };*/

			external_reference_calibration(camD, distortion_coefficients, cam_cal_file, cam_external_file);
			break;
		}
		case 'p':
			cout << "获得标定结果" << endl;
			processes(robot_pose_file, cam_external_file, result_file);
			break;
		case 'q':
			running = false;
			break;
		default:
			printMenu();
			break;
		}
	}

    return 0;
}

