#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <vector>
#include <numeric>

#include "RSocket.h"
#include "re3dx.h"
#include "cvrender.h"
// #include "util.h"
#include "TestTracking.h"
using namespace cv; 
using namespace std;
#pragma comment(lib,"re3dx.lib")

int main()
{
	TestTracking();
	//re3d::Model::build("C:\\Softwares\\3ds-model\\ambulance\\ambulance_r.3ds", "C:\\Softwares\\3ds-model\\ambulance\\ambulance_r.idx122.");
	return 0;
}


//int main()
//{
//	// socket初始化
//	const char* SendBuffer;// [MAX_PATH];
//	char* acceptBuffer;
//	RSocket Rsocket;
//	SOCKET sclient;
//	bool connect_or = false;
//	if (connect_or)
//		sclient = Rsocket.InitSocket(5000, "192.168.143.101");
//
//	//加载模型
//	std::string modelNames[] = {
//		"bottle2_r","xiaoMing_r","blueMoon_r","HardDisk_r","CLemon_r","DaLong_r"
//	};
//	constexpr int nModels = sizeof(modelNames) / sizeof(modelNames[0]);
//	re3d::Model models[nModels];
//	for (int i = 0; i < nModels; ++i)
//		models[i].load(("./data/" + modelNames[i] + ".idx121").c_str());
//
//	// 设置相机
//	Size imgSize(1280, 720);
//	cv::VideoCapture cap;
//#if 1
//	cap.open(CAP_DSHOW);
//	cap.set(CAP_PROP_FRAME_WIDTH, imgSize.width);
//	cap.set(CAP_PROP_FRAME_HEIGHT, imgSize.height);
//#else
//	cap.open("./data/bottle2-2.avi");
//#endif
//	imgSize = Size(int(cap.get(CAP_PROP_FRAME_WIDTH) + 0.5), int(cap.get(CAP_PROP_FRAME_HEIGHT) + 0.5));
//	printf("image-size=(%d,%d)\n", imgSize.width, imgSize.height);
//
//	// 相机参数
//	double cam[9] = { 2179.167509616685, 0, 484.8078908200978,
//						0, 2168.967242704098, 418.4348013616557,
//						0, 0, 1 };
//	cv::Matx33f K = cv::Mat(3, 3, CV_64FC1, cam);
//
//	re3d::Tracker tracker;
//	tracker.setModels(models, nModels, K);
//
//	int iframe = 0; //
//	std::string str_buf; //
//
//	Eigen::Vector4f position(0., 0., 0., 1.); //初始化抓取点
//
//	// 判断是否移动
//	int frame_NO = 0;
//	int frame_num = 5;
//	bool moving = true;
//	vector<cv::Vec3f> movement(frame_num, cv::Vec3f(0.0, 0.0, 0.0));
//	int static_num = 0;
//	bool send_or = true; // 是否发送指令
//
//	Eigen::Matrix4f transMat;//机械手变换矩阵
//	Eigen::Matrix3f transR;//机械手旋转矩阵
//	Eigen::Vector4f transT;//机械手平移向量
//	Eigen::Matrix4f robHobj;//物体与机械臂的相对位置
//
//	Eigen::Quaternionf quat(0.5, -0.5, -0.5, -0.5);
//
//	// 处理每一帧
//	Mat img;
//	while (cap.read(img))
//	{
//		time_t beg = clock();
//
//		re3d::Tracker::Result r;
//		int n = tracker.tracking(img, &r);
//
//		//printf("confidence=%.2f | fps=%.2lf\r", r.trackingConfidence, CLOCKS_PER_SEC / double(clock() - beg));
//
//		//draw the object contour if tracking confidence is high
//		if (n != 0 && r.trackingConfidence > 0.5)
//		{
//			if (r.contourPoints)
//			{
//				std::vector<std::vector<cv::Point>> cpts(1);
//				cpts[0].resize(r.nContourPoints);
//				memcpy(&cpts[0][0], r.contourPoints, sizeof(cv::Point)*r.nContourPoints);
//				cv::polylines(img, cpts, true, cv::Scalar(255, 0, 0), 2);
//
//				cv::Vec3f camT = getCameraT(r.rvec, r.tvec, models[r.objectID].getCenter());
//				char str[32];
//				sprintf(str, "dist=%.0fmm", norm(camT));
//				cv::putText(img, str, cv::Point(30, 30), FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0), 2, CV_AA);
//
//				cv::Vec3f camTrans(camT[0] / 1000, -camT[1] / 1000, -camT[2] / 1000);
//
//				frame_NO = frame_NO % frame_num;
//				movement[frame_NO] = camTrans;
//				frame_NO = frame_NO + 1;
//
//				moving = is_Moving(frame_num, movement, static_num);
//
//				if (!connect_or || !moving) {
//					robHobj = get_robHobj(r.rvec, r.tvec);
//					switch (r.objectID)
//					{
//					case 0:
//						position[0] = camTrans[0];
//						position[1] = camTrans[1];
//						position[2] = camTrans[2];
//						position = getPosition(position);
//						quat = Eigen::Quaternionf(0.5, -0.5, -0.5, -0.5);
//						break;
//					case 1:
//
//						break;
//					case 2:
//						getBlueMoonGraspPose(robHobj, quat, position);
//						break;
//					case 3:
//						position[0] = camTrans[0];
//						position[1] = camTrans[1];
//						position[2] = camTrans[2];
//						position = getPosition(position);
//						quat = Eigen::Quaternionf(0.5, -0.5, -0.5, -0.5);
//						break;
//					case 4:
//						getCLemonGraspPose(robHobj, quat);
//						position[0] = camTrans[0];
//						position[1] = camTrans[1];
//						position[2] = camTrans[2];
//						position = getPosition(position);
//						break;
//					case 5:
//						break;
//					default:
//						break;
//					}
//
//					if (connect_or && send_or) {
//						str_buf = "p " + std::to_string(position[0]) + " " + std::to_string(position[1]) + " " + std::to_string(position[2])
//							+ " " + std::to_string(quat.w()) + " " + std::to_string(quat.x()) + " " + std::to_string(quat.y()) + " " + std::to_string(quat.z());
//
//						cout << "物体静止，机械臂开始运动，运动指令：" << str_buf << endl;
//
//						SendBuffer = str_buf.c_str();
//						Rsocket.Rsend(sclient, SendBuffer);
//
//						send_or = false;
//					}
//					if (!connect_or) {
//						printf("模型：%s, 运动指令：%.2f, %.2f, %.2f; 四元数：%.2f, %.2f, %.2f, %.2f  | fps=%.2lf\r",
//							modelNames[r.objectID], position[0], position[1], position[2], quat.w(), quat.x(), quat.y(), quat.z(),
//							CLOCKS_PER_SEC / double(clock() - beg));
//					}
//				}
//				else {
//					if (connect_or && !send_or) {
//						cout << "物体开始运动，机械臂停止运动！" << endl;
//						SendBuffer = "s";
//						Rsocket.Rsend(sclient, SendBuffer);
//						send_or = true;
//					}
//				}
//			}
//		}
//
//		cv::imshow("result", img);
//
//		if (cv::waitKey(1) == 27 || cv::waitKey(1) == 'q') {
//			cv::destroyAllWindows();
//
//			if (connect_or) {
//				SendBuffer = "q";
//				Rsocket.Rsend(sclient, SendBuffer);
//			}
//			break;
//		}
//	}
//	if (connect_or)
//		closesocket(sclient);
//
//	std::system("pause");
//	return 0;
//}