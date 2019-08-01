#pragma once

#include "re3dx.h"
#include "cvrender.h"
#include "util.h"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
using namespace cv;
#include<time.h>
#include<list>

Vec3f getCameraT(const Vec3f &rvec, const Vec3f &tvec, const Vec3f &modelCenter)
{
	Matx44f mModelView = cvrm::fromRT(rvec, tvec);

	return modelCenter*mModelView;
}

class Actions
{
public:
	struct Pose
	{
		time_t time;
		Vec3f  rvec;
		Vec3f  tvec;
		Vec3f  camT;
		float  confidence;
	};
private:
	int              _objID = -1;
	int              _nOtherObjs = 0;
	std::list<Pose>  _poseList;
	std::list<Pose>  _staticList;
private:
	void _setObj(int objID)
	{
		_objID = objID;
		_poseList.clear();
	}
	int _detectState(Pose &bestPose, int nFrames = 11, float maxDiffOfTranslation = 30, float staticThreshold = 0.8f, float movingThreshold = 0.2f)
	{
		if (_poseList.size() >= nFrames)
		{
			Vec3f tMean(0, 0, 0);
			auto itr = _poseList.rbegin();
			int i = 0;
			for (; i < nFrames; ++itr, ++i)
			{
				tMean += itr->camT;
			}
			tMean *= 1.0f / nFrames;

			int nStatic = 0;
			itr = _poseList.rbegin();
			i = 0;
			maxDiffOfTranslation *= maxDiffOfTranslation;

			float poseConfidence = 0;
			for (; i < nFrames; ++itr, ++i)
			{
				Vec3f dv = itr->camT - tMean;
				if (dv.dot(dv) < maxDiffOfTranslation)
				{
					if (itr->confidence > poseConfidence)
					{
						bestPose = *itr;
						poseConfidence = itr->confidence;
					}
					++nStatic;
				}
			}
			float r = float(nStatic) / nFrames;
			return r > staticThreshold ? 1 : r < movingThreshold ? -1 : 0;
		}
		return 0;
	}
public:
	int  pushResult(const re3d::Tracker::Result &r, Vec3f camT, float minDistanceBetweenStatics = 30)
	{
		if (r.trackingConfidence < 0.5)
			return 0;

		//switch objects
		if (r.objectID != _objID)
		{
			++_nOtherObjs;
			if (_nOtherObjs > 5)
				_setObj(r.objectID);
		}
		else
			_nOtherObjs = 0;

		time_t curTime = clock();
		//if tracking has lost for sometime
		if (!_poseList.empty() && int(curTime - _poseList.back().time) > int(0.5*CLOCKS_PER_SEC))
			_poseList.clear();

		_poseList.push_back({ curTime,r.rvec,r.tvec, camT, r.trackingConfidence });
		Pose bestPose;
		int state = this->_detectState(bestPose);

		if (state != 0)
		{
			float distToPrevStatic = 0;
			if (!_staticList.empty())
			{
				Point3f dv = bestPose.camT - _staticList.front().camT;
				distToPrevStatic = sqrt(dv.dot(dv));

				if (distToPrevStatic > minDistanceBetweenStatics) //mm
				{
					_staticList.clear();
					state = -1;
				}
				else
					state = 1;
			}
			if (state > 0)
			{
				_staticList.push_back(bestPose);
				state = (int)_staticList.size();
			}
		}

		return state;
	}
	//get current static pose, only can be called when pushResult return 1 or 0
	Pose getCurrentStatic(bool getFirst = true) const
	{
		CV_Assert(!_staticList.empty());

		return _staticList.empty() ? Pose() : getFirst ? _staticList.front() : _staticList.back();
	}
};


void TestTracking(std::string dataDir = "")
{
	//re3d::Model::build((dataDir + R"(\re3d\3ds-model\ambulance\ambulance.3ds)").c_str(), "./data/ambulance.idx122");

	// socket初始化
	const char* SendBuffer;// [MAX_PATH];
	char* acceptBuffer;
	RSocket Rsocket;
	SOCKET sclient;
	bool connect_or = false;
	if (connect_or)
		sclient = Rsocket.InitSocket(5000, "192.168.143.101");

	//the precomputed model file
	std::string modelNames[] = {
		"bottle2_r","xiaoMing_r","blueMoon_r","HardDisk_r","CLemon_r","DaLong_r", "ambulance_r"
	};
	constexpr int nModels = sizeof(modelNames) / sizeof(modelNames[0]);

	re3d::Model models[nModels];

	for (int i = 0; i < nModels;++i)
		models[i].load(("./data/"+modelNames[i]+".idx122").c_str());

	Size imgSize(1280, 720);

	// 相机参数
	double cam[9] = { 2173.451736647567, 0, 480.1070719645299,
				0, 2176.984226568406, 407.0237292414436,
				0, 0, 1 };
	//cv::Matx33f K = cv::Mat(3, 3, CV_64FC1, cam);
	cv::Matx33f K = cvrm::defaultK(imgSize, 1.2);

	cv::VideoCapture cap;
#if 1
	cap.open(0+CAP_DSHOW);
	cap.set(CAP_PROP_FRAME_WIDTH, imgSize.width);
	cap.set(CAP_PROP_FRAME_HEIGHT, imgSize.height);
#else
	cap.open("../TestX/bottle2-5.avi");
#endif
	imgSize = Size(int(cap.get(CAP_PROP_FRAME_WIDTH) + 0.5), int(cap.get(CAP_PROP_FRAME_HEIGHT) + 0.5));

	std::printf("image-size=(%d,%d)\n", imgSize.width, imgSize.height);

	re3d::Tracker tracker;
	tracker.setModels(models, nModels, K);
	Actions action;

	int iframe = 0;
	std::string str_buf; //
	Eigen::Vector4f position(0., 0., 0., 1.); //初始化抓取点
	
	// 判断是否移动
	int frame_NO = 0;
	int frame_num = 10;
	bool moving = true;
	std::vector<cv::Vec3f> movement(frame_num, cv::Vec3f(0.0, 0.0, 0.0));
	int top_frame = 0;
	int static_num = 0;
	bool send_or = true; // 是否发送指令
	cv::Vec3f camTrans;

	Eigen::Matrix4f transMat;//机械手变换矩阵
	Eigen::Matrix3f transR;//机械手旋转矩阵
	Eigen::Vector4f transT;//机械手平移向量
	Eigen::Matrix4f robHobj;//物体与机械臂的相对位置

	Eigen::Quaternionf quat(0.5, -0.5, -0.5, -0.5);


	Mat img;
	while (cap.read(img))
	{
		time_t beg = clock();

		re3d::Tracker::Result vr[nModels];
		int n = tracker.tracking(img, vr);
	
		for (int i = 0; i<n; ++i) {
			auto &r = vr[i];
			if (r.trackingConfidence > 0.5 && r.contourPoints)
			{
				std::vector<std::vector<cv::Point> > cpts(1);
				cpts[0].resize(r.nContourPoints);
				memcpy(&cpts[0][0], r.contourPoints, sizeof(cv::Point)*r.nContourPoints);
				cv::polylines(img, cpts, true, Scalar(255, 0, 0), 2);

				Vec3f camT = getCameraT(r.rvec, r.tvec, models[r.objectID].getCenter());

#if 1				
				int s = action.pushResult(r, camT);

				char str[32];

				const int minStatics = 20;
				//sprintf(str, "dv=(%.0f,%.0f,%.0f), dist=%.0f", camT[0], camT[1], camT[2],norm(camT));
				//static int nstatic = 0;
				static Actions::Pose curStatic;
				/*if (s == minStatics)
				{
					curStatic = action.getCurrentStatic();
					printf("curStatic=(%.0f,%.0f,%.0f)\n", curStatic.camT[0], curStatic.camT[1], curStatic.camT[2]);
					++nstatic;
				}
				*/
				sprintf(str, "%s-%d", s > minStatics ? "static" : "moving");

				cv::putText(img, str, Point(30, 30), FONT_HERSHEY_PLAIN, 2, Scalar(255, 0, 0), 2, CV_AA);

				if (s < minStatics)
					moving = true;
				else if((s >= minStatics && !connect_or)) {
					curStatic = action.getCurrentStatic();
					moving = false;
				}
#endif
				//cv::Vec3f camTrans(camT[0] / 1000, -camT[1] / 1000, -camT[2] / 1000);
				//frame_NO = frame_NO % frame_num;
				//movement[frame_NO] = camTrans;
				//frame_NO = frame_NO + 1;
				//top_frame = frame_NO % frame_num;

				//moving = is_Moving(frame_num, movement, top_frame, static_num);

				if (!connect_or || !moving) {
					if(!connect_or)
						camTrans << curStatic.camT[0] / 1000, -curStatic.camT[1] / 1000, -curStatic.camT[2] / 1000;
					else
						camTrans << camT[0] / 1000, -camT[1] / 1000, -camT[2] / 1000;

					robHobj = get_robHobj(r.rvec, r.tvec);
					
					switch (r.objectID)
					{
					case 0: // "bottle2_r",
						position[0] = camTrans[0];
						position[1] = camTrans[1];
						position[2] = camTrans[2];
						position = getPosition(position);
						quat = Eigen::Quaternionf(0.5, -0.5, -0.5, -0.5);
						break;
					case 1: //"xiaoMing_r"
						getCLemonGraspPose(robHobj, quat);
						position[0] = camTrans[0];
						position[1] = camTrans[1];
						position[2] = camTrans[2];
						position = getPosition(position);
						break;
					case 2: // "blueMoon_r"
						getBlueMoonGraspPose(robHobj, quat, position);
						break;
					case 3: // "HardDisk_r"
						position[0] = camTrans[0];
						position[1] = camTrans[1];
						position[2] = camTrans[2];
						position = getPosition(position);
						quat = Eigen::Quaternionf(0.5, -0.5, -0.5, -0.5);
						break;
					case 4: // "CLemon_r"
						getCLemonGraspPose(robHobj, quat);
						position[0] = camTrans[0];
						position[1] = camTrans[1];
						position[2] = camTrans[2];
						position = getPosition(position);
						break;
					case 5: // "DaLong_r"
						getDaLongGraspPose(robHobj, quat, position);
						break;
					default:
						break;
					}

					if (connect_or && send_or) {
						str_buf = "p " + std::to_string(position[0]) + " " + std::to_string(position[1]) + " " + std::to_string(position[2])
							+ " " + std::to_string(quat.w()) + " " + std::to_string(quat.x()) + " " + std::to_string(quat.y()) + " " + std::to_string(quat.z());

						std::cout << "物体静止，机械臂开始运动，运动指令：" << str_buf << std::endl;

						SendBuffer = str_buf.c_str();
						Rsocket.Rsend(sclient, SendBuffer);

						send_or = false;
					}
					if (!connect_or) {
						printf("模型：%s, 运动指令：%.2f, %.2f, %.2f; 四元数：%.2f, %.2f, %.2f, %.2f  | fps=%.2lf\r",
							modelNames[r.objectID], position[0], position[1], position[2], quat.w(), quat.x(), quat.y(), quat.z(),
							CLOCKS_PER_SEC / double(clock() - beg));
					}
				}
				else {
					if (connect_or && !send_or) {
						std::cout << "物体开始运动，机械臂停止运动！" << std::endl;
						SendBuffer = "s";
						Rsocket.Rsend(sclient, SendBuffer);
						send_or = true;
					}
				}
			}
		}
		cv::flip(img, img, 0);
		imshow("result", img); 

		if (cv::waitKey(1) == 27 || cv::waitKey(1) == 'q') {
			cv::destroyAllWindows();

			if (connect_or) {
				SendBuffer = "q";
				Rsocket.Rsend(sclient, SendBuffer);
			}
			break;
		}
	}

	if (connect_or)
		closesocket(sclient);
}



