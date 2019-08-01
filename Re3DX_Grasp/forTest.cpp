#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <vector>
#include <numeric>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include"re3dx.h"
#include"cvrender.h"


using namespace cv;
using namespace std;
#pragma comment(lib,"re3dx.lib")

// 读取文件到mat
cv::Matx44f readf_(string str) {
	cv::Matx44f mat;

	ifstream  file(str);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			file >> mat(i, j);
		}
	}
	file.close();

	return mat;
}

// 矩阵变换
cv::Mat process_transMatrix_(cv::Vec3f rvec, cv::Vec3f tvec) {
	cv::Mat trans_mat(4, 4, CV_32F), rotM;

	Rodrigues(rvec, rotM);

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			trans_mat.at<float>(i, j) = rotM.at<double>(i, j);
		}
	}
	trans_mat.at<float>(0, 3) = tvec[0] / 1000;
	trans_mat.at<float>(1, 3) = tvec[1] / 1000;
	trans_mat.at<float>(2, 3) = tvec[2] / 1000;

	trans_mat.at<float>(3, 0) = 0;
	trans_mat.at<float>(3, 1) = 0;
	trans_mat.at<float>(3, 2) = 0;
	trans_mat.at<float>(3, 3) = 1;

	//camHcal->calHcam
	trans_mat = trans_mat.inv();

	cv::Matx44f robHcam = readf_("..//EyeToHand//data//result.txt");

	return trans_mat * robHcam;
}

// 获得机械臂坐标系下的位置
cv::Vec4f getPosition_(cv::Vec3f tvec) {
	cv::Vec4f position = cv::Vec4f(tvec[0], tvec[1], tvec[2], 1.);
	cv::Matx44f mat = readf_("../EyeToHand/data/result.txt");
	position = mat * position;

	return position;
}

//计算两点之间的距离
double distance_(cv::Vec4f v1, cv::Vec4f v2) {
	return sqrtf(powf((v1[0] - v2[0]), 2) + powf((v1[1] - v2[1]), 2) + powf((v1[2] - v2[2]), 2));
}

Vec3f getCameraT_(const Vec3f &rvec, const Vec3f &tvec, const Vec3f &modelCenter)
{
	Matx44f mModelView = cvrm::fromRT(rvec, tvec);

	// std::cout << "Center: "<< modelCenter << std::endl;
	//std::cout << "mat: " << mModelView << std::endl;

	return modelCenter * mModelView;
}

// 得到物体相对于机械臂实时的变换矩阵
cv::Matx44f getTransMatrix_robHobj(const cv::Vec3f &rvec, const cv::Vec3f &tvec) {
	cv::Vec3f tvec_mm = tvec / 1000;

	Matx44f mModelView = cvrm::fromRT(rvec, tvec_mm);
	//std::cout << "mModelView= " << mModelView.t() << endl;

	cv::Matx44f x_90;
	x_90 << 1.,     0.,     0.,     0.,
			0.,     0.,     1.,     0.,
			0.,    -1.,     0.,     0.,
			0.,     0.,     0.,     1.;
	cv::Matx44f x180;
	x180 << 1,  0,  0,  0,
			0, -1,  0,  0,
			0,  0, -1,  0,
			0,  0,  0,  1;

	cv::Matx44f camHobj = x180 * mModelView.t();
	//std::cout << "camHobj= " << camHobj << endl;

	cv::Matx44f robHcam = readf_("..//EyeToHand//data//result.txt");

	cv::Matx44f robHobj = robHcam * camHobj * x_90;

	return robHobj;
}


//【3】抓取蓝月亮  机械手z轴与物体y轴重合
cv::Matx44f graspTrans_2(cv::Matx44f robHobj) {
	//分两种情况
	//1.把手朝外
	cv::Matx44f x90;
	x90 << 1, 0, 0, 0,
		0, 0, 1, 0,
		0, -1, 0, 0,
		0, 0, 0, 1;
	cv::Matx44f z180;
	z180 <<
		-1, 0, 0, 0,
		0, -1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;;
	return robHobj * x90 * z180;
	//2.把手朝内（机器人方向）
	//return robHobj*x190
}

//计算物体Z轴与基座标系的夹角
Eigen::Vector3f angle(Eigen::Vector3f p) {
	Eigen::Vector3f x(1,0,0), y(0,1,0), z(0,0,1);
	Eigen::Vector3f res;
	res.x() = acos(p.dot(x) / (x.norm()*p.norm()));
	res.y() = acos(p.dot(y) / (y.norm()*p.norm()));
	res.z() = acos(p.dot(z) / (z.norm()*p.norm()));

	return res;
}


int main_()
{

	/*CVRModel cvr_model;
	cvr_model.load("C:\\Users\\gzf\\OneDrive\\code\\RE3D_grasp_demo\\modes-618\\bottle2\\bottle2_r.obj");
	cvr_model.saveAs("C:\\Users\\gzf\\OneDrive\\code\\RE3D_grasp_demo\\modes-618\\bottle2\\bottle2_r.3ds");

	re3d::Model::build("C:\\Users\\gzf\\OneDrive\\code\\RE3D_grasp_demo\\modes-618\\bottle2\\bottle2_r.3ds",
		"C:\\Users\\gzf\\OneDrive\\code\\RE3D_grasp_demo\\modes-618\\bottle2\\bottle2_r.idx121");*/

	// 抓取点初始化
	std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> blueMoon_points;
	blueMoon_points.push_back(Eigen::Vector4f(0., -0.010, 0.120, 1));//顶
	blueMoon_points.push_back(Eigen::Vector4f(0., -0.020, -0.10, 1));//底
	blueMoon_points.push_back(Eigen::Vector4f(0., -0.080, -0.020, 1));//前

	std::string modelNames[] = {
		"bottle2_r","xiaoMing_r","blueMoon_r","HardDisk_r","CLemon_r", "DaLong_r"
	};
	constexpr int nModels = sizeof(modelNames) / sizeof(modelNames[0]);

	re3d::Model models[nModels];

	for (int i = 0; i < nModels; ++i)
		models[i].load(("./data/" + modelNames[i] + ".idx121").c_str());


	Size imgSize(1280, 720);

	cv::VideoCapture cap;
#if 1
	cap.open(CAP_DSHOW);
	cap.set(CAP_PROP_FRAME_WIDTH, imgSize.width);
	cap.set(CAP_PROP_FRAME_HEIGHT, imgSize.height);
#else
	cap.open("./data/bottle2-2.avi");
#endif
	imgSize = Size(int(cap.get(CAP_PROP_FRAME_WIDTH) + 0.5), int(cap.get(CAP_PROP_FRAME_HEIGHT) + 0.5));
	printf("image-size=(%d,%d)\n", imgSize.width, imgSize.height);

	double cam[9] = { 2179.167509616685, 0, 484.8078908200978,
						0, 2168.967242704098, 418.4348013616557,
						0, 0, 1 };
	cv::Matx33f K = cv::Mat(3, 3, CV_64FC1, cam);

	re3d::Tracker tracker;
	tracker.setModels(models, nModels, K);

	Mat img;
	int iframe = 0;
	std::string str_buf;
	cv::Vec4f point = cv::Vec4f(0., 0., 0., 1);
	Eigen::Vector4f position(0., 0., 0., 1.);

	// 判断是否移动
	int frame_NO = 0;
	int frame_num = 10;
	double moving = 0;
	vector<double> movement(frame_num, 0.0);
	bool cur = false; // 当前状态
	bool prev = false; // 前一个状态
	bool send_or = true; // 是否发送指令

	while (cap.read(img))
	{
		time_t beg = clock();

		re3d::Tracker::Result r;
		int n = tracker.tracking(img, &r);

		//printf("confidence=%.2f | fps=%.2lf\r", r.trackingConfidence, CLOCKS_PER_SEC / double(clock() - beg));

		//draw the object contour if tracking confidence is high
		if (n != 0 && r.trackingConfidence > 0.7)
		{
			if (r.contourPoints)
			{
				std::vector<std::vector<cv::Point>> cpts(1);
				cpts[0].resize(r.nContourPoints);
				memcpy(&cpts[0][0], r.contourPoints, sizeof(cv::Point)*r.nContourPoints);
				cv::polylines(img, cpts, true, Scalar(255, 0, 0), 2);

				Vec3f camT = getCameraT_(r.rvec, r.tvec, models[r.objectID].getCenter());
				char str[32];
				sprintf(str, "dist=%.0fmm", norm(camT));
				cv::putText(img, str, Point(30, 30), FONT_HERSHEY_PLAIN, 2, Scalar(255, 0, 0), 2, CV_AA);

				Vec4f camTrans;
				camTrans[0] = camT[0] / 1000;
				camTrans[1] = -camT[1] / 1000;
				camTrans[2] = -camT[2] / 1000;
				camTrans[3] = 1;
				
				frame_NO = frame_NO % frame_num;
				movement[frame_NO] = distance_(camTrans, point);
				frame_NO = frame_NO + 1;
				moving = 0.0;
				for (int i = 0; i < frame_num; i++) {
					moving = moving + movement[i];
				}

				prev = cur;
				if (moving > 0.1)
					cur = true;
				else
					cur = false;

				if (true/*!cur && !prev && send_or*/) {
					cv::Matx44f robHobj = getTransMatrix_robHobj(r.rvec, r.tvec); // 物体相对基座标的变换矩阵
					Eigen::Matrix3f m33_obj;
					m33_obj << robHobj(0, 0), robHobj(0, 1), robHobj(0, 2),
						robHobj(1, 0), robHobj(1, 1), robHobj(1, 2),
						robHobj(2, 0), robHobj(2, 1), robHobj(2, 2);

					cv::Matx44f m_grab = graspTrans_2(robHobj); // 机械手的变换矩阵
					
					Eigen::Matrix3d m33;
					m33 << m_grab(0, 0), m_grab(0, 1), m_grab(0, 2),
						m_grab(1, 0), m_grab(1, 1), m_grab(1, 2),
						m_grab(2, 0), m_grab(2, 1), m_grab(2, 2);
					Eigen::Quaterniond qua = Eigen::Quaterniond(m33.transpose());
					qua.normalize();

					Eigen::Matrix4f robHobj_eigen;
					cv::cv2eigen(robHobj, robHobj_eigen);
					position = robHobj_eigen * blueMoon_points[2];
					str_buf = "p " + std::to_string(position[0]) + " " + std::to_string(position[1]) + " " + std::to_string(position[2]);
						 + " " +std::to_string(qua.w()) + " " + std::to_string(qua.x()) + " " + std::to_string(qua.y()) + " " + std::to_string(qua.z());
					// cout << "物体静止，机械臂开始运动，运动指令：" << str_buf << endl;
					// printf("运动指令：%.2f, %.2f, %.2f; 四元数：%.2f, %.2f, %.2f, %.2f  | fps=%.2lf\r", position[0], position[1], position[2], qua.w(), qua.x(), qua.y(), qua.z(), CLOCKS_PER_SEC / double(clock() - beg));
					
					//std::cout << "robHobj= " << m33_obj << endl;
					std::cout << "m_grab= " << m_grab << endl;
					// std::cout << "Mat33= [" << m33 << "]" << endl;
					// std::cout << "toRotationMatrix= " << qua.toRotationMatrix() << endl;
					//std::cout << "Quat= [" << qua.w() << " " << qua.x() << " " << qua.y() << " " << qua.z() << "]" << std::endl;
					// std::cout << "angle=" << angle(m33_obj * Eigen::Vector3f(0, 0, 1)) << std::endl;

					send_or = false;

				}
				else if (cur && !prev && !send_or) {
					cout << "物体开始运动，机械臂停止运动！" << endl;
					send_or = true;
				}
			
				point = camTrans;
			}
		}

		imshow("result", img);

		//printf("(%d,%d)\r", img.cols, img.rows);

		if (cv::waitKey(1) == 27 || cv::waitKey(1) == 'q') {
			cv::destroyAllWindows();

			break;
		}
	}

	// system("pause");
	return 0;
}
