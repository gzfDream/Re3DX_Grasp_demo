#pragma once
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <vector>
#include <numeric>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include"re3dx.h"
#include"cvrender.h"


#define PI 3.1415926

//生成识别数据(临时没有实现)
void Generator_data(std::string obj, std::string dest) {

	CVRModel cvr_model;
	cvr_model.load("C:\\Users\\gzf\\OneDrive\\code\\RE3D_grasp_demo\\modes-618\\bottle2\\bottle2_r.obj");
	cvr_model.saveAs("C:\\Users\\gzf\\OneDrive\\code\\RE3D_grasp_demo\\modes-618\\bottle2\\bottle2_r.3ds");

	re3d::Model::build("C:\\Users\\gzf\\OneDrive\\code\\RE3D_grasp_demo\\modes-618\\bottle2\\bottle2_r.3ds",
		"C:\\Users\\gzf\\OneDrive\\code\\RE3D_grasp_demo\\modes-618\\bottle2\\bottle2_r.idx121");
}


// 读取文件到mat
cv::Matx44f readf(std::string str) {
	cv::Matx44f mat;

	std::ifstream  file(str);
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
cv::Mat process_transMatrix(cv::Vec3f rvec, cv::Vec3f tvec) {
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

	cv::Matx44f robHcam = readf("..//EyeToHand//data//result.txt");

	return trans_mat * robHcam;
}


// 获得机械臂坐标系下的位置
Eigen::Vector4f getPosition(Eigen::Vector4f tvec) {
	cv::Matx44f mat = readf("../EyeToHand/data/result.txt");
	Eigen::Matrix4f mat_eigen;
	cv::cv2eigen(mat, mat_eigen);
	
	return  mat_eigen * tvec;
}


//计算两点之间的距离
double distance(cv::Vec3f v1, cv::Vec3f v2) {
	return sqrtf(powf((v1[0] - v2[0]), 2) + powf((v1[1] - v2[1]), 2) + powf((v1[2] - v2[2]), 2));
}


// 得到物体相对于相机的位置
cv::Vec3f getCameraT_util(const cv::Vec3f &rvec, const cv::Vec3f &tvec, const cv::Vec3f &modelCenter)
{
	cv::Matx44f mModelView = cvrm::fromRT(rvec, tvec);

	// std::cout << "Center: "<< modelCenter << std::endl;
	//std::cout << "mat: " << mModelView << std::endl;

	return modelCenter * mModelView;
}



bool is_Moving(int frame_num, std::vector<cv::Vec3f> movement, int top_index, int &static_num) {
	int moving_frame = 0; //移动的帧数
	bool moving = true; //是否移动

	for (int i = 0; i < frame_num; i++) {
		double key = 0.03;
		if (distance(movement[i], movement[top_index]) > key)
			moving_frame++;
	}

	if (moving_frame >= 5) {
		moving = true;
		// std::cout << "moving" << std::endl;
		static_num = 0;
	}
	else {
		static_num++;
		if (static_num > 15) {
			moving = false;
			static_num = 15;
			// std::cout << "static" << std::endl;
		}
	}

	return moving;
}

// 得到物体相对于机械臂实时的变换矩阵
Eigen::Matrix4f get_robHobj(const cv::Vec3f &rvec, const cv::Vec3f &tvec) {
	cv::Vec3f tvec_mm = tvec / 1000;

	Matx44f mModelView = cvrm::fromRT(rvec, tvec_mm);
	//std::cout << "mModelView= " << mModelView.t() << endl;

	cv::Matx44f x_90;
	x_90 << 1., 0., 0., 0.,
		0., 0., 1., 0.,
		0., -1., 0., 0.,
		0., 0., 0., 1.;
	cv::Matx44f x180;
	x180 << 1, 0, 0, 0,
		0, -1, 0, 0,
		0, 0, -1, 0,
		0, 0, 0, 1;

	cv::Matx44f camHobj = x180 * mModelView.t();
	//std::cout << "camHobj= " << camHobj << endl;

	cv::Matx44f robHcam = readf("..//EyeToHand//data//result.txt");

	cv::Matx44f robHobj = robHcam * camHobj * x_90;

	Eigen::Matrix4f robHobj_;
	// std::cout << "R: " << robHobj << std::endl;
	cv::cv2eigen(robHobj, robHobj_);
	// std::cout << "L: " << robHobj_ << std::endl;
	return robHobj_;
}


// 获得绕坐标轴的变换矩阵
//Eigen::Matrix4f get_XYZ_TransMat(std::string xyz, double angle) {
//	Eigen::Matrix4f mat;
//	float an = angle / 180 * PI;
//	if (xyz == "x") {
//		mat << 1., 0.0, 0.0, 0.0,
//			0., cos(an), -sin(an), 0.0,
//			0, sin(an), cos(an), 0,
//			0 0 0 1;
//	}
//	else if (xyz == "y") {
//
//	}
//	else if (xyz == "z") {
//
//	}
//}


// 获得蓝月亮的抓取姿态
void getBlueMoonGraspPose(Eigen::Matrix4f robHobj, Eigen::Quaternionf &qua, Eigen::Vector4f &T) {
	// 抓取点初始化
	std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> blueMoon_points;
	blueMoon_points.push_back(Eigen::Vector4f(0., -0.010, 0.120, 1));//顶
	blueMoon_points.push_back(Eigen::Vector4f(0., -0.020, -0.100, 1));//底
	blueMoon_points.push_back(Eigen::Vector4f(0., -0.080, -0.00, 1));//前
	blueMoon_points.push_back(Eigen::Vector4f(0., 0.065, -0.030, 1));//前

	int minP = 0;
	double minD = 1000;
	Eigen::Vector4f Base_position;
	for (int i = 0; i < blueMoon_points.size(); i++) {
		if (i == 0) {
			blueMoon_points[i][1] = blueMoon_points[i][1] + 0.035;
		}
		Base_position = robHobj * blueMoon_points[i];
		if (Base_position[0] < minD) {
			minD = Base_position[0];
			minP = i;
		}
		if(i == 0)
			blueMoon_points[i][1] = blueMoon_points[i][1] - 0.035;
	}

	//设置抓取姿态

	//1.把手朝外
	Eigen::Matrix4f x90;
	x90 << 1, 0, 0, 0,
		0, 0, 1, 0,
		0, -1, 0, 0,
		0, 0, 0, 1;

	Eigen::Matrix4f x_90;
	x_90 << 1, 0, 0, 0,
		0, 0, -1, 0,
		0, 1, 0, 0,
		0, 0, 0, 1;

	Eigen::Matrix4f z180;
	z180 <<
		-1, 0, 0, 0,
		0, -1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	Eigen::Matrix4f m_grab;
	Eigen::Matrix3f R33;

	Eigen::Matrix3f R_z;
	R_z = robHobj.block(0, 0, 3, 3);
	Eigen::Vector3f z(0, 0, 1);
	Eigen::Vector3f z_aixs = R_z * Eigen::Vector3f(0., 0., 1.);
	int angle_z = 0;

	switch (minP)
	{
	case 0:
		qua = Eigen::Quaternionf(0.5, -0.5, -0.5, -0.5);
		T = robHobj * blueMoon_points[0];
		//std::cout << "朝前" << std::endl;
		break;
	case 1:
		qua = Eigen::Quaternionf(0.5, -0.5, -0.5, -0.5);
		T = robHobj * blueMoon_points[1];
		//std::cout << "朝后" << std::endl;
		break;
	case 2:
		angle_z = 180 * (acos(z_aixs.dot(z) / (z.norm()*z_aixs.norm())) / PI);
		if (angle_z<90) {
			m_grab = robHobj * x90 * z180;
			R33 << m_grab(0, 0), m_grab(0, 1), m_grab(0, 2),
				m_grab(1, 0), m_grab(1, 1), m_grab(1, 2),
				m_grab(2, 0), m_grab(2, 1), m_grab(2, 2);
			qua = Eigen::Quaternionf(R33.transpose());

			// qua = Eigen::Quaternionf(0.5, -0.5, -0.5, -0.5);
			T = robHobj * blueMoon_points[2];
			
			//std::cout << "上--";
		}
		else{
			m_grab = robHobj * x90;
			R33 = m_grab.block(0, 0, 3, 3);
			qua = Eigen::Quaternionf(R33.transpose());

			// qua = Eigen::Quaternionf(0.5, -0.5, -0.5, -0.5);
			T = robHobj * blueMoon_points[2];

			//std::cout << "下--";
		}
		
		//std::cout << "把手朝后" << std::endl;
		break;
	case 3:
		angle_z = 180 * (acos(z_aixs.dot(z) / (z.norm()*z_aixs.norm())) / PI);
		if (angle_z < 90) {
			m_grab = robHobj * x_90;
			R33 = m_grab.block(0, 0, 3, 3);
			qua = Eigen::Quaternionf(R33.transpose());

			// qua = Eigen::Quaternionf(0.5, -0.5, -0.5, -0.5);
			T = robHobj * blueMoon_points[2];

			//std::cout << "上--";
		}
		else {
			m_grab = robHobj * x_90 * z180;
			R33 << m_grab(0, 0), m_grab(0, 1), m_grab(0, 2),
				m_grab(1, 0), m_grab(1, 1), m_grab(1, 2),
				m_grab(2, 0), m_grab(2, 1), m_grab(2, 2);
			qua = Eigen::Quaternionf(R33.transpose());

			// qua = Eigen::Quaternionf(0.5, -0.5, -0.5, -0.5);
			T = robHobj * blueMoon_points[2];

			//std::cout << "下--";
		}

		//std::cout << "把手朝前" << std::endl;
		break;
	default:
		break;
	}
}


// 获得C柠檬的抓取姿态
//计算物体Z轴与基座标系的夹角
void getCLemonGraspPose(Eigen::Matrix4f robHobj, Eigen::Quaternionf &qua) {
	Eigen::Matrix3f m3;
	m3 << robHobj(0,0), robHobj(0,1), robHobj(0,2),
		robHobj(1,0), robHobj(1,1), robHobj(1,2),
		robHobj(2,0), robHobj(2,1), robHobj(2,2);

	Eigen::Vector3f z_aixs = m3 * Eigen::Vector3f(0.,0.,1.);
	Eigen::Vector3f x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);
	
	int angle_x = 180*(acos(z_aixs.dot(x) / (x.norm()*z_aixs.norm())) / PI);
	angle_x = angle_x > 90 ? (180 - angle_x) : angle_x;
	//int angle_x = acos(z_aixs.dot(y) / (y.norm()*z_aixs.norm()));
	int angle_z = 180*(acos(z_aixs.dot(z) / (z.norm()*z_aixs.norm())) / PI);
	angle_z = angle_z > 90 ? (180 - angle_z) : angle_z;
	
	/* cout */ 
	// std::cout << "z_axis:" << z_aixs << std::endl;
	// std::cout << "x:" << angle_x << std::endl;
	// std::cout << "z:" << angle_z << std::endl;

	if (angle_x>=45 && angle_z<=45) {
		// std::cout << "垂直" << std::endl;
		qua = Eigen::Quaternionf(0.5, -0.5, -0.5, -0.5);
	}
	else if (angle_x<45 && angle_z>45) {
		// std::cout << "水平" << std::endl;
		qua = Eigen::Quaternionf(-0.0485, 0.6609, 0.747, 0.051);;
	}
	/*else if(angle_x>45 && angle_z>45) {
		std::cout << "水平朝←→" << std::endl;
	}*/

}


// 获得小明的抓取姿态
void getXiaoMingGraspPose(Eigen::Matrix4f robHobj, Eigen::Quaternionf &qua) {
	getCLemonGraspPose(robHobj, qua);
}


// 获得dalong的抓取姿态
void getDaLongGraspPose(Eigen::Matrix4f robHobj, Eigen::Quaternionf &qua, Eigen::Vector4f &T) {
	// 抓取点初始化
	std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> blueMoon_points;
	blueMoon_points.push_back(Eigen::Vector4f(0., 0., 0.100, 1));//顶
	blueMoon_points.push_back(Eigen::Vector4f(0., -0.080, 0.00, 1));//底
	blueMoon_points.push_back(Eigen::Vector4f(0., 0.0, -0.100, 1));//前
	blueMoon_points.push_back(Eigen::Vector4f(0., 0.080, 0.0, 1));//前


	int minP = 0;
	double minD = 1000;
	Eigen::Vector4f Base_position;
	for (int i = 0; i < blueMoon_points.size(); i++) {
		Base_position = robHobj * blueMoon_points[i];
		if (Base_position[0] < minD) {
			minD = Base_position[0];
			minP = i;
		}
	}



	switch (minP)
	{
	case 0:

		qua = Eigen::Quaternionf(0.5, -0.5, -0.5, -0.5);
		T = robHobj * blueMoon_points[0];
		break;
	case 1:
		qua = Eigen::Quaternionf(0.5, -0.5, -0.5, -0.5);
		T = robHobj * blueMoon_points[0];
		break;
	case 2:
		qua = Eigen::Quaternionf(0.5, -0.5, -0.5, -0.5);
		T = robHobj * blueMoon_points[0];
		break;
	case 3:
		qua = Eigen::Quaternionf(0.5, -0.5, -0.5, -0.5);
		T = robHobj * blueMoon_points[0];
		break;
	default:
		break;
	}

}