#pragma once
#include "stdafx.h"

#include "CalChessboard.h"

using namespace cv;
using namespace std;

void getAllImages(std::string path, std::vector<std::string>& files)
{
	intptr_t hFile = 0;

	struct _finddata_t fileinfo;
	std::string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			if ((fileinfo.attrib &  _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				{
					//files.push_back(p.assign(path).append("\\").append(fileinfo.name) );
					getAllImages(p.assign(path).append("\\").append(fileinfo.name), files);
				}
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

// �����Ƶ��
void getImageStream(std::string path) {
	cv::VideoCapture cap;
	Size imgSize(1280, 720);
	cv::Mat img;
	
	cap.open(CAP_DSHOW+1);
	cap.set(CAP_PROP_FRAME_WIDTH, imgSize.width);
	cap.set(CAP_PROP_FRAME_HEIGHT, imgSize.height);

	imgSize = Size(int(cap.get(CAP_PROP_FRAME_WIDTH) + 0.5), int(cap.get(CAP_PROP_FRAME_HEIGHT) + 0.5));
	printf("image-size=(%d,%d)\n", imgSize.width, imgSize.height);

	int image_index = 0;
	int key = 0;

	while (cap.read(img))
	{
		cv::namedWindow("display");
		imshow("display", img);

		key = cv::waitKey(1);
		if (key == '1')
		{
			if (image_index < 10) {
				cv::imwrite(path + "0" + std::to_string(image_index) + ".png", img);
			}
			else {
				cv::imwrite(path + std::to_string(image_index) + ".png", img);
			}
			image_index++;
		}
		else if (key == 27 || key == 'q') {
			cv::destroyAllWindows();
			break;
		}
	}
}


// ��������ڲ�
// @param  ͼƬ·��vector
// @param  ����·��
// @return void
// �ο�link:https://my.oschina.net/abcijkxyz/blog/787659
void internal_reference_calibration(std::vector<std::string> img_files, std::string internal_file) {
	//ifstream fin("calibdata.txt"); /* �궨����ͼ���ļ���·�� */
	ofstream fout(internal_file);  /* ����궨������ļ� */
													 //��ȡÿһ��ͼ�񣬴�����ȡ���ǵ㣬Ȼ��Խǵ���������ؾ�ȷ��	
	cout << "��ʼ��ȡ�ǵ㡭����������";
	int image_count = 0;  /* ͼ������ */
	Size image_size;  /* ͼ��ĳߴ� */
	Size board_size = Size(9, 6);    /* �궨����ÿ�С��еĽǵ��� */
	vector<Point2f> image_points_buf;  /* ����ÿ��ͼ���ϼ�⵽�Ľǵ� */
	vector<vector<Point2f>> image_points_seq; /* �����⵽�����нǵ� */

	int count = -1;//���ڴ洢�ǵ������
	for (int index = 0; index < img_files.size(); index++) {
		image_count++;
		// ���ڹ۲�������
		cout << "image_count = " << image_count << endl;
		/* �������*/
		cout << "-->count = " << count;
		Mat imageInput = imread(img_files[index]);
		if (image_count == 1)  //�����һ��ͼƬʱ��ȡͼ������Ϣ
		{
			image_size.width = imageInput.cols;
			image_size.height = imageInput.rows;
			cout << "image_size.width = " << image_size.width << endl;
			cout << "image_size.height = " << image_size.height << endl;
		}

		/* ��ȡ�ǵ� */
		if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
		{
			cout << "can not find chessboard corners!\n"; //�Ҳ����ǵ�
			exit(1);
		}
		else
		{
			Mat view_gray;
			cvtColor(imageInput, view_gray, CV_RGB2GRAY);
			/* �����ؾ�ȷ�� */
			find4QuadCornerSubpix(view_gray, image_points_buf, Size(5, 5)); //�Դ���ȡ�Ľǵ���о�ȷ��
																			//cornerSubPix(view_gray,image_points_buf,Size(5,5),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
			image_points_seq.push_back(image_points_buf);  //���������ؽǵ�
														   /* ��ͼ������ʾ�ǵ�λ�� */
			drawChessboardCorners(view_gray, board_size, image_points_buf, false); //������ͼƬ�б�ǽǵ�
			imshow("Camera Calibration", view_gray);//��ʾͼƬ
			waitKey(500);//��ͣ0.5S		
		}
	}

	int total = image_points_seq.size();
	cout << "total = " << total << endl;
	int CornerNum = board_size.width*board_size.height;  //ÿ��ͼƬ���ܵĽǵ���
	for (int ii = 0; ii<total; ii++)
	{
		if (0 == ii % CornerNum)// 24 ��ÿ��ͼƬ�Ľǵ���������ж������Ϊ����� ͼƬ�ţ����ڿ���̨�ۿ� 
		{
			int i = -1;
			i = ii / CornerNum;
			int j = i + 1;
			cout << "--> �� " << j << "ͼƬ������ --> : " << endl;
		}
		if (0 == ii % 3)	// ���ж���䣬��ʽ����������ڿ���̨�鿴
		{
			cout << endl;
		}
		else
		{
			cout.width(10);
		}
		//������еĽǵ�
		cout << " -->" << image_points_seq[ii][0].x;
		cout << " -->" << image_points_seq[ii][0].y;
	}
	cout << "�ǵ���ȡ��ɣ�\n";

	//������������궨
	cout << "��ʼ�궨������������";
	/*������ά��Ϣ*/
	Size square_size = Size(3, 3);  /* ʵ�ʲ����õ��ı궨����ÿ�����̸�Ĵ�С */
	vector<vector<Point3f>> object_points; /* ����궨���Ͻǵ����ά���� */
										   /*�������*/
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* ������ڲ������� */
	vector<int> point_counts;  // ÿ��ͼ���нǵ������
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); /* �������5������ϵ����k1,k2,p1,p2,k3 */
	vector<Mat> tvecsMat;  /* ÿ��ͼ�����ת���� */
	vector<Mat> rvecsMat; /* ÿ��ͼ���ƽ������ */
						  /* ��ʼ���궨���Ͻǵ����ά���� */
	int i, j, t;
	for (t = 0; t<image_count; t++)
	{
		vector<Point3f> tempPointSet;
		for (i = 0; i<board_size.height; i++)
		{
			for (j = 0; j<board_size.width; j++)
			{
				Point3f realPoint;
				/* ����궨�������������ϵ��z=0��ƽ���� */
				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}
	/* ��ʼ��ÿ��ͼ���еĽǵ��������ٶ�ÿ��ͼ���ж����Կ��������ı궨�� */
	for (i = 0; i<image_count; i++)
	{
		point_counts.push_back(board_size.width*board_size.height);
	}
	/* ��ʼ�궨 */
	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
	cout << "�궨��ɣ�\n";
	destroyAllWindows();

	//�Ա궨�����������
	cout << "��ʼ���۱궨���������������\n";
	double total_err = 0.0; /* ����ͼ���ƽ�������ܺ� */
	double err = 0.0; /* ÿ��ͼ���ƽ����� */
	vector<Point2f> image_points2; /* �������¼���õ���ͶӰ�� */
	cout << "\tÿ��ͼ��ı궨��\n";
	fout << "ÿ��ͼ��ı궨��\n";
	for (i = 0; i<image_count; i++)
	{
		vector<Point3f> tempPointSet = object_points[i];
		/* ͨ���õ������������������Կռ����ά���������ͶӰ���㣬�õ��µ�ͶӰ�� */
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
		/* �����µ�ͶӰ��;ɵ�ͶӰ��֮������*/
		vector<Point2f> tempImagePoint = image_points_seq[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err += err /= point_counts[i];
		std::cout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
		fout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
	}
	std::cout << "����ƽ����" << total_err / image_count << "����" << endl;
	fout << "����ƽ����" << total_err / image_count << "����" << endl << endl;
	std::cout << "������ɣ�" << endl;
	//���涨����  	
	std::cout << "��ʼ���涨����������������" << endl;
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* ����ÿ��ͼ�����ת���� */
	fout << "����ڲ�������" << endl;
	fout << cameraMatrix << endl << endl;
	fout << "����ϵ����\n";
	fout << distCoeffs << endl << endl << endl;
	for (int i = 0; i<image_count; i++)
	{
		fout << "��" << i + 1 << "��ͼ�����ת������" << endl;
		fout << tvecsMat[i] << endl;
		/* ����ת����ת��Ϊ���Ӧ����ת���� */
		Rodrigues(tvecsMat[i], rotation_matrix);
		fout << "��" << i + 1 << "��ͼ�����ת����" << endl;
		fout << rotation_matrix << endl;
		fout << "��" << i + 1 << "��ͼ���ƽ��������" << endl;
		fout << rvecsMat[i] << endl << endl;
	}
	std::cout << "��ɱ���" << endl;
	fout << endl;
	/************************************************************************
	��ʾ������
	*************************************************************************/
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);
	std::cout << "�������" << endl;
}


// ��������ڲ�
// @param  ͼƬ·��vector
// @param  ����·��
// @param  �任����
// @param  ����
// @return void
// �ο�link:https://my.oschina.net/abcijkxyz/blog/787659
void internal_reference_calibration(std::vector<std::string> img_files, std::string internal_file, Mat& cameraMatrix, Mat distCoeffs) {
	//ifstream fin("calibdata.txt"); /* �궨����ͼ���ļ���·�� */
	ofstream fout(internal_file);  /* ����궨������ļ� */
													 //��ȡÿһ��ͼ�񣬴�����ȡ���ǵ㣬Ȼ��Խǵ���������ؾ�ȷ��	
	cout << "��ʼ��ȡ�ǵ㡭����������";
	int image_count = 0;  /* ͼ������ */
	Size image_size;  /* ͼ��ĳߴ� */
	Size board_size = Size(9, 6);    /* �궨����ÿ�С��еĽǵ��� */
	vector<Point2f> image_points_buf;  /* ����ÿ��ͼ���ϼ�⵽�Ľǵ� */
	vector<vector<Point2f>> image_points_seq; /* �����⵽�����нǵ� */

	for (int index = 0; index < img_files.size(); index++) {
		image_count++;
		// ���ڹ۲�������
		cout << "image_count = " << image_count << endl;
		Mat imageInput = imread(img_files[index]);
		if (image_count == 1)  //�����һ��ͼƬʱ��ȡͼ������Ϣ
		{
			image_size.width = imageInput.cols;
			image_size.height = imageInput.rows;
			cout << "image_size.width = " << image_size.width << endl;
			cout << "image_size.height = " << image_size.height << endl;
		}

		/* ��ȡ�ǵ� */
		if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
		{
			cout << "can not find chessboard corners!\n"; //�Ҳ����ǵ�
			exit(1);
		}
		else
		{
			Mat view_gray;
			cvtColor(imageInput, view_gray, CV_RGB2GRAY);
			/* �����ؾ�ȷ�� */
			find4QuadCornerSubpix(view_gray, image_points_buf, Size(5, 5)); //�Դ���ȡ�Ľǵ���о�ȷ��
																			//cornerSubPix(view_gray,image_points_buf,Size(5,5),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
			image_points_seq.push_back(image_points_buf);  //���������ؽǵ�
														   /* ��ͼ������ʾ�ǵ�λ�� */
			drawChessboardCorners(view_gray, board_size, image_points_buf, false); //������ͼƬ�б�ǽǵ�
			imshow("Camera Calibration", view_gray);//��ʾͼƬ
			waitKey(500);//��ͣ0.5S		
		}
	}

	int total = image_points_seq.size();
	cout << "total = " << total << endl;
	int CornerNum = board_size.width*board_size.height;  //ÿ��ͼƬ���ܵĽǵ���
	for (int ii = 0; ii < total; ii++)
	{
		if (0 == ii % CornerNum)// 24 ��ÿ��ͼƬ�Ľǵ���������ж������Ϊ����� ͼƬ�ţ����ڿ���̨�ۿ� 
		{
			int i = -1;
			i = ii / CornerNum;
			int j = i + 1;
			cout << "--> �� " << j << "ͼƬ������ --> : " << endl;
		}
		if (0 == ii % 3)	// ���ж���䣬��ʽ����������ڿ���̨�鿴
		{
			cout << endl;
		}
		else
		{
			cout.width(10);
		}
		//������еĽǵ�
		cout << " -->" << image_points_seq[ii][0].x;
		cout << " -->" << image_points_seq[ii][0].y;
	}
	cout << "�ǵ���ȡ��ɣ�\n";

	//������������궨
	cout << "��ʼ�궨������������";
	/*������ά��Ϣ*/
	Size square_size = Size(2, 2);  /* ʵ�ʲ����õ��ı궨����ÿ�����̸�Ĵ�С */
	vector<vector<Point3f>> object_points; /* ����궨���Ͻǵ����ά���� */
										   /*�������*/
	cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* ������ڲ������� */
	vector<int> point_counts;  // ÿ��ͼ���нǵ������
	distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); /* �������5������ϵ����k1,k2,p1,p2,k3 */
	vector<Mat> tvecsMat;  /* ÿ��ͼ�����ת���� */
	vector<Mat> rvecsMat; /* ÿ��ͼ���ƽ������ */
						  /* ��ʼ���궨���Ͻǵ����ά���� */
	int i, j, t;
	for (t = 0; t < image_count; t++)
	{
		vector<Point3f> tempPointSet;
		for (i = 0; i < board_size.height; i++)
		{
			for (j = 0; j < board_size.width; j++)
			{
				Point3f realPoint;
				/* ����궨�������������ϵ��z=0��ƽ���� */
				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}
	/* ��ʼ��ÿ��ͼ���еĽǵ��������ٶ�ÿ��ͼ���ж����Կ��������ı궨�� */
	for (i = 0; i < image_count; i++)
	{
		point_counts.push_back(board_size.width*board_size.height);
	}
	/* ��ʼ�궨 */
	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
	cout << "�궨��ɣ�\n";
	destroyAllWindows();

	//�Ա궨�����������
	cout << "��ʼ���۱궨���������������\n";
	double total_err = 0.0; /* ����ͼ���ƽ�������ܺ� */
	double err = 0.0; /* ÿ��ͼ���ƽ����� */
	vector<Point2f> image_points2; /* �������¼���õ���ͶӰ�� */
	cout << "\tÿ��ͼ��ı궨��\n";
	fout << "ÿ��ͼ��ı궨��\n";
	for (i = 0; i < image_count; i++)
	{
		vector<Point3f> tempPointSet = object_points[i];
		/* ͨ���õ������������������Կռ����ά���������ͶӰ���㣬�õ��µ�ͶӰ�� */
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
		/* �����µ�ͶӰ��;ɵ�ͶӰ��֮������*/
		vector<Point2f> tempImagePoint = image_points_seq[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err += err /= point_counts[i];
		std::cout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
		fout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
	}
	std::cout << "����ƽ����" << total_err / image_count << "����" << endl;
	fout << "����ƽ����" << total_err / image_count << "����" << endl << endl;
	std::cout << "������ɣ�" << endl;
	//���涨����  	
	std::cout << "��ʼ���涨����������������" << endl;
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* ����ÿ��ͼ�����ת���� */
	fout << "����ڲ�������" << endl;
	fout << cameraMatrix << endl << endl;
	fout << "����ϵ����\n";
	fout << distCoeffs << endl << endl << endl;
	for (int i = 0; i < image_count; i++)
	{
		fout << "��" << i + 1 << "��ͼ�����ת������" << endl;
		fout << tvecsMat[i] << endl;
		/* ����ת����ת��Ϊ���Ӧ����ת���� */
		Rodrigues(tvecsMat[i], rotation_matrix);
		fout << "��" << i + 1 << "��ͼ�����ת����" << endl;
		fout << rotation_matrix << endl;
		fout << "��" << i + 1 << "��ͼ���ƽ��������" << endl;
		fout << rvecsMat[i] << endl << endl;
	}
	std::cout << "��ɱ���" << endl;
	fout << endl;
	/************************************************************************
	��ʾ������
	*************************************************************************/
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);
	std::cout << "�������" << endl;
}


// ��ȡ���
void external_reference_calibration(cv::Mat camD, Mat distCoeffD, string imgpath, string calibFile) {
	// double distCoeffD[5] = { 0, 0, 0, 0, 0 };
	double markerRealSize = 2;

	CalChessboard cal = CalChessboard(camD, distCoeffD, markerRealSize);

	// �洢�궨ͼƬ
	// string imgpath = "./data/Images/";
	// �洢�궨���
	//string calibFile = "./data/cam_cal_robot.txt";
	cal.calprocess(imgpath, calibFile);
	cv::destroyAllWindows();
}
