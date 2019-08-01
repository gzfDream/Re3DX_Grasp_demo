#pragma once

#ifdef RE3DX_STATIC
	#define _RE3D_API
#endif

#ifndef _RE3D_API

#if defined(_WIN32)

#ifdef RE3DX_EXPORTS
#define _RE3D_API __declspec(dllexport)
#else
#define _RE3D_API __declspec(dllimport)
#endif

#elif defined(__GNUC__) && __GNUC__ >= 4

#ifdef RE3DX_EXPORTS
	#define _RE3D_API __attribute__ ((visibility ("default")))
#else
	#define _RE3D_API 
#endif

#elif
#define _RE3D_API
#endif

#endif

#define _RE3D_BEG  namespace re3d{
#define _RE3D_END  }

#include"opencv2/core/mat.hpp"

_RE3D_BEG

class _RE3D_API Model
{
public:
	class CImpl;

	CImpl *ptr;
public:
	Model();

	~Model();

	Model(const Model &) = delete;
	Model& operator=(const Model&) = delete;

public:
	static void build(const char *_3dModelFile, const char *resultModelFile, const char *params=NULL);

	void load(const char *modelFile);

	cv::Vec3f getCenter();
};

class _RE3D_API Tracker
{
public:
	class CImpl;
	
	CImpl *ptr;
public:
	//class to represent tracking result
	class _RE3D_API Result
	{
	public:
		int  objectID;
		//Confidence of tracking result
		float trackingConfidence;
		//The rotation vector
		cv::Vec3f  rvec;
		//The translation vector
		cv::Vec3f  tvec;
		//The object contours projected to the image
		//std::vector<std::vector<cv::Point>> *projectedContours;
		int			nContourPoints;
		cv::Point  *contourPoints;
	};
public:
	Tracker();

	~Tracker();
	//copy operations are forbidden
	Tracker(const Tracker&) = delete;
	Tracker& operator=(const Tracker&) = delete;

	/*Set precomputed object models and intrinsic camera parameter
	
	@modelFiles : list of object models to be tracked. Currently only single-object tracking is supported.
	@K : intrinsic camera parameters
	*/
	void setModels(Model models[], int count, const cv::Matx33f &K);

	void setModels(Model *models[], int count, const cv::Matx33f &K);

	/*Set optional parameters
	@params : a commandline-like string that represents the input parameters
	          for example, params="-fdFeatures 1000 -fdFeaturesSelected 500" will set -fdFeatures and -fdFeaturesSelected to be the specified value.
	*/
	void setParam(const char *params);

	/*Tracking a frame
	@img : frame image in CV_8UC3 format
	@result[] : array to receive tracking result. The array size should be equal to the number of objects in tracking.
	@return value : the number of results save to @result
	*/
	int tracking(const cv::Mat &img, Result result[]);
};


_RE3D_END



