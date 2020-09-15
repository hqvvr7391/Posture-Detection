#pragma once
#ifndef __HEADPOSE_H_
#define __HEADPOSE_H_

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/core.hpp>
#include <opencv2/face.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <vector>
#include <array>

#include <math.h>

#include <dlib/opencv.h>
#include <opencv2/highgui/highgui.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>

#include "Realsense.hpp"
#include "Matrix.hpp"

#include <qobject.h>
#include <qthread.h>
#include <qmutex.h>
#include <qeventloop.h>
#include <qwaitcondition.h>

#define FACE_DOWNSAMPLE_RATIO 4
#define SKIP_FRAMES 2
#define OPENCV_FACE_RENDER

using namespace std;
using namespace cv;

extern QMutex global_mutex;

class Head : public QThread
{
	Q_OBJECT
public:
	explicit Head(QObject* parent = 0);
	~Head();
	
	void Head_pose();
	
	dlib::frontal_face_detector detector;
	dlib::shape_predictor pose_model;

	dlib::full_object_detection face;

	float x, y, z, roll, pitch, yaw;
	Eigen::Matrix4f coordination;
	

	std::vector<dlib::rectangle> faces;
	std::vector<dlib::full_object_detection> shapes;


	sp_point eye_left = { { 36, 39 } };
	sp_point eye_right = { { 45, 42 } };
	sp_point mouse = { { 48, 54 } };
	sp_point nose = { { 30 } };

	float rotation_angle = 0;

	bool state;

signals:
	void sendFrame(cv::Mat frame);
	void thread_onoff(bool state);


public slots:
	void onoff(bool toggled);
	void receiveFrame(cv::Mat received_image);
	void Head_detection();

protected:
	void run() override;

private:
	
	bool m_stop;

	QWaitCondition m_imageCondition;
	QMutex mutex;
	cv::Mat image;


};

void pixel_rearrange(dlib::full_object_detection* shape, float angle, cv::Point center);

Vector3f nose_projection(Vector3f eye_point, Vector3f mouse_point, std::vector<Vector3f> nose);

Matrix4f coordinate(Vector3f Center_point, Vector3f x_point = Vector3f::Zero(), Vector3f y_point = Vector3f::Zero(), Vector3f z_point = Vector3f::Zero());

std::vector<cv::Point3d> get_3d_model_points();

std::vector<cv::Point2d> get_2d_image_points(dlib::full_object_detection& d);

cv::Mat get_camera_matrix(float focal_length, cv::Point2d center);

bool isRotationMatrix(Mat& R);

Vec3f rotationMatrixToEulerAngle(Matrix4f R);

Vec3f rotationMatrixToEulerAngles(Mat& R);


#else
#endif