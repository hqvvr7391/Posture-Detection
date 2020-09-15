#pragma once
#ifndef __REALSENSE_H_
#define __REALSENSE_H_

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>

#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/core.hpp>

#include <Eigen/Dense>

/*#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>*/

#include "Matrix.hpp"
//#include <openpose/headers.hpp>
//#include "Headpose.hpp"
#include "Debug.h"

#include <vector>
#include <chrono>

#include <qobject.h>
#include <qthread.h>
#include <qimage.h>
#include <qmutex.h>
#include <qwaitcondition.h>

#include <qdebug.h>

using namespace rs2;
using namespace cv;


#define frame_width		640
#define frame_height	480


class Realsense : public QThread
{
	Q_OBJECT

public:
	explicit Realsense(QObject* parent = 0);
	~Realsense();

	//////// Realsense
	colorizer* color_map;
	pipeline* pipe;
	config* cfg;
	rs2::align* ali;

	rs2::frameset frame_data;

	void Get_Frame();
	cv::Mat Get_color();
	cv::Mat Get_depth();
	cv::Mat Get_color_depth();

	Eigen::Vector3f Get_motion();

	cv::Mat Color_image = cv::Mat(cv::Scalar(0));


	//////// Qt
	QMutex mutex;


signals:
	void sendFrame_M(cv::Mat frame);
	void sendFrame_Q(QImage frame);
	
public slots:
	void onoff(bool toggled);
	void receiveGrabFrame();
	void request_image();

protected:
	void run() override;

private:
	bool status;
	bool request = false;

	bool m_stop;

	QWaitCondition m_pauseCondition;

};

//void get_point(op::Array<float> shape, sp_point* sp);
//void get_point(dlib::full_object_detection* shape, sp_point* sp);

#else
#endif