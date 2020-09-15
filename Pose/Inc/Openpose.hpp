#pragma once
#ifndef __OPENPOSE_H_
#define __OPENPOSE_H_



// OpenPose dependencies
#include <openpose/headers.hpp>

#include "Realsense.hpp"
#include "Matrix.hpp"
#include <qthread.h>


class Pose : public QThread
{
	Q_OBJECT
public:
	explicit Pose(QObject* parent = 0);
	~Pose();

	op::Wrapper* opWrapper;

	void op_Init();
	void Get_data();

	sp_point neck = { { 1 } };
	sp_point shoulder_left = { { 2 } };
	sp_point shoulder_right = { { 5 } };

	std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> Datum;
	

signals:
	void sendFrame(cv::Mat image);
	void thread_onoff(bool state);

public slots:
	void onoff(bool toggled);
	
	void receiveFrame(cv::Mat received_image);


protected:

	void run() override;
	//std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> Get_Keypoint(cv::Mat image);
	
private:
	bool isRunning;
	bool m_stop = false;

	QWaitCondition m_imageCondition;
	QMutex mutex;
	cv::Mat image = cv::Mat(300, 300, CV_8UC3, cv::Scalar(0,0,0));
};

//void display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);


#else
#endif