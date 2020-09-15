#pragma once

#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/core.hpp>

#include "Headpose.hpp"
#include "Openpose.hpp"

#include <qobject.h>
#include <qthread.h>
#include <qmutex.h>
#include <qwaitcondition.h>

#define WAIT_MS		500

class Image_process : public QThread
{
	Q_OBJECT
public:
	explicit Image_process(QObject* parent = 0);
	~Image_process();

	bool state;

	Head* head;
	Pose* pose;

signals:
	void request_Image();
	void sendImage(cv::Mat frame);
	void sendFrame_Q(QImage frame);
	void Head_signal(cv::Mat image);

public slots:
	Q_INVOKABLE void receiveFrame(cv::Mat image);

	void head_Status(bool status);
	void pose_Status(bool status);

	void receive_headFrame(cv::Mat image);
	void receive_poseFrame(cv::Mat image);


protected:

	void run() override;

private:
	void post_processing();

	bool head_status = 0;
	bool pose_status = 0;

	bool waitforimage = true;
	bool head_waitforimage = true;
	bool pose_waitforimage = true;

	bool m_stop;
	std::chrono::duration<double> sec;

	cv::Mat Recent_Image;
	cv::Mat post_Image;
	
	cv::Mat Head_Image;
	cv::Mat Pose_Image;

	QMutex mutex;
	
	QWaitCondition m_ImageCondition;
	QMutex image_mutex;

	QWaitCondition m_headCondition;
	QMutex head_mutex;

	QWaitCondition m_poseCondition;
	QMutex pose_mutex;
};