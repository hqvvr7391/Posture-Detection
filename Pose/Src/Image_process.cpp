#include "Image_process.hpp"

Image_process::Image_process(QObject* parent) :
	QThread(parent)
{
	m_stop = false;
}


void Image_process::run()
{
	qInfo() << IM_MSG << "run";
	image_mutex.lock();
	m_ImageCondition.wakeAll();
	image_mutex.unlock();
	this->setPriority(HighPriority);
	while (1)
	{
		std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
		//this->msleep(10);
	
		qDebug() << IM_MSG << "start";
		

		mutex.lock();
		if (m_stop)
		{
			mutex.unlock();
			break;
		}
		mutex.unlock();


		waitforimage = true;
		head_waitforimage = true;
		pose_waitforimage = true;

		qDebug() << IM_MSG << "wait threads";
		emit request_Image();
		if (waitforimage)
		{
			image_mutex.lock();
	
			m_ImageCondition.wait(&image_mutex, WAIT_MS);
			qDebug() << IM_MSG << "wait frame end";
			image_mutex.unlock();
		}
		
		if (head_status)
		{
			if (head_waitforimage)
			{
				head_mutex.lock();
				qDebug() << IM_MSG << "wait head signal";
				m_headCondition.wait(&head_mutex, WAIT_MS);
				head_mutex.unlock();
			}
		}
		
		if (pose_status)
		{
			if (pose_waitforimage)
			{
				pose_mutex.lock();
				qDebug() << IM_MSG << "wait pose signal";
				m_poseCondition.wait(&pose_mutex, WAIT_MS);
				pose_mutex.unlock();
				
			}
		}

		post_processing();
		sec = std::chrono::system_clock::now() - start;

	} qInfo() << IM_MSG << "ended...";

}

void Image_process::receiveFrame(cv::Mat image)
{
	waitforimage = false;

	image_mutex.lock();
	m_ImageCondition.wakeAll();
	image_mutex.unlock();

	this->Recent_Image = image.clone();
	qDebug() << IM_MSG << "image received";
}

void Image_process::receive_headFrame(cv::Mat image)
{
	head_waitforimage = false;
	head_mutex.lock();
	m_headCondition.wakeOne();
	head_mutex.unlock();
	this->Head_Image = image.clone();
	qDebug() << IM_MSG << " head image received";
}


void Image_process::receive_poseFrame(cv::Mat image)
{
	pose_waitforimage = false;
	pose_mutex.lock();
	m_poseCondition.wakeOne();
	pose_mutex.unlock();
	this->Pose_Image = image.clone();
	qDebug() << IM_MSG << "pose image received";
}

void Image_process::post_processing()
{
	
	if (this->Head_Image.empty() || !head_status)
	{
		qDebug() << IM_MSG << "post head null";
		//this->Head_Image = cv::Scalar(0);
	}
	else
	{
		qDebug() << IM_MSG << "post head add";
		cv::add(this->Head_Image, this->Recent_Image, this->Recent_Image);
	}

	if (this->Pose_Image.empty() || !pose_status)
	{
		qDebug() << IM_MSG << "post pose null";
		//this->Pose_Image = cv::Scalar(0);
	}
	else
	{
		qDebug() << IM_MSG << "post pose add";
		//cv::add(this->Pose_Image, this->Recent_Image, this->Recent_Image);
	}
	

	char fps[5];
	sprintf(fps, "%.2f", 1.f / sec.count());
	putText(this->Recent_Image, fps, cv::Point(20,20), 2, 0.75, cv::Scalar(255, 255, 0));
	qDebug() << IM_MSG << "post fps add";

	cv::Mat dp_image;
	cv::cvtColor(this->Recent_Image, dp_image, COLOR_BGR2RGB);
	
	QImage output((const unsigned char*)dp_image.data, dp_image.cols, dp_image.rows, QImage::Format_RGB888);
	emit sendFrame_Q(output);

}


void Image_process::head_Status(bool status)
{
	head_mutex.lock();
	head_status = status;
	head_mutex.unlock();
	if (status)
	{

	}
	else
	{
		head_mutex.lock();
		m_headCondition.wakeAll();
		head_mutex.unlock();
	}
}


void Image_process::pose_Status(bool status)
{
	qDebug() << IM_MSG << "get pose status";
	pose_mutex.lock();
	pose_status = status;
	m_poseCondition.wakeAll();
	pose_mutex.unlock();

	if (status)
	{

	}
	else
	{
		
	}
}


/*void Image_process::stop()
{	
	m_stop = true;

	waitforimage = false;
	head_waitforimage = false;
	pose_waitforimage = false;

	image_mutex.lock();
	m_ImageCondition.wakeAll();
	image_mutex.unlock();

	head_mutex.lock();
	m_headCondition.wakeAll();
	head_mutex.unlock();

	pose_mutex.lock();
	m_poseCondition.wakeAll();
	pose_mutex.unlock();

	m_stop = true;

	this->quit();
	qDebug() << IM_MSG << "stop...");

}
*/
Image_process::~Image_process()
{
	waitforimage = false;
	head_waitforimage = false;
	pose_waitforimage = false;

	image_mutex.lock();
	m_ImageCondition.wakeAll();
	image_mutex.unlock();

	head_mutex.lock();
	m_headCondition.wakeAll();
	head_mutex.unlock();

	pose_mutex.lock();
	m_poseCondition.wakeAll();
	pose_mutex.unlock();

	this->terminate();

	qInfo() << IM_MSG << "Destroyed";
	
}