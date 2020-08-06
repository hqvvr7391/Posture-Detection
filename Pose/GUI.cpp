#include "GUI.h"

GUI::GUI(QWidget *parent)
	: QMainWindow(parent)

{
	ui.setupUi(this);
	qInfo() << GUI_MSG << QThread::currentThreadId();
	realsense = new Realsense(parent);
	image = new Image_process(parent);
	head = new Head(parent);
	pose = new Pose(parent);
	setup();
}

void GUI::setup()
{
	//camera_timer_thread = new QThread();
	
	//connect(realsense, SIGNAL(finished()), realsense, SLOT(deleteLater()));
	
	
	connect(ui.Camera_Switch, SIGNAL(toggled(bool)), realsense, SLOT(onoff(bool)));		// Realsense on off
	connect(ui.Face_Switch, SIGNAL(toggled(bool)), head, SLOT(onoff(bool)));			// face detection
	connect(ui.Pose_Switch, SIGNAL(toggled(bool)), pose, SLOT(onoff(bool)));		// pose onoff status

	connect(head, SIGNAL(thread_onoff(bool)), image, SLOT(Head_status(bool)));		// face onoff status
	connect(pose, SIGNAL(thread_onoff(bool)), image, SLOT(Pose_status(bool)));

	connect(realsense, SIGNAL(started()), image, SLOT(start()));						// image process trigger
	connect(realsense, SIGNAL(finished()), image, SLOT(quit()));						//

	connect(image, SIGNAL(request_Image()), realsense, SLOT(request_image()));			// image request

	connect(realsense, SIGNAL(sendFrame(cv::Mat)), image, SLOT(receiveFrame(cv::Mat)), Qt::DirectConnection);	// base color image transmit

	connect(realsense, SIGNAL(sendFrame2(cv::Mat)), this, SLOT(receiveFrame2(cv::Mat))); // image booked


	connect(head, SIGNAL(sendFrame(cv::Mat)), image, SLOT(receive_headFrame(cv::Mat)), Qt::DirectConnection);
	connect(pose, SIGNAL(sendFrame(cv::Mat)), image, SLOT(receive_poseFrame(cv::Mat)), Qt::DirectConnection);
	
	connect(image, SIGNAL(sendFrame(cv::Mat)), this, SLOT(receiveFrame(cv::Mat)));		// display signal

	qInfo() << GUI_MSG << "Initialized";
	

}

void GUI::receiveFrame(cv::Mat image)
{
	cv::Mat dp_image;
	dp_image = cv::Scalar(0);

	if (!image.empty())
		cv::cvtColor(image, dp_image, COLOR_BGR2RGB);
	qDebug() << GUI_MSG << "image convert";
	QImage output((const unsigned char*)dp_image.data, dp_image.cols, dp_image.rows, QImage::Format_RGB888);
	ui.Camera->setPixmap(QPixmap::fromImage(output));
	ui.Camera->update();

}

void GUI::receiveFrame2(cv::Mat image)
{
	cv::Mat dp_image;
	dp_image = cv::Scalar(0);

	if (!realsense->Color_image.empty())
		cv::cvtColor(realsense->Color_image, dp_image, COLOR_BGR2RGB);

	QImage output((const unsigned char*)dp_image.data, dp_image.cols, dp_image.rows, QImage::Format_RGB888);
	ui.Camera_2->setPixmap(QPixmap::fromImage(output));
	ui.Camera_2->update();
}


void GUI::on_Camera_Switch_clicked(bool toggled)
{
	if (toggled == true)
	{
		
	}
	else if (toggled == false)
	{
		ui.Face_Switch->setChecked(toggled);
		ui.Pose_Switch->setChecked(toggled);
		image->terminate();
	}
}

void GUI::on_Face_Switch_clicked(bool toggled)
{

	if (toggled == true)
	{
		ui.Camera_Switch->setChecked(toggled);
		connect(realsense, SIGNAL(sendFrame(cv::Mat)), head, SLOT(receiveFrame(cv::Mat)), Qt::DirectConnection);
		
	}
	else if (toggled == false)
	{
		disconnect(realsense, SIGNAL(sendFrame(cv::Mat)), head, SLOT(receiveFrame(cv::Mat)));
	}

}

void GUI::on_Pose_Switch_clicked(bool toggled)
{
	if (toggled == true)
	{
		ui.Camera_Switch->setChecked(toggled);
		connect(realsense, SIGNAL(sendFrame(cv::Mat)), pose, SLOT(receiveFrame(cv::Mat)), Qt::DirectConnection);
	}
	else if (toggled == false)
	{
		disconnect(realsense, SIGNAL(sendFrame(cv::Mat)), pose, SLOT(receiveFrame(cv::Mat)));
	}
	/*
	ui.Pose_Switch->hide();

	QLabel* loading_label = new QLabel;
	QMovie* loading_movie = new QMovie(":/Image/Resources/ajax-loader.gif");


	loading_label->setMovie(loading_movie);
	loading_label->setGeometry(ui.Pose_Switch->rect());
	loading_label->setMinimumSize(QSize(640, 480));
	loading_label->setMaximumSize(QSize(640, 480));
	loading_label->setSizeIncrement(QSize(0, 0));
	loading_label->setBaseSize(QSize(0, 0));
	//loading_label->show();
	//loading_movie->start();

	qDebug("Pose starting...");

	qDebug("Pose started...");

	ui.Pose_Switch->show();
	//loading_movie->stop();

	emit Pose_toggled(toggled);


	delete loading_label;
	delete loading_movie;*/

}

GUI::~GUI()
{

	pose->terminate();
	delete pose;

	head->terminate();
	delete head;

	image->terminate();
	delete image;

	realsense->terminate();
	delete realsense;
	
	//delete camera_timer_thread;
	
	//delete &ui;
}
