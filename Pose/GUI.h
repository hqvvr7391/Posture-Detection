#pragma once
#ifndef __GUI_H_
#define __GUI_H_

	/// 1.STD libraries
#include <iostream>

	/// 2. OS libraries
	/// 3. 3rd party libraries
#include <opencv2/core.hpp>

	/// 4. Openpose libraries
#include "Openpose.hpp"


	/// 5. own hpp
#include "Realsense.hpp"
#include "Image_process.hpp"


#include <QtWidgets/QMainWindow>
#include <qthread.h>
#include <qtimer.h>
#include <qdebug.h>
#include <qmovie.h>
#include "ui_GUI.h"

class GUI : public QMainWindow
{
	Q_OBJECT

public:
	GUI(QWidget *parent = Q_NULLPTR);
	~GUI();

	void setup();
		
signals:
	void Pose_toggled(bool toggled);

public slots:
	void receiveFrame(cv::Mat image);
	void receiveFrame2(cv::Mat image);
	void on_Camera_Switch_clicked(bool toggled);
	void on_Face_Switch_clicked(bool toggled);
	void on_Pose_Switch_clicked(bool toggled);


private:
	Ui::GUIClass ui;
	
    Realsense* realsense;
	QThread* image;
	Head* head;
	Body* body;
};

#else
#endif