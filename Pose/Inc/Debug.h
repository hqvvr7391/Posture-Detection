#pragma once
#include <qdebug.h>
#include <qdatetime.h>
#include <qfile.h>
#include <qdir.h>

#include <iostream>


enum Thread {    // 열거형 정의
	em_GUI = 1,         // 초깃값 할당
#define GUI_MSG	QString("\t").repeated(em_GUI).toStdString().c_str() << "GUI "
	em_QML,
#define QML_MSG	QString("\t").repeated(em_QML).toStdString().c_str() << "GUI "
	em_Realsense,
#define RS_MSG	QString("\t").repeated(em_Realsense).toStdString().c_str() << "Realsense "
	em_Image_process,
#define IM_MSG	QString("\t").repeated(em_Image_process).toStdString().c_str() << "Image "
	em_Head,
#define HD_MSG	QString("\t").repeated(em_Head).toStdString().c_str() << "Head "
	em_Pose
#define PS_MSG	QString("\t").repeated(em_Pose).toStdString().c_str() << "Pose"

};

void LogToFile(QtMsgType type, const QMessageLogContext& context, const QString& msg);