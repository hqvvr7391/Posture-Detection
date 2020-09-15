#pragma once
#ifndef __QML_INT_H_
#define __QML_INT_H_
/// 1.STD libraries
#include <iostream>

	/// 2. OS libraries
	/// 3. 3rd party libraries
#include <opencv2/core.hpp>

	/// 4. Openpose libraries
//#include "Openpose.hpp"


	/// 5. own hpp
#include "Realsense.hpp"
//#include "Image_process.hpp"


#include <QtWidgets/QMainWindow>
#include <qobject.h>
#include <qthread.h>
#include <qtimer.h>
#include <qdebug.h>
#include <qmovie.h>
#include <qquickimageprovider.h>
#include <qcache.h>

#include <qquickpainteditem.h>
#include <qquickitem.h>
#include <qpainter.h>
#include <qimage.h>

class ImageItem : public QQuickPaintedItem
{
	Q_OBJECT
		Q_PROPERTY(QImage image READ image WRITE setImage NOTIFY imageChanged)
public:
	ImageItem(QQuickItem* parent = nullptr);
	Q_INVOKABLE void setImage(const QImage image);
	void paint(QPainter* painter);
	QImage image() const;
	QImage current_image;

signals:
	void imageChanged();

private:
	
};

#else
#endif