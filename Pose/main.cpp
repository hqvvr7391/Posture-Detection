
#include <gflags/gflags.h>

#include "Qml_integrator.hpp"
//#include "GUI.h"
#include "Debug.h"


#include <QtWidgets/QApplication>
#include <QGuiApplication>
#include <QApplication>
#include <QQuickWidget>
#include <QQmlApplicationEngine>
#include <QSystemtrayicon>
#include <QQmlContext>
#include <QIcon>

#include <qsystemtrayicon.h>


Q_DECLARE_METATYPE(QSystemTrayIcon::ActivationReason)
Q_DECLARE_METATYPE(QSystemTrayIcon::MessageIcon)


int main(int argc, char* argv[])
{
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	qInstallMessageHandler(LogToFile);

	QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

	QApplication app(argc, argv);


	QQmlApplicationEngine engine;
	const QUrl url(QStringLiteral("qrc:/GUI/Resources/Qml/main.qml"));

	app.setOrganizationName("asdf");
	app.setOrganizationDomain("sdfa");
	app.setApplicationName("capste");
	
	//QSettings settings; //default constructor
	
	QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
		&app, [url](QObject* obj, const QUrl& objUrl) {
			if (!obj && url == objUrl)
				QCoreApplication::exit(-1);
		}, Qt::QueuedConnection);


	qmlRegisterType<QSystemTrayIcon>("QSystemTrayIcon", 1, 0, "QSystemTrayIcon");
	qRegisterMetaType<QSystemTrayIcon::ActivationReason>("ActivationReason");
	qRegisterMetaType<QSystemTrayIcon::MessageIcon>("MessageIcon");

	qmlRegisterType<ImageItem>("myextension", 1, 0, "ImageItem");
	qmlRegisterType<FboInSGRenderer>("OpenGLUnderQML", 1, 0, "Display");
	qmlRegisterType<ThreadRenderer>("OpenGLUnderQML", 1, 0, "Renderer");

	//qmlRegisterType<QML_Integrator>("QML_integrator", 1, 0, "QML_GUI");
	qmlRegisterType<Image_process>("Image_process", 1, 0, "Image_proc");

	qmlRegisterType<Realsense>("librealsense2", 1, 0, "Realsense");
	//qRegisterMetaType <realsense_setting>("realsense_setting");

	qmlRegisterType<Head>("Headpose", 1, 0, "Head");
	qmlRegisterType<Body>("Openpose", 1, 0, "Body");
	qmlRegisterType<Pose>("Pose_estimation", 1, 0, "Pose");
	qmlRegisterType<RobotArm>("RobotArm", 1, 0, "RobotArm");

	engine.rootContext()->setContextProperty("settings", new Settings());
	engine.rootContext()->setContextProperty("iconTray", QIcon(":/Image/Resources/image/tray_icon.png"));

	engine.load(url);

	/*QObject* topLevel = engine.rootObjects().value(0);
	QQuickWindow* window = qobject_cast<QQuickWindow*>(topLevel);

	window->setPersistentOpenGLContext(true);
	window->setPersistentSceneGraph(true);
	window->show();*/
	
	auto execReturn = app.exec();
	

	for (QThread* t : qAsConst(ThreadRenderer::threads)) {
		t->wait();

		delete t;
	}
	return execReturn;
}

/*

int main(int argc, char *argv[])
{
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	qInstallMessageHandler(LogToFile);
	qRegisterMetaType<cv::Mat>();
	QApplication a(argc, argv);

	GUI w;
	w.show();
	return a.exec();
}
*/