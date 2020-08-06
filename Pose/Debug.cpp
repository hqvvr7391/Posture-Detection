#include "Debug.h"



void LogToFile(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
	const static QString logtime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh_mm_ss");
	QString path = ("../tmp/");
	QDir dir;
	QFile file(path + logtime + " log.xls");

	if (!dir.exists(path))
		dir.mkpath(path);

	if (!file.open(QIODevice::Append | QIODevice::Text)) {
		
		//std::cout << "Created " << file.fileName().toStdString() << std::endl;
		return;
	}
	QTextStream out(&file);

	QString curtime = QDateTime::currentDateTime().toString("mm:ss,zzz");//QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
	//context.file, context.line, context.function
	switch (type) {
		case QtDebugMsg:
			out << "[Debug]\t" << curtime << "\t" << msg << "\n";
			break;
		case QtInfoMsg:
			out << "[Info]\t" << curtime << "\t" << msg << "\n";
			break;
		case QtWarningMsg:
			out << "[Warning]\t" << curtime << "\t" << msg << "\n";
			break;
		case QtCriticalMsg:
			out << "[Critical]\t" << curtime << "\t" << msg << context.function << context.line << "\n";
			break;
		case QtFatalMsg:
			out << "[Fatal]\t" << curtime << "\t" << msg << "\n";
			abort();
		}
	file.close();
}
