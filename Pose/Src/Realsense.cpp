#include "Realsense.hpp"

rs2::frameset _data;
rs2_intrinsics intr;
rs2::depth_frame depthframe = _data.get_depth_frame();

QMutex global_mutex;

Realsense::Realsense(QObject* parent) :
	QThread(parent),
	status(false)
{
		/*rs2::context ctx;
		auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
		if (list.size() == 0)
			throw std::runtime_error("No device detected. Is it plugged in?");
		rs2::device dev = list.front();*/

		//dev.hardware_reset();
		
		m_stop = false;

		pipe = new rs2::pipeline();
		cfg = new rs2::config();
		color_map = new rs2::colorizer();
		ali = new rs2::align(RS2_STREAM_COLOR);

		cfg->enable_stream(RS2_STREAM_COLOR, frame_width, frame_height, RS2_FORMAT_RGB8, 30);
		//cfg.enable_stream(RS2_STREAM_INFRARED, frame_width, frame_height, RS2_FORMAT_Y8, 30);
		cfg->enable_stream(RS2_STREAM_DEPTH, frame_width, frame_height, RS2_FORMAT_Z16, 30);
		cfg->enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
		cfg->enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
		// Start streaming with default recommended configuration
		 
		color_map->set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_DEFAULT);
		color_map->set_option(RS2_OPTION_MAX_DISTANCE, 2.f);

		//pipe->get_active_profile().operator bool
		qInfo() << RS_MSG << "Initialized";
		

}


void Realsense::Get_Frame()
{
	qDebug() << RS_MSG << "wait";
	//pipe->poll_for_frames(&frame_data);
	frame_data = pipe->wait_for_frames();
	qDebug() << RS_MSG << "align";
	frame_data = ali->process(frame_data);
}

void Realsense::receiveGrabFrame()
{

	Color_image = Realsense::Get_color();

	cv::Mat dp_image;
	dp_image = cv::Scalar(0);

	//cv::cvtColor(this->Color_image, dp_image, COLOR_BGR2RGB);

	QImage output((const unsigned char*)Color_image.data, Color_image.cols, Color_image.rows, QImage::Format_RGB888);

}


cv::Mat Realsense::Get_color()
{

	cv::Mat image_rgb(cv::Size(frame_width, frame_height), CV_8UC3, (void*)frame_data.get_color_frame().get_data(), cv::Mat::AUTO_STEP);
	
	return image_rgb;
}

cv::Mat Realsense::Get_depth()
{
	color_map->set_option(RS2_OPTION_COLOR_SCHEME, 2); // White to Black
	// Create OpenCV matrix of size (w,h) from the colorized depth data

	cv::Mat image_depth(cv::Size(frame_width, frame_height), CV_8UC3, (void*)depthframe.apply_filter(*this->color_map).get_data(), cv::Mat::AUTO_STEP);

	cv::Mat image_gray;
	cv::cvtColor(image_depth, image_gray, cv::COLOR_BGR2GRAY);

	return image_gray;
}

cv::Mat Realsense::Get_color_depth()
{
	color_map->set_option(RS2_OPTION_COLOR_SCHEME, 0);	// Jet  
	// Create OpenCV matrix of size (w,h) from the colorized depth data

	cv::Mat image_depth(cv::Size(frame_width, frame_height), CV_8UC3, (void*)depthframe.apply_filter(*this->color_map).get_data(), cv::Mat::AUTO_STEP);

	return image_depth;// image_depth;
}


void Realsense::run()
{
	qInfo() << RS_MSG << this->currentThreadId();

	while (1)
	{
		//mutex.lock();
		if (m_stop)
		{
			
			this->pipe->stop();
			this->msleep(500);
			status = false;
			qInfo() << RS_MSG << "stopped";
			//mutex.unlock();
			break;
		}
		//mutex.unlock();


		qDebug() << RS_MSG << "start";
		global_mutex.lock();
		qDebug() << RS_MSG << "mutex";
		Get_Frame();
		qDebug() << RS_MSG << "Frame";
		Get_motion();
		qDebug() << RS_MSG << "Motion";
		
		

	
		if (request)
		{
			qDebug() << RS_MSG << "frame sended";
			if (this->isRunning())
			{
				receiveGrabFrame();
				qDebug() << RS_MSG << "grabbed";
				depthframe = frame_data.get_depth_frame();
				intr = depthframe.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
				
			}

			//QImage output((const unsigned char*)Color_image.data, Color_image.cols, Color_image.rows, QImage::Format_RGB888);
			//cv::Mat output = this->Color_image.clone();
			request = false;
			emit sendFrame_M(Color_image);
		}
		global_mutex.unlock(); 
					

	} 
	//qDebug("Realsense stop...");
}

void Realsense::request_image()
{
	request = true;/*
	global_mutex.lock();
	qDebug() << RS_MSG << "frame sended";
	if (this->isRunning())
	{
		depthframe = frame_data.get_depth_frame();
		intr = depthframe.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
	}
	else
	{

		this->Color_image = cv::Mat(cv::Scalar(0));
	}
	
	//QImage output((const unsigned char*)Color_image.data, Color_image.cols, Color_image.rows, QImage::Format_RGB888);
	//cv::Mat output = this->Color_image.clone();
	emit sendFrame_M(Color_image);
	global_mutex.unlock();*/

}

void Realsense::onoff(bool toggled)
{

	qInfo() <<RS_MSG << "toggled";
	if (toggled == true)
	{

		/*rs2::context ctx;
		auto list2 = ctx.query_devices(); // Get a snapshot of currently connected devices
		if (list2.size() == 0)
			throw std::runtime_error("No device detected. Is it plugged in?");*/

		qInfo() << RS_MSG << "on";

		mutex.lock();
		m_stop = false;
		mutex.unlock();
		this->pipe->start(*this->cfg);

		this->msleep(500);
		
		status = true;
		this->start();
		this->setPriority(HighPriority);
		qInfo() << RS_MSG << "pipe started";

	}

	else if(toggled == false)
	{
		qInfo() << RS_MSG << "off";
		mutex.lock();
		m_stop = true;
		mutex.unlock();

		this->quit();

		qInfo() <<RS_MSG << "quitted";
	}
}

Eigen::Vector3f Realsense::Get_motion()
{
	Eigen::Vector3f theta;
	static std::mutex theta_mtx;
	/* alpha indicates the part that gyro and accelerometer take in computation of theta; higher alpha gives more weight to gyro, but too high
	values cause drift; lower alpha gives more weight to accelerometer, which is more sensitive to disturbances */
	const float alpha = 0.95;
	static bool first = true;
	// Keeps the arrival time of previous gyro frame
	static double last_ts_gyro = 0;

	//auto motion = frame_data.as<rs2::motion_frame>();
	rs2::motion_frame motion = frame_data.first_or_default(RS2_STREAM_GYRO);

	// If casting succeeded and the arrived frame is from gyro stream
	if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
	{
		// Get the timestamp of the current frame
		double ts = motion.get_timestamp();
		// Get gyro measures
		rs2_vector gyro_data = motion.get_motion_data();
		// Call function that computes the angle of motion based on the retrieved measures

		if (first)	// On the first iteration, use only data from accelerometer to set the camera's initial position
		{
			last_ts_gyro = ts;
		}

		else
		{
			// Holds the change in angle, as calculated from gyro
			Eigen::Vector3f gyro_angle;

			// Initialize gyro_angle with data from gyro
			gyro_angle(P_X) = gyro_data.x; // Pitch
			gyro_angle(P_Y) = gyro_data.y; // Yaw
			gyro_angle(P_Z) = gyro_data.z; // Roll

				// Compute the difference between arrival times of previous and current gyro frames
			double dt_gyro = (ts - last_ts_gyro) / 1000.0;
			last_ts_gyro = ts;

			// Change in angle equals gyro measures * time passed since last measurement
			gyro_angle = gyro_angle * dt_gyro;

			// Apply the calculated change of angle to the current angle (theta)
			std::lock_guard<std::mutex> lock(theta_mtx);
			theta += Eigen::Vector3f(-gyro_angle(P_Z), -gyro_angle(P_Y), gyro_angle(P_X));
		}
	}


	motion = frame_data.first_or_default(RS2_STREAM_ACCEL);


	// If casting succeeded and the arrived frame is from accelerometer stream
	if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
	{
		// Get accelerometer measures
		rs2_vector accel_data = motion.get_motion_data();
		// Call function that computes the angle of motion based on the retrieved measures
		Eigen::Vector3f accel_angle;

		// Calculate rotation angle from accelerometer data
		accel_angle(P_Z) = atan2(accel_data.y, accel_data.z);
		accel_angle(P_X) = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));

		// If it is the first iteration, set initial pose of camera according to accelerometer data (note the different handling for Y axis)
		std::lock_guard<std::mutex> lock(theta_mtx);
		if (first)
		{
			first = false;
			theta = accel_angle;
			// Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convetion for the initial pose
			theta(P_Y) = M_PI;
		}
		else
		{
			/*
			Apply Complementary Filter:
				- high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals
					that are steady over time, is used to cancel out drift.
				- low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations
			*/
			theta(P_X) = theta(P_X) * alpha + accel_angle(P_X) * (1 - alpha);
			theta(P_Z) = theta(P_Z) * alpha + accel_angle(P_Z) * (1 - alpha);
		}
	}

	std::lock_guard<std::mutex> lock(theta_mtx);
	return theta;
}

Realsense::~Realsense()
{
	if (m_stop == false)
		onoff(false);

	delete color_map;
	delete ali;
	delete cfg;
	delete pipe;

	this->terminate();

	qInfo() << RS_MSG << "Destoyed";
}



/*
void get_point(dlib::full_object_detection* shape, sp_point* sp)
{
	std::vector<Eigen::Vector2f> d2_points;
	std::vector<Eigen::Vector3f> d3_points;

	float upixel[2];
	float upoint[3];
	float udist;

	for (int i = 0; i < sp->point.size(); i++)
	{
		upixel[P_X] = shape->part(sp->point.at(i)).x();
		upixel[P_Y] = shape->part(sp->point.at(i)).y();

		d2_points.push_back(Eigen::Vector2f(upixel[P_X], upixel[P_Y]));

		udist = depthframe.get_distance(upixel[P_X], upixel[P_Y]);
		rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);
		d3_points.push_back(Eigen::Vector3f(upoint[P_X], upoint[P_Y], upoint[P_Z]));
	}

	sp->point2D.swap(d2_points);
	sp->point3D.swap(d3_points);

	vector<Eigen::Vector2f>().swap(d2_points);
	vector<Eigen::Vector3f>().swap(d3_points);
}

void get_point(op::Array<float> shape, sp_point* sp)
{

	std::vector<Eigen::Vector2f> d2_points;
	std::vector<Eigen::Vector3f> d3_points;

	float upixel[2];
	float upoint[3];
	float udist;

	for (int i = 0; i < sp->point.size(); i++)
	{
		int point = sp->point.at(i) * 3;

		upixel[P_X] = shape[point];
		upixel[P_Y] = shape[point + 1];

		d2_points.push_back(Eigen::Vector2f(upixel[P_X], upixel[P_Y]));

		udist = depthframe.get_distance(upixel[P_X], upixel[P_Y]);
		rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);
		d3_points.push_back(Eigen::Vector3f(upoint[P_X], upoint[P_Y], upoint[P_Z]));
	}
	sp->point2D.swap(d2_points);
	sp->point3D.swap(d3_points);
}*/