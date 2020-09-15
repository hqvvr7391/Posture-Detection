#include "Headpose.hpp"


//dlib::frontal_face_detector detector;
//dlib::shape_predictor pose_model;

Head::Head(QObject* parent) :
	QThread(parent)
{
	this->detector = dlib::get_frontal_face_detector();
	dlib::deserialize("models/face/shape_predictor_68_face_landmarks.dat") >> this->pose_model;

	m_stop = true;
	state = false;
}

void Head::onoff(bool toggled)
{
	
	if (toggled == true)
	{
		qInfo() << HD_MSG << "toggled on";
		this->start();
	}
	else if (toggled == false)
	{
		qInfo() << HD_MSG << "toggled off";
		this->quit();
	}
	emit thread_onoff(toggled);
	
}

void Head::run()
{
	qInfo() << HD_MSG << QThread::currentThreadId();
	while (1)
	{
		qDebug() << HD_MSG << "start";

		mutex.lock();
		m_imageCondition.wait(&mutex);
		mutex.unlock();
		

		Head_detection();
		/*****  emit sendFrame(output_image); ****/
		qDebug() << HD_MSG << "sended image";
		
	}
}

void Head::Head_detection()
{
	/*
	Size winSize(11, 11);
	Mat status, err;
	std::vector<Mat> prevPyr, currPyr;

	if (prev_img.empty())
	{
		prev_img.copyTo(image_rotated);
	}

	buildOpticalFlowPyramid(prev_img, prevPyr, winSize, 3, true);
	buildOpticalFlowPyramid(image_rotated, currPyr, winSize, 3, true);
	*/

	//////////////// Image rotation for better face recognition //////////////////                                                                                                                                                         
	Mat base_image = this->image;

	cv::Mat output_image;
	output_image.zeros(cv::Size(image.cols, image.rows), CV_8UC3);
	
	Mat image_gray;
	
	cvtColor(base_image, image_gray, COLOR_BGR2GRAY);
	Mat hist_item;
	equalizeHist(image_gray, hist_item);

	Ptr<CLAHE> clahe = createCLAHE(6.0, Point(8, 8));
	clahe->apply(image_gray, hist_item);


	
	Mat image_rotated;
	Point image_center(base_image.size().width / 2, base_image.size().height / 2);

	Mat matrix = getRotationMatrix2D(image_center, this->rotation_angle, 1);
	warpAffine(hist_item, image_rotated, matrix, base_image.size());
	
	//matrix = getRotationMatrix2D(image_center, - this->rotation_angle, 1);		//  Get reverse rotation Matrix 
	//warpAffine(image_rotated, *image, matrix, image->size());


	dlib::cv_image<uchar> cimg(image_rotated);											// Grayscale image detection
	//dlib::cv_image<dlib::bgr_pixel> cimg(image);											// B G R	 image detection
	
	//////////////// face recognition  //////////

	this->faces = detector(cimg);
	
	//////////////////TODO : find main face

	this->shapes.clear();
	for (unsigned long i = 0; i < this->faces.size(); ++i)
		this->shapes.push_back(pose_model(cimg, this->faces[i]));

	////
	
	
	
	if (this->faces.size() >= 1)
	{
		this->face = this->shapes.at(0);
		
		float angle = radToDeg(atan2f(
			this->face.part(this->eye_right.point.at(1)).y() - this->face.part(this->eye_left.point.at(1)).y(),
			this->face.part(this->eye_right.point.at(0)).x() - this->face.part(this->eye_left.point.at(0)).x()))
			+ this->rotation_angle;

		pixel_rearrange(&this->face, this->rotation_angle, image_center);

		// Get face's specific point in 2D, 3D
		get_point(&this->face, &this->eye_right);
		get_point(&this->face, &this->eye_left);
		get_point(&this->face, &this->mouse);
		get_point(&this->face, &this->nose);

		Head_pose();
		
		this->rotation_angle = angle;

		for (unsigned int j = 0; j < this->face.num_parts(); ++j)
			circle(output_image, Point(this->face.part(j).x(), this->face.part(j).y()), 5, Scalar(255, 255, 255), -1, 8);

		char no[25];
		sprintf(no, "%.3f", nose.point3D.at(0)(P_Z)*100);
		putText(output_image, no, Point(nose.point2D.at(0)(P_X), nose.point2D.at(0)(P_Y)), 2, 0.75, Scalar(255, 255, 0));
		
		this->state = EXIT_SUCCESS;
	}
	else
	{
		this->state = EXIT_FAILURE;
	}

	emit sendFrame(output_image);

}

void Head::Head_pose() {

	// Find middle point of eyes and mouse

	std::vector<Vector3f> eyeline = slope_3d(this->eye_left.point3D.at(0), this->eye_right.point3D.at(0));
	std::vector<Vector3f> mouseline = slope_3d(this->mouse.point3D.at(0), this->mouse.point3D.at(1));
	Vector3f nose_p;

	//  Find face's center point through vector projection of nose point

	nose_p = nose_projection(eyeline.at(0), mouseline.at(0), nose.point3D);

	// Get an orthogonal matrix 

	this->coordination = coordinate(nose_p, nose.point3D.at(0), eyeline.at(0));


	Vec3f euler_angle = rotationMatrixToEulerAngle(this->coordination);

		
	this->x = this->coordination(0, 3);
	this->y = this->coordination(1, 3);
	this->z = this->coordination(2, 3);

	this->roll = euler_angle(0);
	this->pitch = euler_angle(2);
	this->yaw = euler_angle(1);

	/*
	char no[255];
	sprintf(no, "%.3f %.3f", this->roll, this->yaw);


	float pixel[2];
	float a[3];
	a[0] = lefteye.point3D.at(0).x;
	a[1] = lefteye.point3D.at(0).y;
	a[2] = lefteye.point3D.at(0).z;
	rs2_project_point_to_pixel(pixel, &intr, a);

	sprintf(no, "%.2f %.2f %.2f", lefteye.point3D.at(0).x * 100, lefteye.point3D.at(0).y * 100, lefteye.point3D.at(0).z * 100);
	putText(*image, no, Point(shape.part(34).x() - 100, shape.part(34).y() - 100), 2, 0.75, Scalar(255, 255, 0));
	*/
		

	/*******************
		
	dlib head pose algorithm
		
	*******************/

	/*
	std::vector<cv::Point3d> model_points = get_3d_model_points();
	std::vector<cv::Point2d> image_points = get_2d_image_points(shape);
	double focal_length = image->cols;
	cv::Mat camera_matrix = get_camera_matrix(focal_length, cv::Point2d(image->cols / 2, image->rows / 2));
	cv::Mat rotation_vector;
	cv::Mat rotation_matrix;
	cv::Mat translation_vector;
	cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type);

	cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);
	cv::Rodrigues(rotation_vector, rotation_matrix);
	Vec3f rotationb = rotationMatrixToEulerAngles(rotation_matrix);
	aruco::drawAxis(* image, camera_matrix, dist_coeffs, rotation_vector, translation_vector, 300.0);

	*/
	//image_rotated.copyTo(prev_img);
}
	
void Head::receiveFrame(cv::Mat received_image)
{
	this->image = received_image;
	qDebug() << HD_MSG << "received image";

	mutex.lock();
	m_imageCondition.wakeOne();
	mutex.unlock();

}

Head::~Head()
{
	mutex.lock();
	m_imageCondition.wakeAll();
	mutex.unlock();
	this->terminate();
	qInfo() << HD_MSG << "Destroyed";
}


void pixel_rearrange(dlib::full_object_detection* shape, float angle, cv::Point center)
{
	int crx;
	int cry;

	angle = degToRad(angle);

	for (unsigned int i = 0; i < shape->num_parts(); ++i)
	{
		crx = shape->part(i).x() - center.x;
		cry = shape->part(i).y() - center.y;
		float arx = cosf(angle) * crx - sinf(angle) * cry;
		float ary = sinf(angle) * crx + cosf(angle) * cry;
		shape->part(i).x() = ceil(arx + center.x);
		shape->part(i).y() = ceil(ary + center.y);
	}
}


Vector3f nose_projection(Vector3f eye_point, Vector3f mouse_point, std::vector<Vector3f> nose)
{
	Vector3f a_vector;
	Vector3f b_vector;

	a_vector = eye_point - mouse_point;
	b_vector = nose.at(0) - mouse_point;

	Vector3f n_vector = (a_vector.dot(b_vector)) / a_vector.dot(a_vector) * a_vector + mouse_point;

	//cout << n_vector << endl;
	//cout << "    " << endl;	
	return n_vector;
}

Matrix4f coordinate(Vector3f Center_point, Vector3f x_point, Vector3f y_point, Vector3f z_point)
{
	Vector3f axis_X = x_point - Center_point;
	Vector3f axis_Y = y_point - Center_point;
	Vector3f axis_Z;

	if (z_point == Vector3f::Zero())
	{
		Vector3f tmp_x = x_point + Center_point;
		Vector3f tmp_y = y_point + Center_point;
		axis_Z = axis_Y.cross(axis_X);
	}
	else
	{
		axis_Z = z_point - Center_point;
	}

	axis_X = unit_vector(axis_X);
	axis_Y = unit_vector(axis_Y);
	axis_Z = unit_vector(axis_Z);

	Matrix4f coordinate;

	coordinate << 
		axis_X[P_X], axis_Y[P_X], axis_Z[P_X], Center_point[P_X],
		axis_X[P_Y], axis_Y[P_Y], axis_Z[P_Y], Center_point[P_Y],
		axis_X[P_Z], axis_Y[P_Z], axis_Z[P_Z], Center_point[P_Z],
		0, 0, 0, 1;
	Matrix4f coordinat = coordinate.transpose();
	return coordinate;
}

std::vector<cv::Point3d> get_3d_model_points()
{
	std::vector<cv::Point3d> modelPoints;

	modelPoints.push_back(cv::Point3d(0.0f, 0.0f, 0.0f)); //The first must be (0,0,0) while using POSIT
	modelPoints.push_back(cv::Point3d(0.0f, -330.0f, -65.0f));
	modelPoints.push_back(cv::Point3d(-225.0f, 170.0f, -135.0f));
	modelPoints.push_back(cv::Point3d(225.0f, 170.0f, -135.0f));
	modelPoints.push_back(cv::Point3d(-150.0f, -150.0f, -125.0f));
	modelPoints.push_back(cv::Point3d(150.0f, -150.0f, -125.0f));

	return modelPoints;

}

std::vector<cv::Point2d> get_2d_image_points(dlib::full_object_detection& d)
{
	std::vector<cv::Point2d> image_points;
	image_points.push_back(cv::Point2d(d.part(30).x(), d.part(30).y()));    // Nose tip
	image_points.push_back(cv::Point2d(d.part(8).x(), d.part(8).y()));      // Chin
	image_points.push_back(cv::Point2d(d.part(36).x(), d.part(36).y()));    // Left eye left corner
	image_points.push_back(cv::Point2d(d.part(45).x(), d.part(45).y()));    // Right eye right corner
	image_points.push_back(cv::Point2d(d.part(48).x(), d.part(48).y()));    // Left Mouth corner
	image_points.push_back(cv::Point2d(d.part(54).x(), d.part(54).y()));    // Right mouth corner
	return image_points;

}

cv::Mat get_camera_matrix(float focal_length, cv::Point2d center)
{
	cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << focal_length, 0, center.x, 0, focal_length, center.y, 0, 0, 1);
	return camera_matrix;
}

bool isRotationMatrix(Mat& R)
{
	Mat Rt;
	transpose(R, Rt);
	Mat shouldBeIdentity = Rt * R;
	Mat I = Mat::eye(3, 3, shouldBeIdentity.type());

	return  norm(I, shouldBeIdentity) < 1e-6;

}

Vec3f rotationMatrixToEulerAngle(Matrix4f R)
{

	float sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));

	bool singular = sy < 1e-6; // If

	float x, y, z;
	if (!singular)
	{
		/*x = atan2f(R(2, 1), R(2, 2));
		y = atan2f(-R(2, 0), sy);
		z = atan2f(R(1, 0), R(0, 0));*/

		x = atan2f(R(1, 2), R(0, 2));
		y = atan2f(-R(2, 2), sy);
		z = atan2f(R(2, 1), R(2, 0));
	}
	else
	{
		x = 0;
		y = atan2f(-R(0, 2), sy);
		z = atan2f(-R(1, 0), R(1, 1));
	}

	//cout << radToDeg(z) << "\t\t" << radToDeg(y) << "\t\t" << radToDeg(x) << "\t" << endl;
	return Vec3f(x, y, z);

}

Vec3f rotationMatrixToEulerAngles(Mat& R)
{

	assert(isRotationMatrix(R));

	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

	bool singular = sy < 1e-6; // If

	float x, y, z;
	if (!singular)
	{
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	}
	else
	{
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}

	//cout << radToDeg(x) << "\t\t" << radToDeg(y) << "\t\t" << radToDeg(z) << "\t" << endl;
	return Vec3f(x, y, z);
}


