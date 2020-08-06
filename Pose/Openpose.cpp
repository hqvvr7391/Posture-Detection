#include "Openpose.hpp"

#include <openpose/flags.hpp>



Body::Body(QObject* parent) :
	QThread(parent)
{
	
	image = cv::Mat(640, 480, CV_8UC3, cv::Scalar(0, 0, 0));

	qInfo() << BD_MSG << "Initialized";
}


void Body::op_Init()
{

	qInfo() << BD_MSG << "Configuring OpenPose...";

	const auto opTimer = op::getTimerInit();


	FLAGS_disable_blending = true;
	FLAGS_face = false;
	FLAGS_hand = false;
	FLAGS_hand_detector = 0;
	//FLAGS_disable_multi_thread = true;
	FLAGS_logging_level = 0;


	const auto outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
	// netInputSize
	const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");
	// faceNetInputSize
	//const auto faceNetInputSize = op::flagsToPoint(FLAGS_face_net_resolution, "368x368 (multiples of 16)");
	// handNetInputSize
	const auto handNetInputSize = op::flagsToPoint(FLAGS_hand_net_resolution, "368x368 (multiples of 16)");
	// poseMode
	const auto poseMode = op::flagsToPoseMode(FLAGS_body);
	// poseModel
	const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);
	// JSON saving
	if (!FLAGS_write_keypoint.empty())
		std::cout << "Flag `write_keypoint` is deprecated and will eventually be removed."
			" Please, use `write_json` instead." << std::endl;
	// keypointScaleMode
	const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);
	// heatmaps to add
	const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
		FLAGS_heatmaps_add_PAFs);
	const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
	// >1 camera view?
	const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1);
	// Face and hand detectors
	//const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
	const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);
	// Enabling Google Logging
	const bool enableGoogleLogging = true;

	// Pose configuration (use WrapperStructPose{} for default and recommended configuration)
	const op::WrapperStructPose wrapperStructPose{
		poseMode, netInputSize, outputSize, keypointScaleMode, FLAGS_num_gpu, FLAGS_num_gpu_start,
		FLAGS_scale_number, (float)FLAGS_scale_gap, op::flagsToRenderMode(FLAGS_render_pose, multipleView),
		poseModel, !FLAGS_disable_blending, (float)FLAGS_alpha_pose, (float)FLAGS_alpha_heatmap,
		FLAGS_part_to_show, FLAGS_model_folder, heatMapTypes, heatMapScaleMode, FLAGS_part_candidates,
		(float)FLAGS_render_threshold, FLAGS_number_people_max, FLAGS_maximize_positives, FLAGS_fps_max,
		FLAGS_prototxt_path, FLAGS_caffemodel_path, (float)FLAGS_upsampling_ratio, enableGoogleLogging };
	opWrapper->configure(wrapperStructPose);
	// Face configuration (use op::WrapperStructFace{} to disable it)
	//const op::WrapperStructFace wrapperStructFace{
	//	FLAGS_face, faceDetector, faceNetInputSize,
	//	op::flagsToRenderMode(FLAGS_face_render, multipleView, FLAGS_render_pose),
	//	(float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap, (float)FLAGS_face_render_threshold };
	//opWrapper.configure(wrapperStructFace);
	// Hand configuration (use op::WrapperStructHand{} to disable it)
	const op::WrapperStructHand wrapperStructHand{
		FLAGS_hand, handDetector, handNetInputSize, FLAGS_hand_scale_number, (float)FLAGS_hand_scale_range,
		op::flagsToRenderMode(FLAGS_hand_render, multipleView, FLAGS_render_pose), (float)FLAGS_hand_alpha_pose,
		(float)FLAGS_hand_alpha_heatmap, (float)FLAGS_hand_render_threshold };
	opWrapper->configure(wrapperStructHand);
	// Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
	const op::WrapperStructExtra wrapperStructExtra{
		FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, FLAGS_tracking, FLAGS_ik_threads };
	opWrapper->configure(wrapperStructExtra);
	// Output (comment or use default argument to disable any output)
	const op::WrapperStructOutput wrapperStructOutput{
		FLAGS_cli_verbose, FLAGS_write_keypoint, op::stringToDataFormat(FLAGS_write_keypoint_format),
		FLAGS_write_json, FLAGS_write_coco_json, FLAGS_write_coco_json_variants, FLAGS_write_coco_json_variant,
		FLAGS_write_images, FLAGS_write_images_format, FLAGS_write_video, FLAGS_write_video_fps,
		FLAGS_write_video_with_audio, FLAGS_write_heatmaps, FLAGS_write_heatmaps_format, FLAGS_write_video_3d,
		FLAGS_write_video_adam, FLAGS_write_bvh, FLAGS_udp_host, FLAGS_udp_port };
	opWrapper->configure(wrapperStructOutput);


	// Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
	if (FLAGS_disable_multi_thread)
		opWrapper->disableMultiThreading();

}

void Body::Get_data()
{
	
	const auto imageToProcess = image;
	
	cv::Mat output = cv::Mat(640, 480, CV_8UC3, cv::Scalar(0, 0, 0));

	if (opWrapper->isRunning() && !imageToProcess.empty())
	{
		
		auto datumProcessed = opWrapper->emplaceAndPop(imageToProcess);
		
		if (datumProcessed != nullptr)
		{
			
			//std::cout <<"Body keypoints: 
			//" + datumProcessed->at(0)->poseKeypoints.toString()<< std::endl;
			Datum = datumProcessed;
			output = datumProcessed->at(0)->cvOutputData;
			//return datumProcessed;
		}
		else
		{
			
			//return 0;
		}
	
		if (Datum->at(0)->poseKeypoints.empty() == false)
		{
			get_point(Datum->at(0)->poseKeypoints, &neck);
			get_point(Datum->at(0)->poseKeypoints, &shoulder_left);
			get_point(Datum->at(0)->poseKeypoints, &shoulder_right);

			this->detect = EXIT_SUCCESS;
		}
		else
		{
			point_initialize(&neck);
			point_initialize(&shoulder_left);
			point_initialize(&shoulder_right);
			this->detect = EXIT_FAILURE;
		}
		
	}
	this->image = output;
	//emit sendFrame(output);
	emit sendF();
	qDebug() << BD_MSG << "send";
}

void Body::run()
{

	qInfo() << BD_MSG <<  QThread::currentThreadId();

	if (running == true)
	{
		qInfo() << BD_MSG << "starting opWrapper";

		opWrapper = new op::Wrapper{ op::ThreadManagerMode::Asynchronous };
		this->msleep(100);

		op_Init();
		qInfo() << BD_MSG << "opWrapper thread start";

		this->msleep(100);
		opWrapper->start();

		this->sleep(5);

		status = true;

		emit wrapper_onoff(status);
		qInfo() << BD_MSG << "opWrapper started";
	}


	while (status)
	{
		
		mutex.lock();
		m_imageCondition.wait(&mutex);
		mutex.unlock();
		qDebug() << BD_MSG << "start";
		Get_data();
		/******* emit sendFrame(output); **********/
	}

	if (opWrapper->isRunning() && !running)
	{
		qInfo() << BD_MSG << "opWrapper thread stop";

		opWrapper->stop();
		while (opWrapper->isRunning());
		delete opWrapper;
		this->sleep(3);
		emit wrapper_onoff(status);
		std::cout << "awr" << std::endl;
		qInfo() << BD_MSG << "opWrapper thread stopped";
	}

	qInfo() << BD_MSG << "ended...";
}

void Body::receiveFrame()
{

	//qDebug() << BD_MSG << " pre-received image";
	mutex.lock();
	m_imageCondition.wakeOne();
	mutex.unlock();
	//qDebug() << BD_MSG << " awake";
	global_mutex.lock();
	this->image = Base_image;
	global_mutex.unlock();

	qDebug() << BD_MSG << " received image";
}


void Body::onoff(bool toggled)
{

	qInfo() << BD_MSG << QThread::currentThreadId();
	if (toggled == true)
	{
		mutex.lock();
		running = true;
		mutex.unlock();
		
		this->start();
		//this->setPriority(HighPriority);
		qInfo() << BD_MSG << "on";
	}
	else
	{
		mutex.lock();
		m_imageCondition.wakeAll();
		mutex.unlock();

		mutex.lock();
		running = false;
		status = false;
		mutex.unlock();
		this->quit();
		qInfo() << BD_MSG << "off";
	}
}

Body::~Body()
{
	running = false;
	this->quit();
	while (this->isRunning());
	qInfo() << BD_MSG << "destroying";
	mutex.lock();
	m_imageCondition.wakeAll();
	mutex.unlock();
	qInfo() << BD_MSG << "wakeall";



	//delete opWrapper;
	qInfo() << BD_MSG << "Destroyed";
}