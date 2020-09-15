#include "Openpose.hpp"

#include <openpose/flags.hpp>

Pose::Pose(QObject* parent) :
	QThread(parent)
{
	opWrapper = new op::Wrapper{ op::ThreadManagerMode::Asynchronous };
	op_Init();
	qInfo() << PS_MSG << "Initialized";
	image = cv::Mat(640, 480, CV_8UC3, cv::Scalar(0, 0, 0));
}


void Pose::op_Init()
{

	std::cout <<"Starting OpenPose demo..."<< std::endl;
	const auto opTimer = op::getTimerInit();

	std::cout <<"Configuring OpenPose..."<< std::endl;
	FLAGS_disable_blending = true;
	FLAGS_face = false;
	FLAGS_hand = true;
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

void Pose::Get_data()
{
	//std::cout <<"3-1 : pose get data" << std::endl;

	const auto imageToProcess = image;

	cv::Mat output;
	output = cv::Scalar(0);

	if (opWrapper->isRunning() && !imageToProcess.empty())
	{
		//std::cout <<"3-2 : pose emplace" << std::endl;
		auto datumProcessed = opWrapper->emplaceAndPop(imageToProcess);
		//std::cout <<"3-3 : pose poped" << std::endl;
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
			//std::cout <<"Camera could not be processed." << std::endl;
			//return 0;
		}
		//std::cout <<"3-4 : pose transmit data" << std::endl;



		/*if (Datum->at(0)->poseKeypoints.empty() == 0)
		{
			//get_point(Datum->at(0)->poseKeypoints, &neck);
			//get_point(Datum->at(0)->poseKeypoints, &shoulder_left);
			//get_point(Datum->at(0)->poseKeypoints, &shoulder_right);
		}*/
	}
	
	//std::cout <<"3-4 : pose data" << std::endl;
	
	emit sendFrame(output);
}

void Pose::run()
{

	qInfo() << PS_MSG <<  QThread::currentThreadId();
	while (1)
	{
		qDebug() << PS_MSG << "start";

		mutex.lock();
		if (!opWrapper->isRunning() && !m_stop)
		{
			qDebug() << PS_MSG << "opWrapper thread start";
			opWrapper->start();
			qDebug() << PS_MSG << "opWrapper thread started";
			this->sleep(3);
			emit thread_onoff(true);
		}
		mutex.unlock();


		mutex.lock();
		m_imageCondition.wait(&mutex);
		mutex.unlock();

		qDebug() << PS_MSG << "Pose get data";
		Get_data();
		/******* emit sendFrame(output); **********/



		if (opWrapper->isRunning() && m_stop)
		{
			qDebug() << PS_MSG << "opWrapper thread stop";
			emit thread_onoff(false);
			opWrapper->stop();
			while (opWrapper->isRunning());
			this->sleep(3);
			qDebug() << PS_MSG << "opWrapper thread stopped";
		}

		qDebug() << PS_MSG << "ended...";
	}

}

void Pose::receiveFrame(cv::Mat received_image)
{
	/*
	qDebug() << PS_MSG << " pre-received image";
	mutex.lock();
	m_imageCondition.wakeAll();
	mutex.unlock();
	qDebug() << PS_MSG << " awake";
	received_image.copyTo(image);
	qDebug() << PS_MSG << " received image";*/
	
}


void Pose::onoff(bool toggled)
{

	qDebug() << PS_MSG << "toggled";
	if (toggled == true)
	{
		mutex.lock();
		m_stop = false;
		mutex.unlock();
		
		this->start();
		this->setPriority(HighPriority);

	}
	else
	{
		mutex.lock();
		m_stop = true;
		mutex.unlock();
		this->quit();
		qDebug() << PS_MSG << "ended";
	}
}

Pose::~Pose()
{

	this->quit();

	qInfo() << PS_MSG << "destroying";
	mutex.lock();
	m_imageCondition.wakeAll();
	mutex.unlock();
	qInfo() << PS_MSG << "wakeall";


	if (opWrapper->isRunning())
	{
		opWrapper->stop();
	}
	while (opWrapper->isRunning());
	qInfo() << PS_MSG << "opWrapper stopped";



	delete opWrapper;
	qInfo() << PS_MSG << "Destroyed";
}