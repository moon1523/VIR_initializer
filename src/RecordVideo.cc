#include "RecordVideo.hh"

int Record_HDs(int argc, char** argv)
{
	int cameraNum = 3;
	bool isLaptopCam(false);
	if (argc == 3) {
		cameraNum = atoi(argv[2]);
	}
	else if (argc == 4) {
		cameraNum = atoi(argv[2]);
		isLaptopCam = true;
	}
	int fourcc = cv::VideoWriter::fourcc('X','V','I','D');

	vector<cv::VideoCapture> caps;
	vector<cv::VideoWriter> outs;
	vector<cv::Mat> imgs;
	for (int i=0; i<cameraNum; i++) {
		cv::VideoCapture cap;
		int k = i;
		if (isLaptopCam) k = i+1;
		cap.open(k*2, cv::CAP_V4L2);
		cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
		cap.set(cv::CAP_PROP_FRAME_WIDTH,  1280);
		cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
		cap.set(cv::CAP_PROP_AUTOFOCUS, 0);
		cap.set(cv::CAP_PROP_FPS, 15);
		if (!cap.isOpened()) {
			cerr << "Error! Unable to open camera: " << i << endl;
			return 1;
		}
		caps.push_back(cap);

		cv::VideoWriter out;
		out.open("cam" + to_string(i) + ".avi", fourcc, 15, 
				cv::Size(cap.get(cv::CAP_PROP_FRAME_WIDTH), cap.get(cv::CAP_PROP_FRAME_HEIGHT)), true);
		outs.push_back(out);
		cv::Mat img;
		imgs.push_back(img);
	}

	
	while(true) {
		for (int i=0; i<cameraNum; i++) {
			auto now = std::chrono::system_clock::now();
			time_t end_time = std::chrono::system_clock::to_time_t(now);
			caps[i] >> imgs[i];
			putText(imgs[i], string(ctime(&end_time)),
    			Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255.f,255.f,0.f), 2.0);
		}
		for (int i=0; i<cameraNum; i++) {
			outs[i].write(imgs[i]);
		}
		for (int i=0; i<cameraNum; i++) {
			cv::imshow("img" + to_string(i), imgs[i]);
		}

		char key = (char)cv::waitKey(1);
		if (key == 'q') {
			break;
		}
	}
	cv::destroyAllWindows();
	for (int i=0; i<cameraNum; i++) {
		caps[i].release();
	}

	return EXIT_SUCCESS;
}

int Record_HD_and_FHD(int argc, char** argv)
{
	int cameraNum = 2;
	bool isLaptopCam(false);

	if (argc == 3) {
		isLaptopCam = true;
	}
	int fourcc = cv::VideoWriter::fourcc('X','V','I','D');

	vector<cv::VideoCapture> caps;
	vector<cv::VideoWriter> outs;
	vector<cv::Mat> imgs;
	for (int i=0; i<cameraNum; i++) {
		cv::VideoCapture cap;
		int k = i;
		if (isLaptopCam) k = i+1;
		cap.open(k*2, cv::CAP_V4L2);
		cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
		if (i==0) {
			cap.set(cv::CAP_PROP_FRAME_WIDTH,  1280);
			cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
		}
		else {
			cap.set(cv::CAP_PROP_FRAME_WIDTH,  1920);
			cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
		}
		cap.set(cv::CAP_PROP_AUTOFOCUS, 0);
		cap.set(cv::CAP_PROP_FPS, 15);
		if (!cap.isOpened()) {
			cerr << "Error! Unable to open camera: " << i << endl;
			return 1;
		}
		caps.push_back(cap);

		cv::VideoWriter out;
		out.open("cam" + to_string(i) + ".avi", fourcc, 15, 
				cv::Size(cap.get(cv::CAP_PROP_FRAME_WIDTH), cap.get(cv::CAP_PROP_FRAME_HEIGHT)), true);
		outs.push_back(out);
		cv::Mat img;
		imgs.push_back(img);
	}

	
	while(true) {
		for (int i=0; i<cameraNum; i++) {
			auto now = std::chrono::system_clock::now();
			time_t end_time = std::chrono::system_clock::to_time_t(now);
			caps[i] >> imgs[i];
			putText(imgs[i], string(ctime(&end_time)),
    			Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255.f,255.f,0.f), 2.0);
		}
		for (int i=0; i<cameraNum; i++) {
			outs[i].write(imgs[i]);
		}
		for (int i=0; i<cameraNum; i++) {
			cv::imshow("img" + to_string(i), imgs[i]);
		}

		char key = (char)cv::waitKey(1);
		if (key == 'q') {
			break;
		}
	}
	cv::destroyAllWindows();
	for (int i=0; i<cameraNum; i++) {
		caps[i].release();
	}

	return EXIT_SUCCESS;
}

// int Record_Monitor_and_HDcam(int argc, char** argv)
// {
// 	vector<string> serialVec;
// 	uint32_t device_count = k4a_device_get_installed_count();
// 	cout << "Found " << device_count << " connected devices:" << endl;
// 	vector<k4a_device_t> devices;

// 	k4a_capture_t sensorCapture1 = NULL;
// 	k4a_image_t color_image1 = NULL;
// 	k4a_transformation_t transformation1 = NULL;
// 	int image_width1 , image_height1;
// 	k4a_calibration_t sensorCalibration1;

// 	vector<uint8_t> deviceIdx;
// 	for (uint8_t deviceIndex=0; deviceIndex<device_count;deviceIndex++) {
// 		deviceIdx.push_back(deviceIndex);
// 		cout << deviceIndex << endl;
// 	}
// 	// Azure Kinect
// 	//
// 	// Start camera
// 	k4a_device_t device1 = nullptr;
// 	VERIFY(k4a_device_open(deviceIdx[0], &device1), "Open K4A Device failed!");
// 	char *serial_number = NULL;
// 	size_t serial_number_length = 0;
// 	k4a_device_get_serialnum(device1, NULL, &serial_number_length);
// 	serial_number = (char*)malloc(serial_number_length);
// 	k4a_device_get_serialnum(device1, serial_number,&serial_number_length);
// 	k4a_device_configuration_t deviceConfig1 = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
// 	deviceConfig1.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
// 	deviceConfig1.color_resolution = K4A_COLOR_RESOLUTION_720P;
// 	deviceConfig1.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
// 	deviceConfig1.camera_fps = K4A_FRAMES_PER_SECOND_15;
// 	deviceConfig1.subordinate_delay_off_master_usec = 0;
// 	deviceConfig1.synchronized_images_only = true;
// 	VERIFY(k4a_device_start_cameras(device1, &deviceConfig1), "Start K4A cameras failed!");
// 	VERIFY(k4a_device_get_calibration(device1, deviceConfig1.depth_mode, deviceConfig1.color_resolution, &sensorCalibration1), "Get depth camera calibration failed!");
// 	transformation1 = k4a_transformation_create(&sensorCalibration1);
// 	image_width1  = sensorCalibration1.color_camera_calibration.resolution_width;
// 	image_height1 = sensorCalibration1.color_camera_calibration.resolution_height;
// 	cout << serial_number << endl;
	

// 	return 0;


// 	int fourcc = cv::VideoWriter::fourcc('X','V','I','D');
// 	cv::VideoWriter out1_color, out1_depth, out2_color;
// 	out1_color.open("out1_color.avi", fourcc, 15, cv::Size(image_width1, image_height1), true);
// 	// out1_depth.open("out1_depth.avi", fourcc, 15, cv::Size(depth_width1, depth_height1), true);
// 	// out2_color.open("out1_color.avi", fourcc, 15, cv::Size(image_width2, image_height2), true);
// 	while(true)
// 	{
// 		k4a_device_get_capture(device1, &sensorCapture1, 1000);
// 		// k4a_device_get_capture(device2, &sensorCapture2, 1000);
// 		color_image1 = k4a_capture_get_color_image(sensorCapture1);
// 		// depth_image1 = k4a_capture_get_depth_image(sensorCapture1);
// 		// color_image2 = k4a_capture_get_depth_image(sensorCapture2);
// 		// k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, image_width1, image_height1, image_width * 3 * (int)sizeof(uint16_t), &point_image1);
// 		// k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, image_width1, image_height1, image_width1 * (int)sizeof(uint16_t), &colorlike_depth_image1);
// 		// k4a_transformation_depth_image_to_color_camera(transformation1, depth_image1, colorlike_depth_image1);
// 		// k4a_transformation_depth_image_to_point_cloud(transformation1, colorlike_depth_image1, K4A_CALIBRATION_TYPE_COLOR, point_image1);

// 		cv::Mat color_mat1 = color_to_opencv(color_image1);
// 		// cv::Mat depth_mat1 = depth_to_opencv(depth_image1);
// 		// cv::Mat color_mat2 = color_to_opencv(color_image2);

// 		out1_color.write(color_mat1);
// 		// out1_depth.write(depth_mat1);
// 		// out2_color.write(color_mat2);

// 		cv::imshow("color_mat1", color_mat1);
// 		// cv::imshow("depth_mat1", depth_mat1);
// 		// cv::imshow("color_mat2", color_mat2);

// 		char key = (char)cv::waitKey(1);
// 		if (key == 'q') {
// 			break;
// 		}

// 		k4a_capture_release(sensorCapture1);
// 		// k4a_capture_release(sensorCapture2);
// 		k4a_image_release(color_image1);
// 		// k4a_image_release(depth_image1);
// 		// k4a_image_release(color_image2);
// 	}
// 	k4a_transformation_destroy(transformation1);
// 	// k4a_transformation_destroy(transformation2);
// 	k4a_device_close(devices[0]);
// 	// k4a_device_close(devices[1]);
// 	cv::destroyAllWindows();
// 	cout << "k4a record end" << endl;

// 	return EXIT_SUCCESS;
// }