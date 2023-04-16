
#include "SyncCamera.hh"
#include "RecordVideo.hh"
#include "ViewCoords.hh"
#include "BoardCalib.hh"

int main(int argc, char** argv) 
{	
	if (argc > 1 && string(argv[1]) == "-kinect")
		Sync_Kinect(argc, argv);
	else if (argc > 2 && string(argv[1]) == "-webcam")
		Sync_Webcam(argc, argv);
	else if (argc > 2 && string(argv[1]) == "-record")
		Record_HDs(argc,argv);
	else if (argc > 1 && string(argv[1]) == "-record2")
		Record_HD_and_FHD(argc,argv);
	else if (argc > 1 && string(argv[1]) == "-view")
		View_Coordinates(argc, argv);
	else if (argc > 1 && string(argv[1]) == "-write")
		Batch_Write_Camera_Pose(argc, argv);
	else if (argc > 1 && string(argv[1]) == "-calib")
		Calibrate_with_ChArUco(argc, argv);
	else {
		cout << "option: -kinect (-pcd), -webcam [cam idx], -record [cam #] (labtop_cam), -record2 (labtop_cam)"
		        "-view (path), -write (iso_file), -calib (output_name)" << endl;
	}
	return EXIT_SUCCESS;
}