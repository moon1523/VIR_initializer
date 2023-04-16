#ifndef SYNCCAMERA_HH_
#define SYNCCAMERA_HH_

#include <k4a/k4a.h>
#include <k4a/k4a.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <malloc.h>

#include "CoordMatch.hh"
#include "Functions.hh"

using namespace std;

int Sync_Kinect(int argc, char** argv);
int Sync_Webcam(int argc, char** argv);
int Batch_Write_Camera_Pose(int argc, char** argv);

#endif