#ifndef RECORDVIDEO_HH_
#define RECORDVIDEO_HH_

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

int Record_HDs(int argc, char** argv);
int Record_HD_and_FHD(int argc, char** argv);

#endif