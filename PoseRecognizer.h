/**
PoseRecognizer.h: processes and tracks skeleton structure to identify
specific poses, translating them to interpreted gestures.
Author: David Pietrocola
Date: August 19, 2011
 */

#ifndef POSERECOGNIZER_H
#define POSERECOGNIZER_H

#include <ntk/camera/calibration.h>
#include <iostream>
//#include <QMainWindow>
#include <ntk/utils/opencv_utils.h>
#include <string>	

using namespace cv;

class PoseRecognizer
{

public:
    PoseRecognizer();
    ~PoseRecognizer();
	void setSkeleton(const ntk::RGBDImage& image);
	std::string recognizePose();

private:
    bool isLeftArmStraightOut;
	bool isRightArmStraightOut;
	bool isLeftArmPerpendicular;
	bool isRightArmPerpendicular;
    //GuiController& m_controller;
	vector<cv::Point3f> jointPos;
	bool left,right,forward,stop;
	std::string pose;
	int epsilon;
	

    //friend class GuiController;
};

#endif // POSERECOGNIZER_H
