/**
 PoseRecognizer.h: processes and tracks skeleton structure to identify
 specific poses, translating them to interpreted gestures.
 Author: David Pietrocola
 Date: August 19, 2011
 */

#include <sstream>
#include "PoseRecognizer.h"
#include "ui_RawImagesWindow.h"

#include "GuiController.h"

#include <ntk/camera/rgbd_frame_recorder.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/gesture/skeleton.h>
#include <ntk/utils/opencv_utils.h>
#ifdef NESTK_USE_FREENECT
# include <ntk/camera/freenect_grabber.h>
#endif

#include <QCloseEvent>

using namespace ntk;
using namespace cv;
using namespace std;

PoseRecognizer::PoseRecognizer() :
	jointPos(6),
	epsilon(20)
{ }

PoseRecognizer::~PoseRecognizer()
{
	// Delete stuff....
	//delete ui;
}

void PoseRecognizer :: setSkeleton(const ntk::RGBDImage& image)
{
	if (image.skeleton()) {
	
		jointPos[0] = image.skeleton()->getProjectedJoint(ntk::Skeleton::NTK_SKEL_LEFT_SHOULDER);
		jointPos[1] = image.skeleton()->getProjectedJoint(ntk::Skeleton::NTK_SKEL_LEFT_ELBOW);
		jointPos[2] = image.skeleton()->getProjectedJoint(ntk::Skeleton::NTK_SKEL_LEFT_HAND);
		jointPos[3] = image.skeleton()->getProjectedJoint(ntk::Skeleton::NTK_SKEL_RIGHT_SHOULDER);
		jointPos[4] = image.skeleton()->getProjectedJoint(ntk::Skeleton::NTK_SKEL_RIGHT_ELBOW);
		jointPos[5] = image.skeleton()->getProjectedJoint(ntk::Skeleton::NTK_SKEL_RIGHT_HAND);
	}
}

string PoseRecognizer :: recognizePose() {
		// coordinate convention is (0,0) in upper left corner
		isLeftArmStraightOut = ((jointPos[2].x < jointPos[1].x) && (jointPos[1].x < jointPos[0].x)) && (abs(jointPos[1].y-jointPos[2].y)<epsilon);
		isRightArmStraightOut = ((jointPos[4].x < jointPos[5].x) && (jointPos[3].x < jointPos[4].x)) && (abs(jointPos[4].y-jointPos[5].y)<epsilon);
		isLeftArmPerpendicular = ((jointPos[2].y < jointPos[1].y) && (jointPos[1].x < jointPos[0].x)) && (abs(jointPos[1].x-jointPos[2].x)<epsilon);
		isRightArmPerpendicular = ((jointPos[5].y < jointPos[4].y) && (jointPos[3].x < jointPos[4].x)) && (abs(jointPos[4].x-jointPos[5].x)<epsilon);
		// Determine gesture.
		// Todo: generalize
		left = isLeftArmPerpendicular && isRightArmStraightOut;
		right = isLeftArmStraightOut && isRightArmPerpendicular;
		forward = isLeftArmStraightOut && isRightArmStraightOut;
		stop = isLeftArmPerpendicular && isRightArmPerpendicular;
		
		if (left)
			pose = "LEFT";
		else if (right)
			pose = "RIGHT";
		else if (forward)
			pose = "FORWARD";
		else if (stop)
			pose = "STOP";
		else
			pose = "N/A";
	return pose;
}
	
	
	

