/**
 * This file is part of the rgbdemo project.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Nicolas Burrus <nicolas@burrus.name>, (C) 2010, 2011
 */

#include <sstream>
#include "RawImagesWindow.h"
#include "ui_RawImagesWindow.h"
#include "PoseRecognizer.h"

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

RawImagesWindow::RawImagesWindow(GuiController& controller, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::RawImagesWindow),
    m_controller(controller),
	jointPos(4),
	myRecognizer()
{
  ui->setupUi(this);

  connect(ui->depthView, SIGNAL(mouseMoved(int,int)),
          &m_controller, SLOT(on_depth_mouse_moved(int,int)));
  ui->depthView->setMouseTracking(true);

  ui->action_Show_Object_Detector->setDisabled(true);
  ui->action_3D_View->setDisabled(true);

  if (m_controller.grabber().isSynchronous())
    ui->syncMode->setChecked(true);
}

RawImagesWindow::~RawImagesWindow()
{
  delete ui;
}

void RawImagesWindow :: update(const ntk::RGBDImage& image)
{

	myRecognizer.setSkeleton(image);
	jointPos[0] = image.skeleton()->getProjectedJoint(ntk::Skeleton::NTK_SKEL_HEAD);
	jointPos[1] = image.skeleton()->getProjectedJoint(ntk::Skeleton::NTK_SKEL_NECK);
	jointPos[2] = image.skeleton()->getProjectedJoint(ntk::Skeleton::NTK_SKEL_LEFT_HAND);
	jointPos[3] = image.skeleton()->getProjectedJoint(ntk::Skeleton::NTK_SKEL_RIGHT_HAND);
	int ind = 0;
	
	//update joint readings in joints tableviewwidget
	
	
	for (vector<Point3f>::const_iterator iter = jointPos.begin(); iter != jointPos.end(); ++iter) {
		ind = iter - jointPos.begin();
		QString temp = QString("(%1, %2, %3)").arg(iter->x,0,'f',1).arg(iter->y,0,'f',1).arg(iter->z,0,'f',1);
		if (ui->joints->item(ind,0) == 0) {
			QTableWidgetItem *itab = new QTableWidgetItem;
			itab->setText(temp);
			ui->joints->setItem(ind,0,itab);
		}
		else {
			ui->joints->item(ind,0)->setText(temp);
		}
	} 
	// try to recognize pose and print ui_gesture label
	ui->gesture->setText(QString((myRecognizer.recognizePose()).c_str()));
						 
	if (ui->colorView->isVisible())
		ui->colorView->setImage(image.rgb());

  if (ui->depthView->isVisible())
  {
    double min_dist = m_controller.rgbdProcessor().minDepth();
    double max_dist = m_controller.rgbdProcessor().maxDepth();
    cv::Mat1f masked_distance; image.depth().copyTo(masked_distance);
    apply_mask(masked_distance, image.depthMask());
    cv::Mat3b depth_as_color;
    compute_color_encoded_depth(masked_distance, depth_as_color, &min_dist, &max_dist);
    if (image.skeleton())
      image.skeleton()->drawOnImage(depth_as_color);
    ui->depthView->setImage(depth_as_color);
  }
  // ui->depthView->setImageAsColor(image.depth(), &min_dist, &max_dist);
  // ui->depthView->setImage(image.depth(), &min_dist, &max_dist);
  if (image.userLabels().data && ui->intensityView->isVisible())
  {
    cv::Mat3b user_labels;
    image.fillRgbFromUserLabels(user_labels);
    ui->intensityView->setImage(user_labels);
  }

  int x,y;
  ui->depthView->getLastMousePos(x,y);
  m_controller.on_depth_mouse_moved(x,y);
}

void RawImagesWindow::on_outputDirText_editingFinished()
{
  QString dir = ui->outputDirText->text();
  m_controller.frameRecorder()->setDirectory(dir.toStdString());
}

void RawImagesWindow::on_action_GrabOneFrame_triggered()
{
  m_controller.frameRecorder()->saveCurrentFrame(m_controller.lastImage());
}

void RawImagesWindow::on_action_Quit_triggered()
{
  m_controller.quit();
}

void RawImagesWindow::on_action_Screen_capture_mode_toggled(bool active)
{
  m_controller.setScreenCaptureMode(active);
}

void RawImagesWindow::on_action_GrabFrames_toggled(bool active)
{
  m_controller.setGrabFrames(active);
}

void RawImagesWindow::on_syncMode_toggled(bool checked)
{
  m_controller.grabber().setSynchronous(checked);
  if (checked)
    m_controller.grabber().newEvent();
}

void RawImagesWindow::closeEvent(QCloseEvent *event)
{
  ui->action_Quit->trigger();
  event->accept();
}

void RawImagesWindow::on_actionPause_toggled(bool active)
{
  m_controller.setPaused(active);
}

void RawImagesWindow::on_actionNext_frame_triggered()
{
  m_controller.processOneFrame();
}
