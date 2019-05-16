//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
  \file    plugin_auto_color_calibration.cpp
  \brief   C++ Implementation: PluginAutoColorCalibration
  \author  Nicolai Ommer <nicolai.ommer@gmail.com>, (C) 2016
           Mark Geiger <markgeiger@posteo.de>, (C) 2017
*/
//========================================================================

#include "plugin_auto_color_calibration.h"
#include <opencv2/opencv.hpp>
#include <gui/automatedcolorcalibwidget.h>

PluginAutoColorCalibration::PluginAutoColorCalibration(
        FrameBuffer *_buffer,
        LUT3D *lut,
        const CameraParameters &camera_params,
        const RoboCupField &field)
        :
        VisionPlugin(_buffer),
        global_lut(lut),
        camera_parameters(camera_params) {

  onlineColorCalibrator = new OnlineColorCalibrator(lut, camera_params, field);

  _settings = new VarList("Auto Color Calibration");
  _settings->addChild(_v_debug = new VarBool("debug", false));
  _settings->addChild(onlineColorCalibrator->_v_removeOutlierBlobs);

  // get mouse events
  this->installEventFilter(this);
}


PluginAutoColorCalibration::~PluginAutoColorCalibration() {
  delete onlineColorCalibrator;
}


QWidget *PluginAutoColorCalibration::getControlWidget() {
  if (_accw == nullptr)
    _accw = new AutomatedColorCalibWidget();

  return (QWidget *) _accw;
}


ProcessResult PluginAutoColorCalibration::process(FrameData *frame, RenderOptions *options) {
  (void) options;
  if (frame == nullptr) {
    return ProcessingFailed;
  }

  // handle GUI commands here
  process_gui_commands();

  // run initial calibration
  if (initial_calib_running) {
    ProcessResult result = initialCalibrator.handleInitialCalibration(frame, options, camera_parameters,
                                                                      global_lut);
    if (result == ProcessingOk) {
      processed_frames++;
      if (processed_frames > 5) {
        processed_frames = 0;
        initial_calib_running = false;
      }
    }
  }

  // run online calibration
  ColorFormat source_format = frame->video.getColorFormat();
  if (enabled) {

    if (source_format != COLOR_YUV422_UYVY && source_format != COLOR_RGB8) {
      std::cerr << "Unsupported source format: " << source_format << std::endl;
      enabled = false;
      _accw->set_status("Unsupported source format");
      return ProcessingFailed;
    }

    onlineColorCalibrator->update(frame);

    if (_v_debug->getBool()) {
      Image<raw8> *img_debug;
      if ((img_debug = (Image<raw8> *) frame->map.get("cmv_online_color_calib")) == nullptr) {
        img_debug = (Image<raw8> *) frame->map.insert("cmv_online_color_calib", new Image<raw8>());
      }
      img_debug->allocate(frame->video.getWidth(), frame->video.getHeight());
      img_debug->fillColor(0);

      onlineColorCalibrator->mutex_locs.lock();
      for (auto ll : onlineColorCalibrator->locs) {
        if (ll.clazz >= 0)
          img_debug->setPixel(ll.loc.x, ll.loc.y, static_cast<raw8>(onlineColorCalibrator->cProp[ll.clazz].color));
        else
          img_debug->setPixel(ll.loc.x, ll.loc.y, 1);
      }
      onlineColorCalibrator->mutex_locs.unlock();
    }

    Image<raw8> *img_thresholded;
    if ((img_thresholded = (Image<raw8> *) frame->map.get("cmv_learned_threshold")) == nullptr) {
      img_thresholded = (Image<raw8> *) frame->map.insert("cmv_learned_threshold", new Image<raw8>());
    }
    img_thresholded->allocate(frame->video.getWidth(), frame->video.getHeight());
    if (frame->video.getColorFormat() == COLOR_RGB8) {
      auto *rgblut = (RGBLUT *) onlineColorCalibrator->local_lut.getDerivedLUT(CSPACE_RGB);
      if (rgblut == nullptr) {
        std::cerr << "WARNING: No RGB LUT has been defined." << std::endl;
      } else {
        CMVisionThreshold::thresholdImageRGB(img_thresholded, &(frame->video), rgblut);
      }
    } else if (frame->video.getColorFormat() == COLOR_YUV422_UYVY) {
      CMVisionThreshold::thresholdImageYUV422_UYVY(img_thresholded, &(frame->video), &onlineColorCalibrator->local_lut);
    } else {
      std::cerr << "Unsupported source format for learned threshold: " << source_format << std::endl;
    }
  }

  return ProcessingOk;
}

VarList *PluginAutoColorCalibration::getSettings() {
  return _settings;
}

string PluginAutoColorCalibration::getName() {
  return "AutoColorCalibration";
}

void PluginAutoColorCalibration::process_gui_commands() {
  if (_accw == nullptr) {
    return;
  }
  if (_accw->is_click_initial()) {
    processed_frames = 0;
    initial_calib_running = true;
    _accw->set_status("Triggered initial calibration");
  }
  if (_accw->is_click_start_learning()) {
    enabled = true;
    _accw->set_status("Triggered start learning");
  }
  if (_accw->is_click_finish_learning()) {
    enabled = false;
    _accw->set_status("Triggered finish learning");
  }
  if (_accw->is_click_update_model()) {
    onlineColorCalibrator->globalLutUpdate = true;
    onlineColorCalibrator->CopyToLUT(global_lut);
    _accw->set_status("Model updated");
  }
  if (_accw->is_click_reset()) {
    onlineColorCalibrator->ResetModel();
    _accw->set_status("Model reset");
  }

  if (_accw->is_automatic_mode_active()) {
    _accw->set_status("Automatic mode active");
    onlineColorCalibrator->liveUpdate = true;
    enabled = true;
  } else {
    if (onlineColorCalibrator->liveUpdate) {
      _accw->set_status("Automatic mode deactivated");
      enabled = false;
    }
    onlineColorCalibrator->liveUpdate = false;
  }
}


void PluginAutoColorCalibration::mousePressEvent(QMouseEvent *event, pixelloc loc) {

  std::vector<VarDouble *> ax;
  std::vector<VarDouble *> ay;

  ax.push_back(camera_parameters.additional_calibration_information->init_yellow_x);
  ay.push_back(camera_parameters.additional_calibration_information->init_yellow_y);
  ax.push_back(camera_parameters.additional_calibration_information->init_blue_x);
  ay.push_back(camera_parameters.additional_calibration_information->init_blue_y);
  ax.push_back(camera_parameters.additional_calibration_information->init_green_x);
  ay.push_back(camera_parameters.additional_calibration_information->init_green_y);
  ax.push_back(camera_parameters.additional_calibration_information->init_pink_x);
  ay.push_back(camera_parameters.additional_calibration_information->init_pink_y);
  ax.push_back(camera_parameters.additional_calibration_information->init_orange_x);
  ay.push_back(camera_parameters.additional_calibration_information->init_orange_y);

  if ((event->buttons() & Qt::LeftButton) != 0) {
    drag_x = nullptr;
    drag_y = nullptr;

    for (size_t i = 0; i < ax.size(); i++) {
      if (setDragParamsIfHit(loc, ax[i], ay[i])) {
        break;
      }
    }
    if (drag_x != nullptr && drag_y != nullptr) {
      event->accept();
      doing_drag = true;
    } else {
      event->ignore();
    }
  } else
    event->ignore();
}

bool PluginAutoColorCalibration::setDragParamsIfHit(pixelloc loc, VarDouble *x, VarDouble *y) {
  double drag_threshold = 20; //in px
  const double x_diff = x->getDouble() - loc.x;
  const double y_diff = y->getDouble() - loc.y;
  if (sqrt(x_diff * x_diff + y_diff * y_diff) < drag_threshold) {
    // found a point
    drag_x = x;
    drag_y = y;
    return true;
  }
  return false;
}

void PluginAutoColorCalibration::mouseReleaseEvent(QMouseEvent *event, pixelloc loc) {
  (void) loc;
  doing_drag = false;
  event->accept();
}

void PluginAutoColorCalibration::mouseMoveEvent(QMouseEvent *event, pixelloc loc) {
  if (doing_drag && (event->buttons() & Qt::LeftButton) != 0) {
    if (loc.x < 0) loc.x = 0;
    if (loc.y < 0) loc.y = 0;
    drag_x->setDouble(loc.x);
    drag_y->setDouble(loc.y);
    event->accept();
  } else {
    event->ignore();
  }
}