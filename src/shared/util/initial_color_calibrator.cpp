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
  \file    initial_color_calibrator.cpp
  \brief   C++ Implementation: InitialColorCalibrator
  \author  Mark Geiger <MarkGeiger@posteo.de>, (C) 2017
*/
//========================================================================
#include <framedata.h>
#include <plugins/visionplugin.h>
#include "conversions.h"
#include "image.h"
#include "camera_calibration.h"
#include "lut3d.h"
#include "initial_color_calibrator.h"
#include "color_calibrator.h"

ColorClazz::ColorClazz(const yuv &initColor, int clazz)
        : color_yuv(initColor), clazz(clazz) {
}

void InitialColorCalibrator::addColorToClazz(FrameData *frame,
                                             int x,
                                             int y,
                                             int clazz,
                                             std::vector<ColorClazz> *colors) {
  yuv initColor;
  ColorFormat colorFormat = frame->video.getColorFormat();
  if (colorFormat == COLOR_YUV422_UYVY) {
    initColor = frame->video.getYuv(x, y);
  } else if (colorFormat == COLOR_RGB8) {
    initColor = Conversions::rgb2yuv(frame->video.getRgb(x, y));
  } else {
    std::cerr << "Unsupported source format: " << frame->video.getColorFormat() << std::endl;
  }
  colors->push_back(ColorClazz(initColor, clazz));
}

ProcessResult InitialColorCalibrator::handleInitialCalibration(const FrameData *frame, const RenderOptions *options,
                                                               const CameraParameters &cam_params, LUT3D *global_lut) {
  (void) options;
  if (frame == nullptr)
    return ProcessingFailed;

  std::vector<ColorClazz> colors;
  addColorToClazz((FrameData *) frame,
                  cam_params.additional_calibration_information->init_yellow_x->getInt(),
                  cam_params.additional_calibration_information->init_yellow_y->getInt(),
                  CH_YELLOW,
                  &colors);

  addColorToClazz((FrameData *) frame,
                  cam_params.additional_calibration_information->init_blue_x->getInt(),
                  cam_params.additional_calibration_information->init_blue_y->getInt(),
                  CH_BLUE,
                  &colors);

  addColorToClazz((FrameData *) frame,
                  cam_params.additional_calibration_information->init_pink_x->getInt(),
                  cam_params.additional_calibration_information->init_pink_y->getInt(),
                  CH_PINK,
                  &colors);

  addColorToClazz((FrameData *) frame,
                  cam_params.additional_calibration_information->init_orange_x->getInt(),
                  cam_params.additional_calibration_information->init_orange_y->getInt(),
                  CH_ORANGE,
                  &colors);

  addColorToClazz((FrameData *) frame,
                  cam_params.additional_calibration_information->init_green_x->getInt(),
                  cam_params.additional_calibration_information->init_green_y->getInt(),
                  CH_GREEN,
                  &colors);

  for (int y = 0; y <= 255; y += (0x1u << global_lut->X_SHIFT)) {
    for (int u = 0; u <= 255; u += (0x1u << global_lut->Y_SHIFT)) {
      for (int v = 0; v <= 255; v += (0x1u << global_lut->Z_SHIFT)) {
        yuv color = yuv(static_cast<unsigned char>(y),
                        static_cast<unsigned char>(u),
                        static_cast<unsigned char>(v));
        float minDiff = 1e10;
        int clazz = 0;
        for (auto &j : colors) {
          float diff = 0;
          if (j.clazz == CH_ORANGE) {
            diff = ratedYuvColorDist(color, j.color_yuv, maxColorDist, 2.2);
          } else {
            diff = ratedYuvColorDist(color, j.color_yuv, maxColorDist, 1.0);
          }
          if (diff < minDiff) {
            minDiff = diff;
            clazz = j.clazz;
          }
        }

        if (minDiff < maxColorDist) {
          global_lut->set(color.y, color.u, color.v, static_cast<lut_mask_t>(clazz));
        }
      }
    }
  }
  global_lut->updateDerivedLUTs();

  return ProcessingOk;
}

