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
  \file    initial_color_calibrator.h
  \brief   C++ Interface: InitialColorCalibrator
  \author  Mark Geiger <MarkGeiger@posteo.de>, (C) 2017
*/
//========================================================================
#ifndef INITIAL_COLOR_CALIBRATOR_H
#define INITIAL_COLOR_CALIBRATOR_H


static float ratedYuvColorDist(yuv &c1, yuv &c2, float maxColorDist, float weight) {
  float midToC1U = c1.u - 127;
  float midToC2U = c2.u - 127;
  float midToC1V = c1.v - 127;
  float midToC2V = c2.v - 127;
  float normFac1 = std::sqrt(midToC1U * midToC1U + midToC1V * midToC1V);
  float normFac2 = std::sqrt(midToC2U * midToC2U + midToC2V * midToC2V);
  midToC1U *= 1 / normFac1;
  midToC1V *= 1 / normFac1;
  midToC2U *= 1 / normFac2;
  midToC2V *= 1 / normFac2;
  float scalar = midToC1U * midToC2U + midToC1V * midToC2V;
  float angle = std::acos(scalar);

  float maxAngle = 0.3;
  float bonus = 0;
  if (angle < maxAngle) {
    // give bonus for good angles
    bonus = (1 - (angle / maxAngle)) * maxColorDist * 0.5f;
  } else {
    // give penalty for bad angles
    angle = std::min(90.0f, angle);
    bonus = -maxColorDist * 0.5f * angle / maxAngle;
  }

  float u = c1.u - c2.u;
  float v = c1.v - c2.v;
  float y = c1.y - c2.y;

  float uvDist = u * u + v * v;
  return ((uvDist + y * y) - bonus) * weight;
}


class ColorClazz {
public:
    ColorClazz(const yuv &initColor, int clazz);

    yuv color_yuv;
    int clazz;
};

class InitialColorCalibrator {

private:
    virtual void addColorToClazz(FrameData *frame,
                                 int x,
                                 int y,
                                 int clazz,
                                 std::vector<ColorClazz> *colors);

    float maxColorDist = 3000;

public:
    InitialColorCalibrator() = default;

    ~InitialColorCalibrator() = default;

    ProcessResult handleInitialCalibration(const FrameData *frame,
                                           const RenderOptions *options,
                                           const CameraParameters &cam_params,
                                           LUT3D *global_lut);

};


#endif //INITIAL_COLOR_CALIBRATOR_H
