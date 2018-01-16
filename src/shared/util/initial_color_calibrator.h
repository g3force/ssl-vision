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


class ColorClazz {
public:
    ColorClazz(unsigned char r, unsigned char g, unsigned char b, int clazz);

    rgb color_rgb;
    yuv color_yuv;
    int clazz;
};

class InitialColorCalibrator {

private:
    virtual void addColorToClazz(FrameData *frame, int x, int y, int clazz, std::vector<ColorClazz> *colors);

    float maxColorDist;

public:
    InitialColorCalibrator();
    virtual ~InitialColorCalibrator();

    virtual ProcessResult handleInitialCalibration(const FrameData *frame, const RenderOptions *options,
                                                   const CameraParameters &cam_params, LUT3D *global_lut);

};


#endif //INITIAL_COLOR_CALIBRATOR_H
