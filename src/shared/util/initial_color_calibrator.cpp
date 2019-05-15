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

#define CH_ORANGE 2
#define CH_YELLOW 3
#define CH_BLUE 4
#define CH_PINK 5
#define CH_GREEN 7

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

static yuv getColorFromImage(RawImage *img, int x, int y) {
    yuv color;
    uyvy color2 = *((uyvy *) (img->getData()
                              + (sizeof(uyvy)
                                 * (((y * (img->getWidth())) + x) / 2))));
    color.u = color2.u;
    color.v = color2.v;
    if ((x % 2) == 0) {
        color.y = color2.y1;
    } else {
        color.y = color2.y2;
    }
    return color;
}

ColorClazz::ColorClazz(unsigned char r, unsigned char g, unsigned char b, int clazz)
        : color_rgb(r, g, b), clazz(clazz) {
    color_yuv = Conversions::rgb2yuv(color_rgb);
}

InitialColorCalibrator::InitialColorCalibrator()
        : maxColorDist(3000) {
}

InitialColorCalibrator::~InitialColorCalibrator() = default;

void InitialColorCalibrator::addColorToClazz(FrameData *frame, int x, int y, int clazz, std::vector<ColorClazz> *colors) {
  rgb initColorRGB;
  ColorFormat colorFormat = frame->video.getColorFormat();
  if(colorFormat == COLOR_YUV422_UYVY) {
    yuv initColor = getColorFromImage(&frame->video, x, y);
    initColorRGB = Conversions::yuv2rgb(initColor);
  } else if(colorFormat == COLOR_RGB8) {
    rgbImage rgb_img(frame->video);
    rgb * color_rgb=rgb_img.getPixelData();
    color_rgb += y * frame->video.getWidth() + x;
    initColorRGB = *color_rgb;
  } else {
    std::cerr << "Unsupported source format: " << frame->video.getColorFormat() << std::endl;
  }
  colors->emplace_back(initColorRGB.r, initColorRGB.g, initColorRGB.b, clazz);
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

    for (int y = 0; y <= 255; y += (0x1 << global_lut->X_SHIFT)) {
        for (int u = 0; u <= 255; u += (0x1 << global_lut->Y_SHIFT)) {
            for (int v = 0; v <= 255; v += (0x1 << global_lut->Z_SHIFT)) {
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

