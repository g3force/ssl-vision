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
  \file    color_calibrator.h
  \brief   Common code for color calibrators
  \author  Nicolai Ommer <nicolai.ommer@gmail.com>, (C) 2016
*/
//========================================================================
#ifndef SSL_VISION_COLOR_CALIBRATOR_H
#define SSL_VISION_COLOR_CALIBRATOR_H

#include "geometry.h"

#define CH_DARK_GREEN 1
#define CH_ORANGE 2
#define CH_YELLOW 3
#define CH_BLUE 4
#define CH_PINK 5
#define CH_GREEN 7

class LocLabeled {
public:
    pixelloc loc;
    int clazz;
};


class BotPosStamped {
public:
    double time;
    vector3d pos;
    double orientation;
};

#endif //SSL_VISION_COLOR_CALIBRATOR_H
