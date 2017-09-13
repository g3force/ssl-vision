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
  \file    automatedcolorcalibwidget.h
  \brief   C++ Interface: AutomatedColorCalibWidget
  \author  Mark Geiger, (C) 2017
*/
//========================================================================

#ifndef AUTOMATEDCOLORCALIBWIDGET_H
#define AUTOMATEDCOLORCALIBWIDGET_H

#include <QWidget>
#include <QSlider>
#include <QLabel>

class AutomatedColorCalibWidget : public QWidget {
Q_OBJECT

public:
    void focusInEvent ( QFocusEvent * event );
    AutomatedColorCalibWidget();
    ~AutomatedColorCalibWidget();

    void set_slider_from_vars();

protected:
    QSlider *lineSearchCorridorWidthSlider;
    QLabel *lineSearchCorridorWidthLabelRight;
    QSlider *cameraHeightSlider;
    QLabel *cameraHeightLabelRight;
    QSlider *distortionSlider;
    QLabel *distortionLabelRight;

public slots:
    void is_clicked_initial();
    void is_clicked_full();
    void is_clicked_reset();
    void edges_is_clicked();
    void cameraheight_slider_changed(int val);
    void distortion_slider_changed(int val);
    void line_search_slider_changed(int val);
};

#endif
