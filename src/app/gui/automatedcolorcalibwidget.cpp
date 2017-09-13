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
  \file    automatedcolorcalibwidget.cpp
  \brief   C++ Implementation: AutomatedColorCalibWidget
  \author  Mark Geiger, (C) 2017
*/
//========================================================================


#include "automatedcolorcalibwidget.h"
#include <QCheckBox>
#include <QVBoxLayout>

AutomatedColorCalibWidget::AutomatedColorCalibWidget() {
    /*
    // The calibration points and the fit button:
    QGroupBox *calibrationStepsBox = new QGroupBox(tr("Calibration Steps"));
    QPushButton *initialCalibrationButton = new QPushButton(tr("Do initial calibration"));
    connect(initialCalibrationButton, SIGNAL(clicked()), SLOT(is_clicked_initial()));
    QPushButton *fullCalibrationButton = new QPushButton(tr("Do full calibration"));
    connect(fullCalibrationButton, SIGNAL(clicked()), SLOT(is_clicked_full()));
    QPushButton *additionalPointsButton = new QPushButton(tr("Detect additional calibration points"));
    connect(additionalPointsButton, SIGNAL(clicked()), SLOT(edges_is_clicked()));
    QPushButton *resetButton = new QPushButton(tr("Reset"));
    connect(resetButton, SIGNAL(clicked()), SLOT(is_clicked_reset()));

    QGroupBox *calibrationParametersBox = new QGroupBox(tr("Calibration Parameters"));
    // The slider for the width of the line search corridor:
    QLabel *widthLabel = new QLabel("Line Search Corridor Width (in mm) ");
    lineSearchCorridorWidthSlider = new QSlider(Qt::Horizontal);
    lineSearchCorridorWidthSlider->setMinimum(50);
    lineSearchCorridorWidthSlider->setMaximum(800);
    lineSearchCorridorWidthLabelRight = new QLabel();
    lineSearchCorridorWidthLabelRight->setNum(200);
    connect(lineSearchCorridorWidthSlider, SIGNAL(valueChanged(int)), this, SLOT(line_search_slider_changed(int)));

    QGroupBox *cameraParametersBox = new QGroupBox(tr("Initial Camera Parameters"));
    // The slider for height control:
    QLabel *heightLabel = new QLabel("Camera Height (in mm) ");
    cameraHeightSlider = new QSlider(Qt::Horizontal);
    cameraHeightLabelRight = new QLabel();
    connect(cameraHeightSlider, SIGNAL(valueChanged(int)), this, SLOT(cameraheight_slider_changed(int)));
    // Distortion slider
    QLabel *distortionLabel = new QLabel("Distortion ");
    distortionSlider = new QSlider(Qt::Horizontal);
    distortionLabelRight = new QLabel();
    distortionLabelRight->setNum(1. / 100. * (double) (distortionSlider->value()));
    connect(distortionSlider, SIGNAL(valueChanged(int)), this, SLOT(distortion_slider_changed(int)));

    // Layout for calibration control:
    QVBoxLayout *vbox = new QVBoxLayout;
    vbox->addWidget(initialCalibrationButton);
    vbox->addWidget(additionalPointsButton);
    vbox->addWidget(fullCalibrationButton);
    vbox->addWidget(resetButton);
    vbox->addStretch(1);
    calibrationStepsBox->setLayout(vbox);
    // Layout for calibration parameters
    QGridLayout *gridCalibration = new QGridLayout;
    gridCalibration->addWidget(widthLabel, 0, 0);
    gridCalibration->addWidget(lineSearchCorridorWidthSlider, 0, 1);
    gridCalibration->addWidget(lineSearchCorridorWidthLabelRight, 0, 2);
    calibrationParametersBox->setLayout(gridCalibration);
    // Layout for camera parameters
    QGridLayout *gridCamera = new QGridLayout;
    gridCamera->addWidget(heightLabel, 0, 0);
    gridCamera->addWidget(cameraHeightSlider, 0, 1);
    gridCamera->addWidget(cameraHeightLabelRight, 0, 2);
    gridCamera->addWidget(distortionLabel, 1, 0);
    gridCamera->addWidget(distortionSlider, 1, 1);
    gridCamera->addWidget(distortionLabelRight, 1, 2);
    cameraParametersBox->setLayout(gridCamera);

    // Overall layout:
    QVBoxLayout *vbox2 = new QVBoxLayout;
    vbox2->addWidget(calibrationStepsBox);
    vbox2->addWidget(calibrationParametersBox);
    vbox2->addWidget(cameraParametersBox);
    this->setLayout(vbox2);
     */
}

AutomatedColorCalibWidget::~AutomatedColorCalibWidget() {
    // Destroy GUI here
}

void AutomatedColorCalibWidget::focusInEvent(QFocusEvent *event) {
    (void) event;
}

void AutomatedColorCalibWidget::is_clicked_initial() {
}

void AutomatedColorCalibWidget::is_clicked_full() {
}

void AutomatedColorCalibWidget::is_clicked_reset() {
}

void AutomatedColorCalibWidget::edges_is_clicked() {
}

void AutomatedColorCalibWidget::set_slider_from_vars() {
}

void AutomatedColorCalibWidget::cameraheight_slider_changed(int val) {
}

void AutomatedColorCalibWidget::distortion_slider_changed(int val) {
}

void AutomatedColorCalibWidget::line_search_slider_changed(int val) {
}

