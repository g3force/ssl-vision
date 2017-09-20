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
#include <QtGui/QGroupBox>
#include <QtGui/QPushButton>

AutomatedColorCalibWidget::AutomatedColorCalibWidget() {

    QGroupBox *calibrationStepsBox = new QGroupBox(tr("Manual Color-Calibration Steps"));

    QPushButton *initialCalibrationButton = new QPushButton(tr("Do initial calibration"));
    QPushButton *startLearningButton = new QPushButton(tr("Start learning"));
    QPushButton *resteModelButton = new QPushButton(tr("Reset learned model"));
    QPushButton *finishLearningButton = new QPushButton(tr("Finish Learning"));
    QPushButton *updateModelButton = new QPushButton(tr("Update model"));

    connect(initialCalibrationButton, SIGNAL(clicked()), SLOT(is_clicked_initial()));
    connect(startLearningButton, SIGNAL(clicked()), SLOT(is_clicked_start_learning()));
    connect(resteModelButton, SIGNAL(clicked()), SLOT(is_clicked_reset_model()));
    connect(finishLearningButton, SIGNAL(clicked()), SLOT(is_clicked_finish_learning()));
    connect(updateModelButton, SIGNAL(clicked()), SLOT(is_clicked_update_model()));

    auto *boxWrapper = new QVBoxLayout;
    auto *gridLayout = new QGridLayout;
    gridLayout->addWidget(initialCalibrationButton, 0, 0, 1, 1);
    gridLayout->addWidget(resteModelButton, 1, 0, 1, 1);
    gridLayout->addWidget(startLearningButton, 2, 0, 1, 1);
    gridLayout->addWidget(finishLearningButton, 3, 0, 1, 1);
    gridLayout->addWidget(updateModelButton, 4, 0, 1, 1);

    gridLayout->addWidget(new QLabel("Do rough initial calibration: first set markers in camera calibration view"), 0,
                          1,
                          1, 1);
    gridLayout->addWidget(new QLabel("Resets the learned model data"), 1, 1, 1, 1);
    gridLayout->addWidget(new QLabel("Start to gather model data (bots should be detectable)"), 2, 1, 1, 1);
    gridLayout->addWidget(new QLabel("Stop gathering model data"), 3, 1, 1, 1);
    gridLayout->addWidget(new QLabel("Update LUT using the learned model"), 4, 1, 1, 1);

    gridLayout->setColumnStretch(0, 1);
    gridLayout->setColumnStretch(1, 3);

    boxWrapper->addLayout(gridLayout, 1);
    calibrationStepsBox->setLayout(boxWrapper);

    QGroupBox *autoBox = new QGroupBox(tr("Automatic Color-Calibration"));
    auto *vBoxAuto = new QVBoxLayout();
    auto *checkBox = new QCheckBox("Activate automatic online calibration (needs rough initial calibration)");
    connect(checkBox, SIGNAL(stateChanged(int)), SLOT(check_box_state_changed(int)));
    QPushButton *initialCalibrationButton2 = new QPushButton(tr("Do initial calibration"));
    connect(initialCalibrationButton2, SIGNAL(clicked()), SLOT(is_clicked_initial()));
    vBoxAuto->addWidget(initialCalibrationButton2);
    vBoxAuto->addWidget(checkBox);
    vBoxAuto->addWidget(
            new QLabel("If activated, the tool will continuously gather new model data and update the LUT"));
    autoBox->setLayout(vBoxAuto);
    status_label = new QLabel("/");

    QGroupBox *statusBox = new QGroupBox(tr("Status"));
    auto *statusGrid = new QGridLayout;
    statusGrid->addWidget(new QLabel("Status: "), 0, 0, 1, 1);
    statusGrid->addWidget(status_label, 0, 1, 1, 2);
    statusGrid->setColumnStretch(1, 1);
    statusBox->setLayout(statusGrid);

    // Overall layout:
    auto *vbox2 = new QVBoxLayout;
    vbox2->addWidget(calibrationStepsBox);
    vbox2->addStrut(5);
    vbox2->addWidget(autoBox);
    vbox2->addStretch(1);
    vbox2->addWidget(statusBox);
    this->setLayout(vbox2);
}

AutomatedColorCalibWidget::~AutomatedColorCalibWidget() {
    // Destroy GUI here
}

void AutomatedColorCalibWidget::focusInEvent(QFocusEvent *event) {
    (void) event;
}

void AutomatedColorCalibWidget::is_clicked_initial() {
    _is_click_initial = true;
}

void AutomatedColorCalibWidget::is_clicked_start_learning() {
    _is_click_start_learning = true;
}

void AutomatedColorCalibWidget::is_clicked_reset_model() {
    _is_click_reset = true;
}

void AutomatedColorCalibWidget::is_clicked_finish_learning() {
    _is_click_finish_learning = true;
}

void AutomatedColorCalibWidget::is_clicked_update_model() {
    _is_click_update_model = true;
}

void AutomatedColorCalibWidget::check_box_state_changed(int state) {
    _is_automatic_mode_active = static_cast<bool>(state);
}

bool AutomatedColorCalibWidget::is_automatic_mode_active() {
    return _is_automatic_mode_active;
}

bool AutomatedColorCalibWidget::is_click_initial() {
    bool tmp = _is_click_initial;
    _is_click_initial = false;
    return tmp;
}

bool AutomatedColorCalibWidget::is_click_reset() {
    bool tmp = _is_click_reset;
    _is_click_reset = false;
    return tmp;
}

bool AutomatedColorCalibWidget::is_click_start_learning() {
    bool tmp = _is_click_start_learning;
    _is_click_start_learning = false;
    return tmp;
}

bool AutomatedColorCalibWidget::is_click_finish_learning() {
    bool tmp = _is_click_finish_learning;
    _is_click_finish_learning = false;
    return tmp;
}

bool AutomatedColorCalibWidget::is_click_update_model() {
    bool tmp = _is_click_update_model;
    _is_click_update_model = false;
    return tmp;
}

bool AutomatedColorCalibWidget::set_status(std::string status, std::string color) {
    status_label->setText(status.c_str());
    std::string tmp = "QLabel { background-color : red; color : " + color + "; }";
    status_label->setStyleSheet(tmp.c_str());
}
