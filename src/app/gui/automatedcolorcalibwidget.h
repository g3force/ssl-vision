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

private:
    QLabel *status_label;
    bool _is_automatic_mode_active;
    bool _is_click_initial;
    bool _is_click_reset;
    bool _is_click_start_learning;
    bool _is_click_finish_learning;
    bool _is_click_update_model;

public:
    AutomatedColorCalibWidget();

    ~AutomatedColorCalibWidget();

    bool is_automatic_mode_active();

    bool is_click_initial();

    bool is_click_reset();

    bool is_click_start_learning();

    bool is_click_finish_learning();

    bool is_click_update_model();

    bool set_status(std::string status);

    void focusInEvent(QFocusEvent *event);

public slots:
    void is_clicked_initial();

    void is_clicked_start_learning();

    void is_clicked_reset_model();

    void is_clicked_finish_learning();

    void is_clicked_update_model();

    void check_box_state_changed(int state);
};

#endif
