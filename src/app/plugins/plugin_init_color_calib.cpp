/*
 * plugin_init_color_calib.cpp
 *
 *  Created on: Aug 12, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 *      Mark Geiger <markgeiger@posteo.de>
 */

#include "plugin_init_color_calib.h"

PluginInitColorCalib::PluginInitColorCalib(FrameBuffer *_buffer,
                                           LUT3D *lut, const CameraParameters &camera_params,
                                           const RoboCupField &field) :
        VisionPlugin(_buffer),
        cam_params(camera_params), field(field) {

}

PluginInitColorCalib::~PluginInitColorCalib() {
}


void PluginInitColorCalib::mousePressEvent(QMouseEvent *event, pixelloc loc) {
    std::vector<VarDouble *> ax;
    std::vector<VarDouble *> ay;

    ax.push_back(
            cam_params.additional_calibration_information->init_yellow_x);
    ay.push_back(
            cam_params.additional_calibration_information->init_yellow_y);
    ax.push_back(
            cam_params.additional_calibration_information->init_blue_x);
    ay.push_back(
            cam_params.additional_calibration_information->init_blue_y);
    ax.push_back(
            cam_params.additional_calibration_information->init_green_x);
    ay.push_back(
            cam_params.additional_calibration_information->init_green_y);
    ax.push_back(
            cam_params.additional_calibration_information->init_pink_x);
    ay.push_back(
            cam_params.additional_calibration_information->init_pink_y);
    ax.push_back(
            cam_params.additional_calibration_information->init_orange_x);
    ay.push_back(
            cam_params.additional_calibration_information->init_orange_y);

    if ((event->buttons() & Qt::LeftButton) != 0) {
        drag_x = 0;
        drag_y = 0;

        for (int i = 0; i < ax.size(); i++) {
            if (setDragParamsIfHit(loc,
                                   ax[i],
                                   ay[i])) {
                break;
            }
        }
        if (drag_x != 0 && drag_y != 0) {
            event->accept();
            doing_drag = true;
        } else {
            event->ignore();
        }
    } else
        event->ignore();
}

bool PluginInitColorCalib::setDragParamsIfHit(pixelloc loc, VarDouble *x, VarDouble *y) {
    double drag_threshold = 20; //in px
    const double x_diff =
            x->getDouble() - loc.x;
    const double y_diff =
            y->getDouble() - loc.y;
    if (sqrt(x_diff * x_diff + y_diff * y_diff) < drag_threshold) {
        // found a point
        drag_x = x;
        drag_y = y;
        return true;
    }
    return false;
}

void PluginInitColorCalib::mouseReleaseEvent(QMouseEvent *event, pixelloc loc) {
    (void) loc;
    doing_drag = false;
    event->accept();
}

void PluginInitColorCalib::mouseMoveEvent(QMouseEvent *event, pixelloc loc) {
    if (doing_drag && (event->buttons() & Qt::LeftButton) != 0) {
        if (loc.x < 0) loc.x = 0;
        if (loc.y < 0) loc.y = 0;
        drag_x->setDouble(loc.x);
        drag_y->setDouble(loc.y);
        event->accept();
    } else
        event->ignore();
}
