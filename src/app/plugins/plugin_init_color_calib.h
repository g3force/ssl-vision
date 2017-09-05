/*
 * plugin_init_color_calib.h
 *
 *  Created on: Aug 12, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 *      Mark Geiger <markgeiger@posteo.de>
 */

#ifndef SRC_APP_PLUGINS_PLUGIN_INIT_COLOR_CALIB_H_
#define SRC_APP_PLUGINS_PLUGIN_INIT_COLOR_CALIB_H_

#include <visionplugin.h>
#include "cmvision_region.h"
#include "lut3d.h"
#include "field.h"
#include "camera_calibration.h"
#include "plugin_visualize.h"
#include "VarTypes.h"
#include "messages_robocup_ssl_detection.pb.h"

#include <chrono>
#include <QMouseEvent>

#include "BlobDetector.h"

class ColorClazz {
public:
    ColorClazz(unsigned char r, unsigned char g, unsigned char b, int clazz);

    rgb color_rgb;
    yuv color_yuv;
    int clazz;
};

class PluginInitColorCalib : public VisionPlugin {
Q_OBJECT

public:
    PluginInitColorCalib(FrameBuffer *_buffer, LUT3D *lut,
                         const CameraParameters &camera_params, const RoboCupField &field);

    virtual ~PluginInitColorCalib();

    virtual ProcessResult process(FrameData *data, RenderOptions *options);

    virtual VarList *getSettings();

    virtual string getName();

    virtual void mousePressEvent(QMouseEvent *event, pixelloc loc);

    virtual void mouseReleaseEvent(QMouseEvent *event, pixelloc loc);

    virtual void mouseMoveEvent(QMouseEvent *event, pixelloc loc);

protected slots:
    void slotUpdateTriggeredInitial();

private:
    VarList *_settings;
    const CameraParameters &cam_params;
    const RoboCupField &field;
    VarTrigger *_update;
    bool running;
    int nFrames;
    long nSamples;
    LUT3D *global_lut;

    std::vector<ColorClazz> colors;
    float maxColorDist;

    BlobDetector blobDetector;

    VarDouble *drag_x;
    VarDouble *drag_y;

    bool doing_drag;

    void addColorToClazz(FrameData *frame, int x, int y, int clazz);

    bool setDragParamsIfHit(pixelloc loc, VarDouble *x, VarDouble *y);
};

#endif /* SRC_APP_PLUGINS_PLUGIN_INIT_COLOR_CALIB_H_ */
