/*
 * plugin_online_color_calib.h
 *
 *  Created on: Jul 19, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 *      Mark Geiger <markgeiger@posteo.de>
 */

#ifndef SRC_APP_PLUGINS_PLUGIN_ONLINE_COLOR_CALIB_H_
#define SRC_APP_PLUGINS_PLUGIN_ONLINE_COLOR_CALIB_H_

#include <visionplugin.h>
#include "cmvision_region.h"
#include "lut3d.h"
#include "field.h"
#include "camera_calibration.h"
#include "plugin_visualize.h"
#include "VarTypes.h"
#include "messages_robocup_ssl_detection.pb.h"

#include <chrono>
#include <lwpr.hh>

#include <QThread>
#include <QObject>
#include <QString>

#include <mutex>
#include <condition_variable>

#include <gui/automatedcolorcalibwidget.h>

#include "blob_detector.h"
#include "initial_color_calibrator.h"

class LocLabeled {
public:
    pixelloc loc;
    int8_t clazz;
};

class BotPosStamped {
public:
    double time;
    vector3d pos;
    double orientation;
};

class AngleRange {
public:
    double min;
    double max;
};

class ClazzProperties {
public:
    double minDist;
    double maxDist;
    double height;
    double radius;
    int color;
    int nAngleRanges;
    AngleRange angleRanges[2];
};

class WorkerInput {
public:
    long long int number;
    RawImage image;
    SSL_DetectionFrame detection_frame;
    std::vector<CMVision::Region> regions;
};

class Worker : public QObject {
Q_OBJECT

public:
    Worker(LUT3D *lut, const CameraParameters &camera_params, const RoboCupField &field);

    ~Worker() override;

    virtual void update(FrameData *frame);

    virtual void CopyToLUT(LUT3D *lut);

    virtual void ResetModel();


    std::vector<ClazzProperties> cProp;
    std::vector<int> color2Clazz;
    const CameraParameters &camera_parameters;
    YUVLUT local_lut;
    LUT3D *global_lut;

    VarBool *_v_removeOutlierBlobs;


    std::mutex mutex_locs;
    std::vector<LocLabeled> locs;

    bool globalLutUpdate = false;
    bool running = true;
    bool liveUpdate = false;
public slots:

    void process();

signals:

    void finished();

    void error(QString err);

private:
    int getColorFromModelOutput(
            doubleVec &output);

    void getRegionDesiredPixelDim(
            const CMVision::Region *region,
            int clazz,
            int &width,
            int &height);

    bool isInAngleRange(
            const vector3d &pField,
            int clazz,
            const BotPosStamped *botPos);

    BotPosStamped *findNearestBotPos(
            const vector3d &loc,
            double *dist,
            const BotPosStamped *exceptThisBot);

    void updateModel(
            const RawImage *image,
            const pixelloc &loc,
            uint8_t clazz);

    void updateBotPositions(
            const SSL_DetectionFrame *detection_frame);

    void addRegionKMeans(
            const RawImage *img,
            int targetClazz,
            const CMVision::Region *region,
            int width,
            int height,
            int offset,
            std::vector<LocLabeled> &locations);

    void processRegions(
            const RawImage *img,
            const std::vector<CMVision::Region> &regions,
            std::vector<LocLabeled> &locations);

    // synchronization
    std::mutex mutex_sync;
    std::mutex mutex_input;
    std::mutex mutex_model;
    std::condition_variable d_condition;

    // input data
    WorkerInput *input;
    WorkerInput inputData[2];
    int inputIdx;

    // state
    const RoboCupField &field;

    std::vector<LWPR_Object *> models;
    float robot_tracking_time;
    int max_regions = 1000;
    std::vector<BotPosStamped *> botPoss;

    BlobDetector blobDetector;
};

class PluginOnlineColorCalib : public VisionPlugin {
Q_OBJECT

public:
    PluginOnlineColorCalib(FrameBuffer *_buffer,
                           LUT3D *lut,
                           const CameraParameters &camera_params,
                           const RoboCupField &field);

    ~PluginOnlineColorCalib() override;

    ProcessResult process(FrameData *data, RenderOptions *options) override;

    QWidget *getControlWidget() override;

    VarList *getSettings() override;

    string getName() override;

    void mousePressEvent(QMouseEvent *event, pixelloc loc) override;

    void mouseReleaseEvent(QMouseEvent *event, pixelloc loc) override;

    void mouseMoveEvent(QMouseEvent *event, pixelloc loc) override;

private:

    InitialColorCalibrator initialCalibrator;

    LUT3D *global_lut;
    const CameraParameters &camera_parameters;
    bool initial_calib_running = false;
    int nFrames = 0;

    AutomatedColorCalibWidget *_accw = nullptr;
    VarList *_settings;
    VarBool *_v_debug;
    Worker *worker;

    VarDouble *drag_x = nullptr;
    VarDouble *drag_y = nullptr;

    bool doing_drag = false;
    bool enabled = false;

    bool setDragParamsIfHit(pixelloc loc, VarDouble *x, VarDouble *y);

    void process_gui_commands();
};

#endif /* SRC_APP_PLUGINS_PLUGIN_ONLINE_COLOR_CALIB_H_ */
