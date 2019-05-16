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
  \file    online_color_calibrator.h
  \brief   C++ Implementation: OnlineColorCalibrator
  \author  Nicolai Ommer <nicolai.ommer@gmail.com>, (C) 2016
*/
//========================================================================

#ifndef SSL_VISION_ONLINECOLORCALIBRATOR_H
#define SSL_VISION_ONLINECOLORCALIBRATOR_H

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
#include "color_calibrator.h"


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


class OnlineColorCalibrator : public QObject {
Q_OBJECT

public:
    OnlineColorCalibrator(LUT3D *lut, const CameraParameters &camera_params, const RoboCupField &field);

    ~OnlineColorCalibrator() override;

    void update(FrameData *frame);

    void CopyToLUT(LUT3D *lut);

    void ResetModel();


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


#endif //SSL_VISION_ONLINECOLORCALIBRATOR_H
