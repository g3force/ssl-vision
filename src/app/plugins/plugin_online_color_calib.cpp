/*
 * plugin_online_color_calib.cpp
 *
 *  Created on: Jul 19, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 *      Mark Geiger <markgeiger@posteo.de>
 */

#include "plugin_online_color_calib.h"
#include <opencv2/opencv.hpp>
#include <gui/automatedcolorcalibwidget.h>

#define CH_ORANGE 2
#define CH_YELLOW 3
#define CH_BLUE 4
#define CH_PINK 5
#define CH_GREEN 7

PluginOnlineColorCalib::PluginOnlineColorCalib(
        FrameBuffer *_buffer,
        LUT3D *lut,
        const CameraParameters &camera_params,
        const RoboCupField &field)
        :
        VisionPlugin(_buffer),
        camera_parameters(camera_params) {

    auto *thread = new QThread();
    thread->setObjectName("OnlineColorCalib");

    _accw = nullptr;
    worker = new Worker(lut, camera_params, field);
    worker->moveToThread(thread);

    global_lut = lut;
    initial_calib_running = false;
    nFrames = 0;

    _settings = new VarList("Online Color Calib");

    _settings->addChild(
            _updGlob = new VarTrigger("UpdateGlob", "Update Global LUT"));
    connect(_updGlob, SIGNAL(signalTriggered()), this,
            SLOT(slotUpdateTriggered()));

    _settings->addChild(
            _update = new VarTrigger("Init", "Classify initial LUT"));
    connect(_update, SIGNAL(signalTriggered()), this,
            SLOT(slotUpdateTriggeredInitial()));

    _settings->addChild(
            _resetModel = new VarTrigger("Reset", "Reset Model"));
    connect(_resetModel, SIGNAL(signalTriggered()), this,
            SLOT(slotResetModelTriggered()));

    _settings->addChild(_v_enable = new VarBool("enabled", false));
    _settings->addChild(_v_debug = new VarBool("debug", false));
    _settings->addChild(worker->_v_lifeUpdate);
    _settings->addChild(worker->_v_removeOutlierBlobs);

    connect(thread, SIGNAL(started()), worker, SLOT(process()));
    connect(worker, SIGNAL(finished()), thread, SLOT(quit()));
    connect(worker, SIGNAL(finished()), worker, SLOT(deleteLater()));
    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
    thread->start();

    this->installEventFilter(this);
}

PluginOnlineColorCalib::~PluginOnlineColorCalib() {
    worker->running = false;
}

QWidget *PluginOnlineColorCalib::getControlWidget() {
    if (_accw == 0)
        _accw = new AutomatedColorCalibWidget();

    return (QWidget *) _accw;
}


void PluginOnlineColorCalib::slotUpdateTriggered() {
    worker->globalLutUpdate = true;
}

void PluginOnlineColorCalib::slotResetModelTriggered() {
    worker->ResetModel();
}

Worker::Worker(
        LUT3D *lut,
        const CameraParameters &camera_params,
        const RoboCupField &field)
        :
        cProp(5),
        color2Clazz(10, 0),
        camera_parameters(camera_params),
        local_lut(6, 6, 6),
        field(field) {
    global_lut = lut;

    _v_lifeUpdate = new VarBool("life update", false);
    _v_removeOutlierBlobs = new VarBool("remove outlier blobs", false);


    cProp[0].color = CH_ORANGE;
    color2Clazz[CH_ORANGE] = 0;
    cProp[0].height = 42;
    cProp[0].minDist = 200;
    cProp[0].maxDist = DBL_MAX;
    cProp[0].radius = 22; // also accept small detected blobs of orange // 22
    cProp[0].nAngleRanges = 0;

    cProp[1].color = CH_YELLOW;
    color2Clazz[CH_YELLOW] = 1;
    cProp[2].color = CH_BLUE;
    color2Clazz[CH_BLUE] = 2;
    for (int i = 1; i < 3; i++) {
        cProp[i].height = 130;
        cProp[i].minDist = 0;
        cProp[i].maxDist = 20;
        cProp[i].radius = 25;
        cProp[i].nAngleRanges = 0;
    }

    cProp[3].color = CH_PINK;
    color2Clazz[CH_PINK] = 3;
    cProp[4].color = CH_GREEN;
    color2Clazz[CH_GREEN] = 4;
    for (int i = 3; i < 5; i++) {
        cProp[i].height = 130;
        cProp[i].minDist = 40;
        cProp[i].maxDist = 90;
        cProp[i].radius = 20;
        cProp[i].nAngleRanges = 2;
        cProp[i].angleRanges[0].min = 40.0 / 180.0 * M_PI;
        cProp[i].angleRanges[0].max = 75.0 / 180.0 * M_PI;
        cProp[i].angleRanges[1].min = 125.0 / 180.0 * M_PI;
        cProp[i].angleRanges[1].max = 165.0 / 180.0 * M_PI;
    }

    ResetModel();

    robot_tracking_time = 10;
    max_regions = 200;
    input = inputData;
    inputIdx = 0;
    inputData[0].number = -1;
    inputData[1].number = -1;
    globalLutUpdate = false;
    running = true;
}

Worker::~Worker() = default;

void Worker::ResetModel() {
    mutex_model.lock();
    local_lut.reset();

    for (auto &model : models)
        delete model;
    models.clear();

    for (size_t i = 0; i < cProp.size(); i++) {
        auto *model = new LWPR_Object(3, 1);
        doubleVec norm(3, 255);
        model->normIn(norm);
        model->setInitD(100);
        model->wGen(0.1);
        model->initLambda(0.995);
        model->finalLambda(0.999);
        models.push_back(model);
    }
    std::cout << "Models resetted" << std::endl;
    mutex_model.unlock();
}

void Worker::CopytoLUT(
        LUT3D *lut) {
    doubleVec input(3);
    doubleVec output(cProp.size());

    mutex_model.lock();
    for (int y = 0; y <= 255; y += (0x1 << global_lut->X_SHIFT)) {
        for (int u = 0; u <= 255; u += (0x1 << global_lut->Y_SHIFT)) {
            for (int v = 0; v <= 255; v += (0x1 << global_lut->Z_SHIFT)) {
                input[0] = (double) y;
                input[1] = (double) u;
                input[2] = (double) v;
                for (size_t i = 0; i < cProp.size(); i++) {
                    doubleVec out = models[i]->predict(input, 0.01);
                    output[i] = out[0];
                }
                int color = getColorFromModelOutput(output);
                local_lut.set(static_cast<unsigned char>(y), static_cast<unsigned char>(u),
                              static_cast<unsigned char>(v), static_cast<lut_mask_t>(color));
            }
        }
    }
    mutex_model.unlock();

    lut->lock();
    for (int y = 0; y <= 255; y += (0x1 << global_lut->X_SHIFT)) {
        for (int u = 0; u <= 255; u += (0x1 << global_lut->Y_SHIFT)) {
            for (int v = 0; v <= 255; v += (0x1 << global_lut->Z_SHIFT)) {
                int color = local_lut.get(static_cast<const unsigned char>(y),
                                          static_cast<const unsigned char>(u),
                                          static_cast<const unsigned char>(v));
                lut->set(static_cast<unsigned char>(y),
                         static_cast<unsigned char>(u),
                         static_cast<unsigned char>(v),
                         static_cast<lut_mask_t>(color));
            }
        }
    }
    lut->unlock();

}

static yuv getColorFromImage(
        const RawImage *img,
        const int x,
        const int y) {
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

int Worker::getColorFromModelOutput(
        doubleVec &output) {
    size_t maxIdx = 0;
    double maxValue = 0;
    for (size_t i = 0; i < cProp.size(); i++) {
        double val = output[i];

        /*
        if (cProp[i].color == CH_ORANGE) {
            val = std::abs(val * 2.0);
        }*/

        if (val > maxValue) {
            // maybe give confidence boni to orange here ?
            maxValue = val;
            maxIdx = i;
        }
    }
    if (maxValue > 0.5)
        return cProp[maxIdx].color;
    return 0;
}

void Worker::getRegionDesiredPixelDim(
        const CMVision::Region *region,
        const int clazz,
        int &width,
        int &height) {
    vector2d pImg;
    pImg.x = region->cen_x;
    pImg.y = region->cen_y;
    vector3d pField;
    camera_parameters.image2field(pField, pImg, cProp[clazz].height);

    vector3d pField_left = pField;
    vector3d pField_right = pField;
    vector3d pField_top = pField;
    vector3d pField_bottom = pField;
    pField_left.x -= cProp[clazz].radius;
    pField_right.x += cProp[clazz].radius;
    pField_top.y += cProp[clazz].radius;
    pField_bottom.y -= cProp[clazz].radius;

    vector2d pImg_left;
    vector2d pImg_right;
    vector2d pImg_top;
    vector2d pImg_bottom;
    camera_parameters.field2image(pField_left, pImg_left);
    camera_parameters.field2image(pField_right, pImg_right);
    camera_parameters.field2image(pField_top, pImg_top);
    camera_parameters.field2image(pField_bottom, pImg_bottom);

    width = static_cast<int>(std::abs(pImg_left.x - pImg_right.x));
    height = static_cast<int>(std::abs(pImg_top.y - pImg_bottom.y));
}

void Worker::getRegionFieldDim(
        const CMVision::Region *region,
        const int clazz,
        double &width,
        double &height) {
    vector2d pImg;
    pImg.x = region->cen_x;
    pImg.y = region->cen_y;

    vector2d pImg_left = pImg;
    vector2d pImg_right = pImg;
    vector2d pImg_bottom = pImg;
    vector2d pImg_top = pImg;
    pImg_left.x += region->width() / 2;
    pImg_right.x -= region->width() / 2;
    pImg_bottom.y -= region->height() / 2;
    pImg_top.y += region->height() / 2;

    vector3d pField_left;
    vector3d pField_right;
    vector3d pField_bottom;
    vector3d pField_top;
    camera_parameters.image2field(pField_left, pImg_left, cProp[clazz].height);
    camera_parameters.image2field(pField_right, pImg_right,
                                  cProp[clazz].height);
    camera_parameters.image2field(pField_bottom, pImg_bottom,
                                  cProp[clazz].height);
    camera_parameters.image2field(pField_top, pImg_top, cProp[clazz].height);

    width = std::abs(pField_right.x - pField_left.x);
    height = std::abs(pField_top.y - pField_bottom.y);
}

static double normalizeAngle(double angle) {
    return (angle - (round((angle / (M_PI * 2)) - 1e-6) * M_PI * 2));
}

static double angleDiff(double angle1, double angle2) {
    return normalizeAngle(normalizeAngle(angle1) - normalizeAngle(angle2));
}

static double norm(vector2d vec) {
    return sqrt(vec.x * vec.x + vec.y * vec.y);
}

static double getAngle(vector2d vec) {
    if (vec.x == 0 && vec.y == 0)
        return 0;
    double tmp = acos(vec.x / norm(vec));
    if (vec.y > 0) {
        return tmp;
    }
    return -tmp;
}

bool Worker::isInAngleRange(
        const vector3d &pField,
        const int clazz,
        const BotPosStamped *botPos) {
    bool inAngleRange = true;
    if (cProp[clazz].nAngleRanges > 0.01) {
        inAngleRange = false;
        vector2d regionDir;
        regionDir.x = pField.x - botPos->pos.x;
        regionDir.y = pField.y - botPos->pos.y;
        double regionAngle = getAngle(regionDir);
        double diff = std::abs(angleDiff(botPos->orientation, regionAngle));

        for (int i = 0; i < cProp[clazz].nAngleRanges; i++) {
            if (diff > cProp[clazz].angleRanges[i].min
                && diff < cProp[clazz].angleRanges[i].max) {
                inAngleRange = true;
                break;
            }
        }
    }
    return inAngleRange;
}

static double distanceSq(
        const vector3d &p1,
        const vector3d &p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return dx * dx + dy * dy;
}

BotPosStamped *Worker::findNearestBotPos(
        const vector3d &loc,
        double *dist,
        const BotPosStamped *exceptThisBot) {
    double minDistSq = 1e20;
    BotPosStamped *nearestBot = 0;
    for (auto &botPos : botPoss) {
        double distSq = distanceSq(botPos->pos, loc);
        if (distSq < minDistSq && exceptThisBot != botPos) {
            minDistSq = distSq;
            nearestBot = botPos;
        }
    }
    *dist = sqrt(minDistSq);
    return nearestBot;
}

void Worker::updateModel(
        const RawImage *image,
        const pixelloc &loc,
        const uint8_t clazz) {
    if (loc.x < 0 || loc.x >= image->getWidth()
        || loc.y < 0 || loc.y >= image->getHeight()) {
        return;
    }

    yuv color = getColorFromImage(image, loc.x, loc.y);

    doubleVec v_in(3);
    v_in[0] = color.y;
    v_in[1] = color.u;
    v_in[2] = color.v;

    mutex_model.lock();
    doubleVec output(models.size());

    doubleVec v_out(1);
    for (size_t i = 0; i < models.size(); i++) {
        v_out[0] = (clazz == i);
        doubleVec out = models[i]->update(v_in, v_out);
        output[i] = out[0];
    }
    mutex_model.unlock();
}

void Worker::updateBotPositions(
        const SSL_DetectionFrame *detection_frame) {
    std::vector<SSL_DetectionRobot> robots;
    robots.insert(robots.end(), detection_frame->robots_blue().begin(),
                  detection_frame->robots_blue().end());
    robots.insert(robots.end(), detection_frame->robots_yellow().begin(),
                  detection_frame->robots_yellow().end());

    for (auto &robot : robots) {
        vector3d loc;
        loc.x = robot.x();
        loc.y = robot.y();
        loc.z = robot.height();
        double dist = 0;
        BotPosStamped *nearestBot = findNearestBotPos(loc, &dist);
        if (nearestBot != 0 && dist < 180) {
            nearestBot->time = detection_frame->t_capture();
            nearestBot->pos = loc;
            nearestBot->orientation = robot.orientation();
        } else {
            auto *botPos = new BotPosStamped;
            botPos->time = detection_frame->t_capture();
            botPos->pos = loc;
            botPos->orientation = robot.orientation();
            botPoss.push_back(botPos);
        }
    }

    // remove old ones
    for (auto it = botPoss.begin();
         it != botPoss.end();) {
        BotPosStamped *botPos = *it;
        if ((detection_frame->t_capture() - botPos->time) > robot_tracking_time) {
            delete botPos;
            it = botPoss.erase(it);
        } else {
            it++;
        }
    }
}

void Worker::addRegionCross(const int targetClazz, const CMVision::Region *region, const int width, const int height,
                            const int exclWidth, const int exclHeight, const int offset,
                            std::vector<LocLabeled> &locs) {
    for (double i = -width / 2 - offset; i <= width / 2 + offset; i++) {
        if (abs(i) < exclWidth)
            continue;
        pixelloc loc{};
        loc.x = static_cast<int>(region->cen_x + i);
        loc.y = static_cast<int>(region->cen_y);
        LocLabeled ll{};
        ll.loc = loc;
        ll.clazz = targetClazz;
        locs.push_back(ll);
    }
    for (int i = -height / 2 - offset; i <= height / 2 + offset; i++) {
        if (abs(i) < exclHeight)
            continue;
        pixelloc loc{};
        loc.x = static_cast<int>(region->cen_x);
        loc.y = static_cast<int>(region->cen_y + i);
        LocLabeled ll{};
        ll.loc = loc;
        ll.clazz = targetClazz;
        locs.push_back(ll);
    }
}

void Worker::addRegionKMeans(
        const RawImage *img,
        const int targetClazz,
        const CMVision::Region *region,
        const int width,
        const int height,
        const int offset,
        std::vector<LocLabeled> &locs) {
    Blob blob;
    blob.center.x = static_cast<int>(region->cen_x);
    blob.center.y = static_cast<int>(region->cen_y);
    blob.height = height + offset;
    blob.width = width + offset;
    bool ok = blobDetector.detectBlob(img, blob, nullptr);
    if (ok) {
        int minX = std::min(region->x1, region->x2);
        int maxX = std::max(region->x1, region->x2);
        int minY = std::min(region->y1, region->y2);
        int maxY = std::max(region->y1, region->y2);

        for (int x = minX; x < maxX; x++) {
            for (int y = minY; y < maxY; y++) {
                bool setPixel = false;
                for (size_t i = 0; i < blob.detectedPixels.size(); i++) {
                    // update model with pixel detected inside a blob
                    pixelloc loc{};
                    loc.x = blob.center.x + blob.detectedPixels[i].x;
                    loc.y = blob.center.y + blob.detectedPixels[i].y;
                    if (loc.x == x && loc.y == y) {
                        LocLabeled ll{};
                        ll.loc = loc;
                        ll.clazz = targetClazz;
                        locs.push_back(ll);
                        setPixel = true;
                        break;
                    }
                }
                if (!setPixel) {
                    // set pixel inside of region but not in blob -1
                    LocLabeled ll{};
                    ll.loc = {x, y};
                    ll.clazz = -1;
                    locs.push_back(ll);
                }
            }
        }
    }
}

void Worker::processRegions(
        const RawImage *img,
        const std::vector<CMVision::Region> &regions,
        std::vector<LocLabeled> &locs) {
    for (const auto &i : regions) {
        const CMVision::Region *region = &i;
        int clazz = color2Clazz[region->color.v];
        vector2d pImg;
        pImg.x = region->cen_x;
        pImg.y = region->cen_y;
        vector3d pField;
        camera_parameters.image2field(pField, pImg,
                                      cProp[clazz].height);

        double dist;
        BotPosStamped *botPos = findNearestBotPos(pField,
                                                  &dist);

        int pWidth, pHeight;
        getRegionDesiredPixelDim(region, clazz, pWidth,
                                 pHeight);

        if (dist < cProp[clazz].maxDist) {

            if (dist < cProp[clazz].minDist
                || !isInAngleRange(pField, clazz, botPos)) {

                if (_v_removeOutlierBlobs->getBool()) {
                   addRegionCross(-1, region, region->width(), region->height(), -1, -1, 0, locs);
                }

            } else {
                addRegionKMeans(img, clazz, region, pWidth, pHeight, 2, locs);
            }
        } else if (_v_removeOutlierBlobs->getBool()) {
            addRegionCross(-1, region, region->width(), region->height(), -1, -1, 0, locs);
        }
    }
}

void Worker::update(
        FrameData *frame) {
    SSL_DetectionFrame *detection_frame =
            (SSL_DetectionFrame *) frame->map.get("ssl_detection_frame");
    if (detection_frame == nullptr) {
        printf("no detection frame\n");
        return;
    }

    CMVision::ColorRegionList *colorlist;
    colorlist = (CMVision::ColorRegionList *) frame->map.get(
            "cmv_colorlist");
    if (colorlist == nullptr) {
        printf(
                "error in robot detection plugin: no region-lists were found!\n");
        return;
    }

    mutex_input.lock();
    this->input->number++;
    this->input->detection_frame = *detection_frame;
    this->input->image.deepCopyFromRawImage(frame->video, true);
    this->input->regions.clear();
    int nReg = 0;
    for (auto &clazz : cProp) {
        CMVision::Region *region = colorlist->getRegionList(
                clazz.color).getInitialElement();
        while (region != nullptr && nReg < max_regions) {
            this->input->regions.push_back(*region);
            region = region->next;
            nReg++;
        }
    }
    if (nReg == max_regions) {
        std::cout << "Too many regions." << std::endl;
    }
    mutex_input.unlock();

    d_condition.notify_one();
}

void Worker::process() {
    long long int lastNumber = -1;
    while (running) {
        WorkerInput *input;
        {
            std::unique_lock<std::mutex> lock(this->mutex_sync);
            this->d_condition.wait(lock,
                                   [=] { return this->input->number != -1 && (this->input->number > lastNumber); });
            mutex_input.lock();
            lastNumber = this->input->number;
            input = this->input;
            inputIdx = (inputIdx + 1) % 2;
            this->input = inputData + inputIdx;
            mutex_input.unlock();
        }

        updateBotPositions(&input->detection_frame);

        std::vector<LocLabeled> locs;
        auto t1 = std::chrono::system_clock::now();
        processRegions(&input->image, input->regions, locs);
        auto t2 = std::chrono::system_clock::now();

        for (auto &loc : locs) {
            updateModel(&input->image, loc.loc, loc.clazz);
        }

        std::chrono::duration<double> tProc = (t2 - t1);
        std::cout << "Proc time: " <<
                  tProc.count() <<
                  " new locs: " << locs.size() <<
                  std::endl;

        mutex_locs.lock();
        this->locs = locs;
        mutex_locs.unlock();

        if (_v_lifeUpdate->getBool()) {
            globalLutUpdate = true;
        }

        if (globalLutUpdate) {
            auto tt1 = std::chrono::system_clock::now();
            CopytoLUT(global_lut);
            std::chrono::duration<double> diff = std::chrono::system_clock::now() - tt1;
            std::cout << "LUT updated in " << diff.count() << "s" << std::endl;
            globalLutUpdate = false;
        }

    }
    emit finished();
}

ProcessResult PluginOnlineColorCalib::process(FrameData *frame,
                                              RenderOptions *options) {
    (void) options;
    if (frame == nullptr)
        return ProcessingFailed;

    // handle GUI commands here
    process_gui_commands();

    // run initial calibration
    if (initial_calib_running) {
        ProcessResult result = initialCalibrator.handleInitialCalibration(frame, options, camera_parameters,
                                                                          global_lut);
        if (result == ProcessingOk) {
            nFrames++;
            if (nFrames > 5) {
                nFrames = 0;
                initial_calib_running = false;
            }
        }
    }

    // run online calibration
    ColorFormat source_format = frame->video.getColorFormat();
    if (_v_enable->getBool()) {

        if (source_format != COLOR_YUV422_UYVY) {
            std::cerr << "Unsupported source format: " << source_format
                      << std::endl;
            _v_enable->setBool(false);
            return ProcessingFailed;
        }

        worker->update(frame);

        if (_v_debug->getBool()) {
            Image<raw8> *img_debug;
            if ((img_debug = (Image<raw8> *) frame->map.get(
                    "cmv_online_color_calib")) == nullptr) {
                img_debug = (Image<raw8> *) frame->map.insert(
                        "cmv_online_color_calib", new Image<raw8>());
            }
            img_debug->allocate(frame->video.getWidth(), frame->video.getHeight());
            img_debug->fillColor(0);

            worker->mutex_locs.lock();
            for (auto ll : worker->locs) {
                if (ll.clazz >= 0)
                    img_debug->setPixel(ll.loc.x, ll.loc.y, static_cast<raw8>(worker->cProp[ll.clazz].color));
                else
                    img_debug->setPixel(ll.loc.x, ll.loc.y, 1);
            }
            worker->mutex_locs.unlock();
        }
    }

    if (source_format == COLOR_YUV422_UYVY) {
        Image<raw8> *img_thresholded;
        if ((img_thresholded = (Image<raw8> *) frame->map.get(
                "cmv_learned_threshold")) == nullptr) {
            img_thresholded = (Image<raw8> *) frame->map.insert(
                    "cmv_learned_threshold", new Image<raw8>());
        }
        img_thresholded->allocate(frame->video.getWidth(),
                                  frame->video.getHeight());
        CMVisionThreshold::thresholdImageYUV422_UYVY(img_thresholded,
                                                     &(frame->video), &worker->local_lut);
    }

    return ProcessingOk;
}

VarList *PluginOnlineColorCalib::getSettings() {
    return _settings;
}

string PluginOnlineColorCalib::getName() {
    return "OnlineColorCalib";
}

void PluginOnlineColorCalib::slotUpdateTriggeredInitial() {
    // global_lut->reset();
    nFrames = 0;
    initial_calib_running = true;
}

void PluginOnlineColorCalib::process_gui_commands() {
    if (_accw == nullptr) {
        return;
    }
    if (_accw->is_click_initial()) {
        slotUpdateTriggeredInitial();
        _accw->set_status("Triggered initial calibration");
    }
    if (_accw->is_click_start_learning()) {
        _v_enable->setBool(true);
        _accw->set_status("Triggered start learning");
    }
    if (_accw->is_click_finish_learning()) {
        _v_enable->setBool(false);
        _accw->set_status("Triggered finish learning");
    }
    if (_accw->is_click_update_model()) {
        worker->globalLutUpdate = true;
        worker->CopytoLUT(global_lut);
        _accw->set_status("Model updated");
    }
    if (_accw->is_click_reset()) {
        slotResetModelTriggered();
        _accw->set_status("Model resetted");
    }

    if (_accw->is_automatic_mode_active()) {
        _accw->set_status("Automatic mode active!");
        worker->_v_lifeUpdate->setBool(true);
        _v_enable->setBool(true);
    } else {
        if (worker->_v_lifeUpdate->getBool()) {
            _accw->set_status("Automatic mode deactivated");
            _v_enable->setBool(false);
        }
        worker->_v_lifeUpdate->setBool(false);
    }
}

void PluginOnlineColorCalib::mousePressEvent(QMouseEvent *event, pixelloc loc) {

    std::vector<VarDouble *> ax;
    std::vector<VarDouble *> ay;

    ax.push_back(
            camera_parameters.additional_calibration_information->init_yellow_x);
    ay.push_back(
            camera_parameters.additional_calibration_information->init_yellow_y);
    ax.push_back(
            camera_parameters.additional_calibration_information->init_blue_x);
    ay.push_back(
            camera_parameters.additional_calibration_information->init_blue_y);
    ax.push_back(
            camera_parameters.additional_calibration_information->init_green_x);
    ay.push_back(
            camera_parameters.additional_calibration_information->init_green_y);
    ax.push_back(
            camera_parameters.additional_calibration_information->init_pink_x);
    ay.push_back(
            camera_parameters.additional_calibration_information->init_pink_y);
    ax.push_back(
            camera_parameters.additional_calibration_information->init_orange_x);
    ay.push_back(
            camera_parameters.additional_calibration_information->init_orange_y);

    if ((event->buttons() & Qt::LeftButton) != 0) {
        drag_x = nullptr;
        drag_y = nullptr;

        for (size_t i = 0; i < ax.size(); i++) {
            if (setDragParamsIfHit(loc,
                                   ax[i],
                                   ay[i])) {
                break;
            }
        }
        if (drag_x != nullptr && drag_y != nullptr) {
            event->accept();
            doing_drag = true;
        } else {
            event->ignore();
        }
    } else
        event->ignore();
}

bool PluginOnlineColorCalib::setDragParamsIfHit(pixelloc loc, VarDouble *x, VarDouble *y) {
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

void PluginOnlineColorCalib::mouseReleaseEvent(QMouseEvent *event, pixelloc loc) {
    (void) loc;
    doing_drag = false;
    event->accept();
}

void PluginOnlineColorCalib::mouseMoveEvent(QMouseEvent *event, pixelloc loc) {
    if (doing_drag && (event->buttons() & Qt::LeftButton) != 0) {
        if (loc.x < 0) loc.x = 0;
        if (loc.y < 0) loc.y = 0;
        drag_x->setDouble(loc.x);
        drag_y->setDouble(loc.y);
        event->accept();
    } else
        event->ignore();
}