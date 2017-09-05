/*
 * plugin_init_color_calib.cpp
 *
 *  Created on: Aug 12, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 *      Mark Geiger <markgeiger@posteo.de>
 */

#include "plugin_init_color_calib.h"

#include <opencv2/opencv.hpp>


#define CH_ORANGE 2
#define CH_YELLOW 3
#define CH_BLUE 4
#define CH_PINK 5
#define CH_GREEN 7

PluginInitColorCalib::PluginInitColorCalib(FrameBuffer *_buffer,
                                           LUT3D *lut, const CameraParameters &camera_params,
                                           const RoboCupField &field) :
        VisionPlugin(_buffer),
        cam_params(camera_params), field(field) {

    _settings = new VarList("Init Color Calib");

    _settings->addChild(
            _update = new VarTrigger("Init", "Classify initial LUT"));
    connect(_update, SIGNAL(signalTriggered()), this,
            SLOT(slotUpdateTriggeredInitial()));

    global_lut = lut;
    running = false;
    nFrames = 0;
    nSamples = 0;

    maxColorDist = 3000;
    this->installEventFilter(this);
}

PluginInitColorCalib::~PluginInitColorCalib() {
}

ColorClazz::ColorClazz(unsigned char r, unsigned char g, unsigned char b, int clazz)
        : color_rgb(r, g, b) {
    this->clazz = clazz;
    color_yuv = Conversions::rgb2yuv(color_rgb);
}

void PluginInitColorCalib::slotUpdateTriggeredInitial() {
    global_lut->reset();
    nFrames = 0;
    nSamples = 0;
    running = true;
}

static float rgbColorDist(rgb &c1, rgb &c2) {
    float r = c1.r - c2.r;
    float g = c1.g - c2.g;
    float b = c1.b - c2.b;
    return r * r + g * g + b * b;
}

static float ratedYuvColorDist(yuv &c1, yuv &c2, float maxColorDist, float weight) {
    float midToC1U = c1.u - 127;
    float midToC2U = c2.u - 127;
    float midToC1V = c1.v - 127;
    float midToC2V = c2.v - 127;
    float normFac1 = std::sqrt(midToC1U * midToC1U + midToC1V * midToC1V);
    float normFac2 = std::sqrt(midToC2U * midToC2U + midToC2V * midToC2V);
    midToC1U *= 1 / normFac1;
    midToC1V *= 1 / normFac1;
    midToC2U *= 1 / normFac2;
    midToC2V *= 1 / normFac2;
    float scalar = midToC1U * midToC2U + midToC1V * midToC2V;
    float angle = std::acos(scalar);

    float maxAngle = 0.3;
    float bonus = 0;
    if (angle < maxAngle) {
        // give bonus for good angles
        bonus = (1 - (angle / maxAngle)) * maxColorDist * 0.5f;
    } else {
        // give penalty for bad angles
        angle = std::min(90.0f, angle);
        bonus = -maxColorDist * 0.5f * angle / maxAngle;
    }

    float u = c1.u - c2.u;
    float v = c1.v - c2.v;
    float y = c1.y - c2.y;

    float uvDist = u * u + v * v;
    return ((uvDist + y * y) - bonus) * weight;
}

static yuv getColorFromImage(RawImage *img, int x, int y) {
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

ProcessResult PluginInitColorCalib::process(FrameData *frame,
                                            RenderOptions *options) {
    (void) options;
    if (frame == 0)
        return ProcessingFailed;

    Image<raw8> *img_debug;
    if ((img_debug = (Image<raw8> *) frame->map.get(
            "cmv_online_color_calib")) == 0) {
        img_debug = (Image<raw8> *) frame->map.insert(
                "cmv_online_color_calib", new Image<raw8>());
    }
    img_debug->allocate(frame->video.getWidth(), frame->video.getHeight());
    img_debug->fillColor(0);

    SSL_DetectionFrame *detection_frame =
            (SSL_DetectionFrame *) frame->map.get("ssl_detection_frame");
    if (detection_frame == 0) {
        printf("no detection frame\n");
        return ProcessingFailed;
    }

    if (running) {
        // delete colors and set newly

        colors.clear();

        addColorToClazz(frame,
                        cam_params.additional_calibration_information->init_yellow_x->getInt(),
                        cam_params.additional_calibration_information->init_yellow_y->getInt(),
                        CH_YELLOW);

        addColorToClazz(frame,
                        cam_params.additional_calibration_information->init_blue_x->getInt(),
                        cam_params.additional_calibration_information->init_blue_y->getInt(),
                        CH_BLUE);

        addColorToClazz(frame,
                        cam_params.additional_calibration_information->init_pink_x->getInt(),
                        cam_params.additional_calibration_information->init_pink_y->getInt(),
                        CH_PINK);

        addColorToClazz(frame,
                        cam_params.additional_calibration_information->init_orange_x->getInt(),
                        cam_params.additional_calibration_information->init_orange_y->getInt(),
                        CH_ORANGE);

        addColorToClazz(frame,
                        cam_params.additional_calibration_information->init_green_x->getInt(),
                        cam_params.additional_calibration_information->init_green_y->getInt(),
                        CH_GREEN);

        int nConflicts = 0;
        for (int y = 0; y <= 255; y += (0x1 << global_lut->X_SHIFT)) {
            for (int u = 0; u <= 255; u += (0x1 << global_lut->Y_SHIFT)) {
                for (int v = 0; v <= 255; v += (0x1 << global_lut->Z_SHIFT)) {
                    yuv color = yuv(y, u, v);

                    float minDiff = 1e10;
                    int clazz = 0;
                    for (int j = 0; j < colors.size(); j++) {
                        float diff = 0;
                        if (colors[j].clazz == CH_ORANGE) {
                            diff = ratedYuvColorDist(color, colors[j].color_yuv, maxColorDist, 2.2);
                        } else {
                            diff = ratedYuvColorDist(color, colors[j].color_yuv, maxColorDist, 1.0);
                        }
                        if (diff < minDiff) {
                            minDiff = diff;
                            clazz = colors[j].clazz;
                        }
                    }

                    if (minDiff < maxColorDist) {
                        int curClazz = global_lut->get(color.y, color.u, color.v);
                        if (curClazz != 0 && curClazz != clazz) {
                            nConflicts++;
                        }
                        global_lut->set(color.y, color.u, color.v, clazz);
                    }
                }
            }
        }
        std::cout << "Num conflicts: " << nConflicts << std::endl;
        int n = frame->video.getHeight() * frame->video.getWidth();
        nSamples += n;
        nFrames++;
        if (nFrames == 5) {
            running = false;
        }
    }

    return ProcessingOk;
}

void PluginInitColorCalib::addColorToClazz(FrameData *frame, int x, int y, int clazz) {
    yuv initColor = getColorFromImage(&frame->video, x, y);
    rgb initColorRGB = Conversions::yuv2rgb(initColor);
    colors.push_back(ColorClazz(initColorRGB.r, initColorRGB.g, initColorRGB.b, clazz));
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

VarList *PluginInitColorCalib::getSettings() {
    return _settings;
}

string PluginInitColorCalib::getName() {
    return "InitColorCalib";
}
