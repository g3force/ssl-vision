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
  \file    online_color_calibrator.cpp
  \brief   C++ Implementation: OnlineColorCalibrator
  \author  Nicolai Ommer <nicolai.ommer@gmail.com>, (C) 2016
*/
//========================================================================

#include "online_color_calibrator.h"

OnlineColorCalibrator::OnlineColorCalibrator(
        YUVLUT *lut,
        const CameraParameters &camera_params,
        const RoboCupField &field)
        :
        cProp(5),
        color2Clazz(10, 0),
        camera_parameters(camera_params),
        local_lut(lut->getSizeX(), lut->getSizeY(), lut->getSizeZ()),
        global_lut(lut),
        field(field) {

  local_lut.addDerivedLUT(new RGBLUT(5, 5, 5, ""));


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
  input = inputData;
  inputIdx = 0;
  inputData[0].number = -1;
  inputData[1].number = -1;

  _settings = new VarList("Auto Color Calibration");
  _settings->addChild(_v_removeOutlierBlobs = new VarBool("remove outlier blobs", true));

  auto *thread = new QThread();
  thread->setObjectName("OnlineColorCalibrator");
  moveToThread(thread);
  connect(thread, SIGNAL(started()), this, SLOT(process()));
  connect(this, SIGNAL(finished()), thread, SLOT(quit()));
  connect(this, SIGNAL(finished()), this, SLOT(deleteLater()));
  connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
  thread->start();
}

OnlineColorCalibrator::~OnlineColorCalibrator() {
  running = false;
}

void OnlineColorCalibrator::ResetModel() {
  mutex_model.lock();
  local_lut.reset();
  local_lut.updateDerivedLUTs();

  for (auto &model : models) {
    delete model;
  }
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
  mutex_model.unlock();
}

void OnlineColorCalibrator::CopyToLUT(YUVLUT *lut) {
  lut->lock();
  for (int y = 0; y <= 255; y += (0x1u << lut->X_SHIFT)) {
    for (int u = 0; u <= 255; u += (0x1u << lut->Y_SHIFT)) {
      for (int v = 0; v <= 255; v += (0x1u << lut->Z_SHIFT)) {
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
  lut->updateDerivedLUTs();
}

void OnlineColorCalibrator::updateLocalLUT() {
  doubleVec in(3);
  doubleVec output(cProp.size());

  // update local LUT by quering the model. This can take a bit longer, so rather not lock the non-local LUT for too long
  mutex_model.lock();
  for (int y = 0; y <= 255; y += (0x1u << local_lut.X_SHIFT)) {
    for (int u = 0; u <= 255; u += (0x1u << local_lut.Y_SHIFT)) {
      for (int v = 0; v <= 255; v += (0x1u << local_lut.Z_SHIFT)) {
        in[0] = (double) y;
        in[1] = (double) u;
        in[2] = (double) v;
        for (size_t i = 0; i < cProp.size(); i++) {
          doubleVec out = models[i]->predict(in, 0.01);
          output[i] = out[0];
        }
        int color = getColorFromModelOutput(output);
        local_lut.set(static_cast<unsigned char>(y), static_cast<unsigned char>(u),
                      static_cast<unsigned char>(v), static_cast<lut_mask_t>(color));
      }
    }
  }
  mutex_model.unlock();
  local_lut.updateDerivedLUTs();
}

int OnlineColorCalibrator::getColorFromModelOutput(
        doubleVec &output) {
  size_t maxIdx = 0;
  double maxValue = 0;
  for (size_t i = 0; i < cProp.size(); i++) {
    double val = output[i];

    if (val > maxValue) {
      // maybe give confidence boni to orange here ?
      maxValue = val;
      maxIdx = i;
    }
  }
  if (maxValue > 0.5) {
    return cProp[maxIdx].color;
  }
  return 0;
}

void OnlineColorCalibrator::getRegionDesiredPixelDim(
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
  if (vec.x == 0 && vec.y == 0) {
    return 0;
  }
  double tmp = acos(vec.x / norm(vec));
  if (vec.y > 0) {
    return tmp;
  }
  return -tmp;
}

bool OnlineColorCalibrator::isInAngleRange(
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

BotPosStamped *OnlineColorCalibrator::findNearestBotPos(
        const vector3d &loc,
        double *dist,
        const BotPosStamped *exceptThisBot = nullptr) {
  double minDistSq = 1e20;
  BotPosStamped *nearestBot = nullptr;
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

void OnlineColorCalibrator::updateModel(
        const RawImage *image,
        const pixelloc &loc,
        const uint8_t clazz) {
  if (loc.x < 0 || loc.x >= image->getWidth()
      || loc.y < 0 || loc.y >= image->getHeight()) {
    return;
  }

  yuv color = image->getYuv(loc.x, loc.y);

  doubleVec v_in(3);
  v_in[0] = color.y;
  v_in[1] = color.u;
  v_in[2] = color.v;

  mutex_model.lock();

  doubleVec v_out(1);
  for (size_t i = 0; i < models.size(); i++) {
    v_out[0] = (clazz == i);
    models[i]->update(v_in, v_out);
  }
  mutex_model.unlock();
}

void OnlineColorCalibrator::updateBotPositions(const SSL_DetectionFrame *detection_frame) {
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
    if (nearestBot != nullptr && dist < 180) {
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
  for (auto it = botPoss.begin(); it != botPoss.end();) {
    BotPosStamped *botPos = *it;
    if ((detection_frame->t_capture() - botPos->time) > robot_tracking_time) {
      delete botPos;
      it = botPoss.erase(it);
    } else {
      it++;
    }
  }
}

static void addRegionCross(const int targetClazz,
                           const CMVision::Region *region,
                           const int width,
                           const int height,
                           const int exclWidth,
                           const int exclHeight,
                           const int offset,
                           std::vector<LocLabeled> &locations) {
  for (int i = -width / 2 - offset; i <= width / 2 + offset; i++) {
    if (abs(i) < exclWidth) {
      continue;
    }
    pixelloc loc{};
    loc.x = static_cast<int>(region->cen_x + i);
    loc.y = static_cast<int>(region->cen_y);
    LocLabeled ll{};
    ll.loc = loc;
    ll.clazz = targetClazz;
    locations.push_back(ll);
  }
  for (int i = -height / 2 - offset; i <= height / 2 + offset; i++) {
    if (abs(i) < exclHeight) {
      continue;
    }
    pixelloc loc{};
    loc.x = static_cast<int>(region->cen_x);
    loc.y = static_cast<int>(region->cen_y + i);
    LocLabeled ll{};
    ll.loc = loc;
    ll.clazz = targetClazz;
    locations.push_back(ll);
  }
}

void OnlineColorCalibrator::addRegionKMeans(
        const RawImage *img,
        const int targetClazz,
        const CMVision::Region *region,
        const int width,
        const int height,
        const int offset,
        std::vector<LocLabeled> &locations) {
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
            locations.push_back(ll);
            setPixel = true;
            break;
          }
        }
        if (!setPixel) {
          // set pixel inside of region but not in blob -1
          LocLabeled ll{};
          ll.loc = {x, y};
          ll.clazz = -1;
          locations.push_back(ll);
        }
      }
    }
  }
}

void OnlineColorCalibrator::processRegions(
        const RawImage *img,
        const std::vector<CMVision::Region> &regions,
        std::vector<LocLabeled> &locations) {
  for (const auto &i : regions) {
    const CMVision::Region *region = &i;
    int clazz = color2Clazz[region->color.v];
    vector2d pImg;
    pImg.x = region->cen_x;
    pImg.y = region->cen_y;
    vector3d pField;
    camera_parameters.image2field(pField, pImg, cProp[clazz].height);

    double dist;
    BotPosStamped *botPos = findNearestBotPos(pField, &dist);

    int pWidth, pHeight;
    getRegionDesiredPixelDim(region, clazz, pWidth, pHeight);

    if (dist < cProp[clazz].maxDist) {
      if (dist < cProp[clazz].minDist || !isInAngleRange(pField, clazz, botPos)) {
        if (_v_removeOutlierBlobs->getBool()) {
          addRegionCross(-1, region, region->width(), region->height(), -1, -1, 0, locations);
        }
      } else {
        addRegionKMeans(img, clazz, region, pWidth, pHeight, 2, locations);
      }
    } else if (_v_removeOutlierBlobs->getBool()) {
      addRegionCross(-1, region, region->width(), region->height(), -1, -1, 0, locations);
    }
  }
}

void OnlineColorCalibrator::update(FrameData *frame) {
  SSL_DetectionFrame *detection_frame = (SSL_DetectionFrame *) frame->map.get("ssl_detection_frame");
  if (detection_frame == nullptr) {
    printf("no detection frame\n");
    return;
  }

  CMVision::ColorRegionList *colorlist;
  colorlist = (CMVision::ColorRegionList *) frame->map.get("cmv_colorlist");
  if (colorlist == nullptr) {
    printf("error in robot detection plugin: no region-lists were found!\n");
    return;
  }

  mutex_input.lock();
  this->input->number++;
  this->input->detection_frame = *detection_frame;
  this->input->image.deepCopyFromRawImage(frame->video, true);
  this->input->regions.clear();
  int nReg = 0;
  for (auto &clazz : cProp) {
    CMVision::Region *region = colorlist->getRegionList(clazz.color).getInitialElement();
    while (region != nullptr && nReg < max_regions) {
      this->input->regions.push_back(*region);
      region = region->next;
      nReg++;
    }
  }
  if (nReg == max_regions) {
    std::cout << "Too many regions: " << nReg << std::endl;
  }
  mutex_input.unlock();

  d_condition.notify_one();
}

void OnlineColorCalibrator::process() {
  long long int lastNumber = -1;
  while (running) {
    WorkerInput *workerInput;
    {
      std::unique_lock<std::mutex> lock(this->mutex_sync);
      this->d_condition.wait(lock,
                             [=] { return this->input->number != -1 && (this->input->number > lastNumber); });
      mutex_input.lock();
      lastNumber = this->input->number;
      workerInput = this->input;
      inputIdx = (inputIdx + 1) % 2;
      this->input = inputData + inputIdx;
      mutex_input.unlock();
    }

    updateBotPositions(&workerInput->detection_frame);

    std::vector<LocLabeled> locations;
    auto t1 = std::chrono::system_clock::now();

    processRegions(&workerInput->image, workerInput->regions, locations);

    auto t2 = std::chrono::system_clock::now();

    for (auto &loc : locations) {
      updateModel(&workerInput->image, loc.loc, loc.clazz);
    }

    auto t3 = std::chrono::system_clock::now();

    updateLocalLUT();

    auto t4 = std::chrono::system_clock::now();

    std::chrono::duration<double> tProc = (t2 - t1);
    std::chrono::duration<double> tUpdate = (t3 - t2);
    std::chrono::duration<double> tPredict = (t4 - t3);
    std::cout <<
              "processing: " << tProc.count() << std::endl <<
              "update:     " << tUpdate.count() << std::endl <<
              "predict:    " << tPredict.count() << std::endl <<
              "new locs:   " << locations.size() << std::endl << std::endl;

    mutex_locs.lock();
    this->locs = locations;
    mutex_locs.unlock();

    if (liveUpdate) {
      globalLutUpdate = true;
    }

    if (globalLutUpdate) {
      auto tt1 = std::chrono::system_clock::now();
      CopyToLUT(global_lut);
      std::chrono::duration<double> diff = std::chrono::system_clock::now() - tt1;
      std::cout << "LUT updated in " << diff.count() << "s" << std::endl;
      globalLutUpdate = false;
    }

  }
  emit finished();
}
