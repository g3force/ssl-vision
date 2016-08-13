/*
 * plugin_online_color_calib.cpp
 *
 *  Created on: Jul 19, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "plugin_online_color_calib.h"
#include <iostream>
#include "cmvision_region.h"
#include <algorithm>
#include "geometry.h"

#define CH_ORANGE 2
#define CH_YELLOW 3
#define CH_BLUE 4
#define CH_PINK 5
#define CH_GREEN 7

PluginOnlineColorCalib::PluginOnlineColorCalib(FrameBuffer * _buffer,
		LUT3D * lut, const CameraParameters& camera_params,
		const RoboCupField& field) :
		VisionPlugin(_buffer) {

	QThread* thread = new QThread();
	thread->setObjectName("OnlineColorCalib");
	worker = new Worker(lut, camera_params, field);
	worker->moveToThread(thread);

	_settings = new VarList("Online Color Calib");

	_settings->addChild(
			_updGlob = new VarTrigger("UpdateGlob", "Update Global LUT"));
	connect(_updGlob, SIGNAL(signalTriggered()), this,
			SLOT(slotUpdateTriggered()));

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
}

PluginOnlineColorCalib::~PluginOnlineColorCalib() {
}

void PluginOnlineColorCalib::slotUpdateTriggered() {
	worker->globalLutUpdate = true;
}

void PluginOnlineColorCalib::slotResetModelTriggered() {
	worker->resetModel();
}

Worker::Worker(LUT3D * lut, const CameraParameters& camera_params, const RoboCupField& field) :
		cProp(5), color2Clazz(10, 0), camera_parameters(camera_params), field(field) {
	global_lut = lut;

	_v_lifeUpdate = new VarBool("life update", false);
	_v_removeOutlierBlobs = new VarBool("remove outlier blobs", false);


	cProp[0].color = CH_ORANGE;
	color2Clazz[CH_ORANGE] = 0;
	cProp[0].height = 42;
	cProp[0].minDist = 150;
	cProp[0].maxDist = 20000;
	cProp[0].radius = 22;
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

	model = NULL;
	resetModel();

	robot_tracking_time = 2;
	this->input.number = -1;
	globalLutUpdate = false;
}

Worker::~Worker() {
}

void Worker::resetModel()
{
	mutex_model.lock();
	if(model != NULL)
	{
		delete model;
	}
	model = new LWPR_Object(3, cProp.size());
	doubleVec norm(3, 255);
	model->normIn(norm);
	model->setInitD(500);
	model->wGen(0.1);
	model->initLambda(0.9999);
	model->finalLambda(0.99999);
	mutex_model.unlock();
}

void Worker::process() {

	long long int lastNumber = -1;
	while (true) {
		{
			std::unique_lock<std::mutex> lock(this->d_mutex);
			this->d_condition.wait(lock,
					[=] {return this->input.number != -1 && ( this->input.number != lastNumber );});
			lastNumber = this->input.number;
		}
		mutex_input.lock();
		WorkerInput input = this->input;
		mutex_input.unlock();

		updateBotPositions(&input.detection_frame, botPoss);
		processRegions(&input.detection_frame, input.regions);

		mutex_locs.lock();
		updateLocs(&input.detection_frame, locs);
		mutex_locs.unlock();

		for (int i = 0; i < locs.size(); i++) {
			updateModel(input.image, locs[i]->loc, locs[i]->clazz);
		}

		if(globalLutUpdate)
		{
			CopytoLUT(global_lut);
			globalLutUpdate = false;
		}
	}
	emit finished();
}

void Worker::update(FrameData * frame) {
	SSL_DetectionFrame * detection_frame =
					(SSL_DetectionFrame *) frame->map.get("ssl_detection_frame");
	if (detection_frame == 0) {
		printf("no detection frame\n");
		return;
	}

	CMVision::ColorRegionList * colorlist;
	colorlist = (CMVision::ColorRegionList *) frame->map.get(
			"cmv_colorlist");
	if (colorlist == 0) {
		printf(
				"error in robot detection plugin: no region-lists were found!\n");
		return;
	}

	mutex_input.lock();
	this->input.number++;
	this->input.image.deepCopyFromRawImage(frame->video, true);
	this->input.detection_frame.CopyFrom(*detection_frame);
	this->input.regions.clear();
	for (int clazz = 0; clazz < cProp.size(); clazz++) {
		CMVision::Region* region = colorlist->getRegionList(
				cProp[clazz].color).getInitialElement();
		while (region != 0) {
			this->input.regions.push_back(*region);
			region = region->next;
		}
	}
	mutex_input.unlock();

	d_condition.notify_one();
}

void Worker::updateModel(RawImage& image, pixelloc& loc,
		int clazz) {
	if(loc.x < 0 || loc.x >= image.getWidth()
			|| loc.y < 0 || loc.y >= image.getHeight())
	{
		return;
	}

	yuv color;
	uyvy color2 = *((uyvy*) (image.getData()
			+ (sizeof(uyvy)
					* (((loc.y * (image.getWidth())) + loc.x) / 2))));
	color.u = color2.u;
	color.v = color2.v;
	if ((loc.x % 2) == 0) {
		color.y = color2.y1;
	} else {
		color.y = color2.y2;
	}

	doubleVec v_in(3);
	doubleVec v_out(cProp.size());
	v_in[0] = color.y;
	v_in[1] = color.u;
	v_in[2] = color.v;
	for (int i = 0; i < v_out.size(); i++)
		v_out[i] = (clazz == i);

	mutex_model.lock();
	doubleVec out = model->update(v_in, v_out);
	int col = getColorFromModelOutput(out);
	local_lut.set(color.y, color.u, color.v, col);
	if (_v_lifeUpdate->getBool()) {
		global_lut->set(color.y, color.u, color.v, col);
	}
	mutex_model.unlock();
}

int Worker::getColorFromModelOutput(doubleVec& output) {
	int maxIdx = 0;
	double maxValue = 0;
	for (int i = 0; i < cProp.size(); i++) {
		if (output[i] > maxValue) {
			maxValue = output[i];
			maxIdx = i;
		}
	}
	if (maxValue > 0.5)
		return cProp[maxIdx].color;
	return 0;
}

void Worker::CopytoLUT(LUT3D *lut) {

	doubleVec input(3);
	doubleVec output(cProp.size());

	lut->lock();
	mutex_model.lock();
	for (int y = 0; y <= 255; y += (0x1 << global_lut->X_SHIFT)) {
		for (int u = 0; u <= 255; u += (0x1 << global_lut->Y_SHIFT)) {
			for (int v = 0; v <= 255; v += (0x1 << global_lut->Z_SHIFT)) {
				input[0] = (double) y;
				input[1] = (double) u;
				input[2] = (double) v;
				output = model->predict(input, 0.01);
				int color = getColorFromModelOutput(output);
				lut->set(y, u, v, color);

			}
		}
	}
	mutex_model.unlock();
	lut->unlock();
}

void Worker::GetLocs(std::vector<LocStamped>& locs)
{
	mutex_locs.lock();
	for(int i=0;i<this->locs.size();i++)
	{
		locs.push_back(*this->locs[i]);
	}
	mutex_locs.unlock();
}

static double distanceSq(const pixelloc& p1, const pixelloc& p2) {
	double dx = p1.x - p2.x;
	double dy = p1.y - p2.y;
	return dx * dx + dy * dy;
}

static LocStamped* findNearestLoc(const std::vector<LocStamped*>& locs,
		const LocStamped& loc, double* dist, LocStamped* exceptThis = 0) {
	double minDistSq = 1e20;
	LocStamped* nearest = 0;
	for (int i = 0; i < locs.size(); i++) {
		if (locs[i]->clazz != loc.clazz)
			continue;
		double distSq = distanceSq(locs[i]->loc, loc.loc);
		if (distSq < minDistSq && exceptThis != locs[i]) {
			minDistSq = distSq;
			nearest = locs[i];
		}
	}
	*dist = sqrt(minDistSq);
	return nearest;
}

static void addLoc(std::vector<LocStamped*>& locs, LocStamped& loc) {
	double dist = 0;
	LocStamped* nearest = findNearestLoc(locs, loc, &dist);
	if (nearest != 0 && dist < 1) {
		*nearest = loc;
	} else {
		LocStamped* newLoc = new LocStamped;
		*newLoc = loc;
		locs.push_back(newLoc);
	}
}

void Worker::addRegion(
		const SSL_DetectionFrame * detection_frame,
		std::vector<LocStamped*>& locs, int clazz, float x, float y,
		double prop) {
	LocStamped locStamped;
	locStamped.clazz = clazz;
	locStamped.time = detection_frame->t_capture();
	locStamped.loc.x = x;
	locStamped.loc.y = y;
	locStamped.prop = prop;
	addLoc(locs, locStamped);
}

void Worker::addRegionCross(
		const SSL_DetectionFrame * detection_frame,
		std::vector<LocStamped*>& locs, int clazz, int targetClazz,
		CMVision::Region* region, int width, int height, int exclWidth,
		int exclHeight, int offset) {
	for (double i = -width / 2 - offset; i <= width / 2 + offset; i++) {
		if (abs(i) < exclWidth)
			continue;
		int x = region->cen_x + i;
		int y = region->cen_y;
		double prop = 1; //exp(-(i*i) / (2.0*width*width));
		addRegion(detection_frame, locs, targetClazz, x, y, prop);
	}
	for (int i = -height / 2 - offset; i <= height / 2 + offset; i++) {
		if (abs(i) < exclHeight)
			continue;
		int x = region->cen_x;
		int y = region->cen_y + i;
		double prop = 1; // = exp(-(i*i) / (2.0*height*height));
		addRegion(detection_frame, locs, targetClazz, x, y, prop);
	}
}

void Worker::addRegionEllipse(
		const SSL_DetectionFrame * detection_frame,
		std::vector<LocStamped*>& locs, int clazz, int targetClazz,
		CMVision::Region* region, int width, int height, int exclWidth,
		int exclHeight, int offset) {
	int maxY = height / 2 + offset;
	int maxX = width / 2 + offset;
	for (int y = -maxY; y <= maxY; y += 1) {
		for (int x = -maxX; x <= maxX; x += 1) {
			if (x * x * exclWidth * exclWidth / 4 + y * y * exclHeight * exclHeight / 4
					< exclWidth * exclWidth * exclHeight * exclHeight / 16)
			{
				continue;
			}
			if (x * x * maxY * maxY + y * y * maxX * maxX
					<= maxY * maxY * maxX * maxX) {
				int rx = region->cen_x + x;
				int ry = region->cen_y + y;

				double prop = 1;
				if (x * x * width * width / 4 + y * y * height * height / 4
						<= width * width * height * height / 16)
					addRegion(detection_frame, locs, targetClazz, rx, ry, prop);
				else
					addRegion(detection_frame, locs, -1, rx, ry, prop);
			}
		}
	}
}

static double distanceSq(const vector3d& p1, const vector3d& p2) {
	double dx = p1.x - p2.x;
	double dy = p1.y - p2.y;
	return dx * dx + dy * dy;
}

static BotPosStamped* findNearestBotPos(
		const std::vector<BotPosStamped*> botPoss, const vector3d& loc,
		double* dist, BotPosStamped* exceptThisBot = 0) {
	double minDistSq = 1e20;
	BotPosStamped* nearestBot = 0;
	for (int i = 0; i < botPoss.size(); i++) {
		double distSq = distanceSq(botPoss[i]->pos, loc);
		if (distSq < minDistSq && exceptThisBot != botPoss[i]) {
			minDistSq = distSq;
			nearestBot = botPoss[i];
		}
	}
	*dist = sqrt(minDistSq);
	return nearestBot;
}

//static RobotRegions* findNearestBotPos(
//		const std::vector<RobotRegions*> robotRegionsList, const vector3d& loc,
//		double* dist, RobotRegions* exceptThisBot = 0) {
//	double minDistSq = 1e20;
//	RobotRegions* nearestBot = 0;
//	for (int i = 0; i < robotRegionsList.size(); i++) {
//		double distSq = distanceSq(robotRegionsList[i]->pos, loc);
//		if (distSq < minDistSq && exceptThisBot != robotRegionsList[i]) {
//			minDistSq = distSq;
//			nearestBot = robotRegionsList[i];
//		}
//	}
//	*dist = sqrt(minDistSq);
//	return nearestBot;
//}

//void Worker::updateRobotRegions(
//		const SSL_DetectionFrame * detection_frame,
//		std::vector<RobotRegions*>& robotRegionsList) {
//	std::vector<SSL_DetectionRobot> robots;
//	robots.insert(robots.end(), detection_frame->robots_blue().begin(),
//			detection_frame->robots_blue().end());
//	robots.insert(robots.end(), detection_frame->robots_yellow().begin(),
//			detection_frame->robots_yellow().end());
//
//	for (std::vector<SSL_DetectionRobot>::iterator robot = robots.begin();
//			robot != robots.end(); robot++) {
//		vector3d loc;
//		loc.x = robot->x();
//		loc.y = robot->y();
//		double dist = 0;
//		RobotRegions* robotRegions = findNearestBotPos(robotRegionsList, loc,
//				&dist);
//		if (robotRegions == 0 || dist > 50) {
//			robotRegions = new RobotRegions;
//			robotRegionsList.push_back(robotRegions);
//		}
//		robotRegions->time = detection_frame->t_capture();
//		robotRegions->pos.x = robot->x();
//		robotRegions->pos.y = robot->y();
//		robotRegions->orientation = robot->orientation();
//
//		for (int i = 0; i < robot->colorregion_size(); i++) {
//			ColorRegion cr = robot->colorregion(i);
//			CMVision::Region reg;
//			reg.x1 = cr.x1();
//			reg.x2 = cr.x2();
//			reg.y1 = cr.y1();
//			reg.y2 = cr.y2();
//			reg.area = reg.width() * reg.height();
//			reg.cen_x = min(reg.x1, reg.x2) + reg.width() / 2.0;
//			reg.cen_y = min(reg.y1, reg.y2) + reg.height() / 2.0;
//			reg.color.v = cr.colorid();
//			robotRegions->regions.push_back(reg);
//		}
//	}
//
//	// remove old ones
//	for (std::vector<RobotRegions*>::iterator it = robotRegionsList.begin();
//			it != robotRegionsList.end();) {
//		RobotRegions* robotRegions = *it;
//		if ((detection_frame->t_capture() - robotRegions->time)
//				> robot_tracking_time) {
//			delete robotRegions;
//			robotRegions = 0;
//			it = robotRegionsList.erase(it);
//		} else {
//			it++;
//		}
//	}
//}

void Worker::updateBotPositions(const SSL_DetectionFrame * detection_frame,
		std::vector<BotPosStamped*>& botPoss) {
	std::vector<SSL_DetectionRobot> robots;
	robots.insert(robots.end(), detection_frame->robots_blue().begin(),
			detection_frame->robots_blue().end());
	robots.insert(robots.end(), detection_frame->robots_yellow().begin(),
			detection_frame->robots_yellow().end());

	for (std::vector<SSL_DetectionRobot>::iterator robot = robots.begin();
			robot != robots.end(); robot++) {
		vector3d loc;
		loc.x = robot->x();
		loc.y = robot->y();
		double dist = 0;
		BotPosStamped* nearestBot = findNearestBotPos(botPoss, loc, &dist);
		if (nearestBot != 0 && dist < 50) {
			nearestBot->time = detection_frame->t_capture();
			nearestBot->pos = loc;
			nearestBot->orientation = robot->orientation();
		} else {
			BotPosStamped* botPos = new BotPosStamped;
			botPos->time = detection_frame->t_capture();
			botPos->pos = loc;
			botPos->orientation = robot->orientation();
			botPoss.push_back(botPos);
		}
	}

	// remove old ones
	for (std::vector<BotPosStamped*>::iterator it = botPoss.begin();
			it != botPoss.end();) {
		BotPosStamped* botPos = *it;
		if ((detection_frame->t_capture() - botPos->time) > 2) {
			delete botPos;
			botPos = 0;
			it = botPoss.erase(it);
		} else {
			it++;
		}
	}
}

void Worker::updateLocs(const SSL_DetectionFrame * detection_frame,
		std::vector<LocStamped*>& locs) {
	// remove old ones
	for (std::vector<LocStamped*>::iterator it = locs.begin(); it != locs.end();
			) {
		LocStamped* loc = *it;
		if ((detection_frame->t_capture() - loc->time) > 0.2) {
			delete loc;
			loc = 0;
			it = locs.erase(it);
		} else {
			it++;
		}
	}
}

void Worker::regionDesiredPixelDim(CMVision::Region* region,
		int clazz, int& width, int& height) {
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

	width = pImg_left.x - pImg_right.x;
	height = pImg_top.y - pImg_bottom.y;
}

void Worker::regionFieldDim(CMVision::Region* region, int clazz,
		double& width, double& height) {
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

	width = pField_right.x - pField_left.x;
	height = pField_top.y - pField_bottom.y;
}

double normalizeAngle(double angle) {
	return (angle - (round((angle / (M_PI * 2)) - 1e-6) * M_PI * 2));
}

double angleDiff(double angle1, double angle2) {
	return normalizeAngle(normalizeAngle(angle1) - normalizeAngle(angle2));
}

double norm(vector2d vec) {
	return sqrt(vec.x * vec.x + vec.y * vec.y);
}

double getAngle(vector2d vec) {
	if (vec.x == 0 && vec.y == 0)
		return 0;
	double tmp = acos(vec.x / norm(vec));
	if (vec.y > 0) {
		return tmp;
	}
	return -tmp;
}

bool Worker::isInAngleRange(vector3d& pField, int clazz,
		BotPosStamped* botPos) {
	bool inAngleRange = true;
	if (cProp[clazz].nAngleRanges > 0) {
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

void Worker::processRegions(const SSL_DetectionFrame * detection_frame, std::vector<CMVision::Region>& regions)
{
	for(int i=0;i<regions.size(); i++)
	{
		CMVision::Region* region = &regions[i];
		int clazz = color2Clazz[region->color.v];
		vector2d pImg;
		pImg.x = region->cen_x;
		pImg.y = region->cen_y;
		vector3d pField;
		camera_parameters.image2field(pField, pImg,
				cProp[clazz].height);

//		if (pField.x
//				> -(field.field_length->getDouble()) / 2
//						- field.boundary_width->getDouble()
//				&& pField.x
//						< field.field_length->getDouble() / 2
//								+ field.boundary_width->getDouble()
//				&& pField.y
//						> -field.field_width->getDouble() / 2
//								- field.boundary_width->getDouble()
//				&& pField.y
//						< field.field_width->getDouble() / 2
//								+ field.boundary_width->getDouble()) {
			double dist;
			BotPosStamped* botPos = findNearestBotPos(botPoss, pField,
					&dist);
			if (dist < cProp[clazz].maxDist) {
				double fWidth, fHeight;
				regionFieldDim(region, clazz, fWidth, fHeight);

				if (dist < cProp[clazz].minDist
						|| !isInAngleRange(pField, clazz, botPos)) {
					// region too narrow / not in angle range -> unwanted
					mutex_locs.lock();
					addRegionCross(detection_frame, locs, clazz, -1,
							region, region->width(), region->height(),
							-1, -1, 0);
					mutex_locs.unlock();
				} else {
					int pWidth, pHeight;
					regionDesiredPixelDim(region, clazz, pWidth,
							pHeight);

					mutex_locs.lock();
					if (fWidth > cProp[clazz].radius * 2 + 10
							|| fHeight > cProp[clazz].radius * 2 + 10) {
						// region too large -> force decreasing size
						addRegionEllipse(detection_frame, locs, clazz,
								-1, region, region->width(),
								region->height(), pWidth, pHeight, 0);
					} else {
						addRegionEllipse(detection_frame, locs, clazz,
							clazz, region, pWidth, pHeight, -1, -1, 2);
					}
					mutex_locs.unlock();
				}
			} else if (_v_removeOutlierBlobs->getBool()) {
				mutex_locs.lock();
				addRegionCross(detection_frame, locs, clazz, -1, region,
						region->width(), region->height(), -1, -1, 0);
				mutex_locs.unlock();
			}
//		} else if (_v_removeOutlierBlobs->getBool()) {
//			mutex_locs.lock();
//			addRegionCross(detection_frame, locs, clazz, -1, region,
//					region->width(), region->height(), -1, -1, 0);
//			mutex_locs.unlock();
//		}
	}
}

ProcessResult PluginOnlineColorCalib::process(FrameData * frame,
		RenderOptions * options) {
	(void) options;
	if (frame == 0)
		return ProcessingFailed;

	if (_v_enable->getBool()) {
		ColorFormat source_format = frame->video.getColorFormat();
		if (source_format != COLOR_YUV422_UYVY) {
			std::cerr << "Unsupported source format: " << source_format
					<< std::endl;
			_v_enable->setBool(false);
			return ProcessingFailed;
		}

		worker->update(frame);

		if(_v_debug->getBool())
		{
	//		auto t1 = std::chrono::system_clock::now();
	//		std::chrono::duration<double> diff = std::chrono::system_clock::now()-t1;
			Image<raw8> * img_debug;
			if ((img_debug = (Image<raw8> *) frame->map.get(
					"cmv_online_color_calib")) == 0) {
				img_debug = (Image<raw8> *) frame->map.insert(
						"cmv_online_color_calib", new Image<raw8>());
			}
			img_debug->allocate(frame->video.getWidth(), frame->video.getHeight());
			img_debug->fillColor(0);

			std::vector<LocStamped> locs;
			worker->GetLocs(locs);
			for (int i = 0; i <locs.size(); i++) {
				raw8 color;
				if (locs[i].clazz >= 0)
					color = worker->cProp[locs[i].clazz].color;
				else
					color = 1;
				img_debug->setPixel(locs[i].loc.x, locs[i].loc.y, color);
			}
		}
	}

	Image<raw8> * img_thresholded;
	if ((img_thresholded = (Image<raw8> *) frame->map.get(
			"cmv_learned_threshold")) == 0) {
		img_thresholded = (Image<raw8> *) frame->map.insert(
				"cmv_learned_threshold", new Image<raw8>());
	}
	img_thresholded->allocate(frame->video.getWidth(),
			frame->video.getHeight());
	CMVisionThreshold::thresholdImageYUV422_UYVY(img_thresholded,
			&(frame->video), &worker->local_lut);

	return ProcessingOk;
}

VarList * PluginOnlineColorCalib::getSettings() {
	return _settings;
}

string PluginOnlineColorCalib::getName() {
	return "OnlineColorCalib";
}
