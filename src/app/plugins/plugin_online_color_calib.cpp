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
#include <opencv2/opencv.hpp>

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
	worker->running = false;
}

void PluginOnlineColorCalib::slotUpdateTriggered() {
	worker->globalLutUpdate = true;
}

void PluginOnlineColorCalib::slotResetModelTriggered() {
	worker->ResetModel();
}

Worker::Worker(LUT3D * lut, const CameraParameters& camera_params, const RoboCupField& field) :
		cProp(5), color2Clazz(10, 0), camera_parameters(camera_params), field(field),
		local_lut(8,8,8) {
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

	ResetModel();

	robot_tracking_time = 10;
	loc_tracking_time = 5.0;
	max_locs = 10000;
	max_regions = 200;
	input = inputData;
	inputIdx = 0;
	inputData[0].number = -1;
	inputData[1].number = -1;
	globalLutUpdate = false;
	running = true;
}

Worker::~Worker() {
}

void Worker::ResetModel()
{
	mutex_model.lock();

	local_lut.reset();

	for(int i=0;i<models.size();i++)
		delete models[i];
	models.clear();

	for(int i=0;i<cProp.size();i++)
	{
		LWPR_Object* model = new LWPR_Object(3,1);
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
				for(int i=0;i<cProp.size();i++)
				{
					doubleVec out = models[i]->predict(input, 0.01);
					output[i] = out[0];
				}
				int color = getColorFromModelOutput(output);
				lut->set(y, u, v, color);

			}
		}
	}
	mutex_model.unlock();
	lut->unlock();
}

void Worker::process() {

	long long int lastNumber = -1;
	while (running) {
		WorkerInput* input;
		{
			std::unique_lock<std::mutex> lock(this->mutex_sync);
			this->d_condition.wait(lock,
					[=] {return this->input->number != -1 && ( this->input->number > lastNumber );});
			mutex_input.lock();
			lastNumber = this->input->number;
			input = this->input;
			inputIdx = (inputIdx+1) % 2;
			this->input = inputData + inputIdx;
			mutex_input.unlock();
		}

		auto t3 = std::chrono::system_clock::now();
		processRegions(input->image, input->time, input->regions);
		auto t4 = std::chrono::system_clock::now();

		updateLocs(input->time);

		auto t5 = std::chrono::system_clock::now();
		for (int i = 0; i < locs.size(); i++) {
			updateModel(input->image, locs[i]->loc, locs[i]->clazz);
		}
		auto t6 = std::chrono::system_clock::now();
		std::chrono::duration<double> tRegions = (t4-t3);
		std::chrono::duration<double> tModel = (t6-t5);
		std::chrono::duration<double> tProc = (t6-t3);
//		std::cout << locs.size() << " " << botPoss.size() << " " << input->regions.size() << std::endl;
		std::cout << "Proc time:" <<
				" tRegions=" << tRegions.count() <<
				" tModel=" << tModel.count() <<
				" tProc=" << tProc.count() <<
				std::endl;

		std::cout << "lwpr: ";
		for(int i=0;i<models.size();i++)
		{
			std::cout << models[i]->numRFS(0) << " ";
		}
		std::cout << std::endl;

		mutex_locs.lock();
		locs_out.clear();
		for(int i=0;i<this->locs.size();i++)
		{
			Loc loc;
			loc.x = this->locs[i]->loc.x;
			loc.y = this->locs[i]->loc.y;
			loc.clazz = this->locs[i]->clazz;
			locs_out.push_back(loc);
		}
		mutex_locs.unlock();

		if(globalLutUpdate)
		{
			auto t1 = std::chrono::system_clock::now();
			CopytoLUT(global_lut);
			std::chrono::duration<double> diff = std::chrono::system_clock::now()-t1;
			std::cout << "LUT updated in " << diff.count() << "s" << std::endl;
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
	updateBotPositions(detection_frame);

	mutex_input.lock();
	this->input->number++;
	this->input->time = detection_frame->t_capture();
	this->input->image.deepCopyFromRawImage(frame->video, true);
	this->input->regions.clear();
	int nReg = 0;
	for (int clazz = 0; clazz < cProp.size(); clazz++) {
		CMVision::Region* region = colorlist->getRegionList(
				cProp[clazz].color).getInitialElement();
		while (region != 0 && nReg < max_regions) {
			this->input->regions.push_back(*region);
			region = region->next;
			nReg++;
		}
	}
	if(nReg == max_regions)
	{
		std::cout << "Too many regions." << std::endl;
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
	v_in[0] = color.y;
	v_in[1] = color.u;
	v_in[2] = color.v;

	//	if(clazz != 0 && clazz != 3) return;

	mutex_model.lock();
	doubleVec output(models.size());

	doubleVec v_out(1);
	if(clazz >= 0)
	{
//		v_out[0] = 1;
//		doubleVec out = models[clazz]->update(v_in, v_out);
//		output[clazz] = out[0];
		for(int i=0;i<models.size();i++)
		{
			v_out[0] = (clazz == i);
			doubleVec out = models[i]->update(v_in, v_out);
//			doubleVec out = models[i]->predict(v_in, 0.001);
			output[i] = out[0];
		}
	}
	else {
		v_out[0] = 0;
		for(int i=0;i<models.size();i++)
		{
			doubleVec out = models[i]->update(v_in, v_out);
			output[i] = out[0];
		}
	}


	int col = getColorFromModelOutput(output);
//	local_lut.set(color.y, color.u, color.v, col);
	local_lut.set(color.y, color.u, color.v, cProp[clazz].color);
	if (_v_lifeUpdate->getBool()) {
//		global_lut->set(color.y, color.u, color.v, col);
		global_lut->set(color.y, color.u, color.v, cProp[clazz].color);
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

void Worker::updateLocs(double time) {
	// remove old ones
	for (std::vector<LocStamped*>::iterator it = locs.begin(); it != locs.end();
			) {
		LocStamped* loc = *it;
		if ((time - loc->time) > loc_tracking_time) {
			delete loc;
			loc = 0;
			mutex_locs.lock();
			it = locs.erase(it);
			mutex_locs.unlock();
		} else {
			it++;
		}
	}
}

static double distanceSq(const pixelloc& p1, const pixelloc& p2) {
	double dx = p1.x - p2.x;
	double dy = p1.y - p2.y;
	return dx * dx + dy * dy;
}

LocStamped* Worker::findNearestLoc(const LocStamped& loc,
		double* dist, LocStamped* exceptThis) {
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

void Worker::addLoc(double time, int clazz, float x, float y) {
	LocStamped loc;
	loc.clazz = clazz;
	loc.time = time;
	loc.loc.x = x;
	loc.loc.y = y;

	double dist = 0;
	LocStamped* nearest = findNearestLoc(loc, &dist);
	if (nearest != 0 && dist < 1) {
		*nearest = loc;
	} else {
		LocStamped* newLoc = new LocStamped;
		*newLoc = loc;
		mutex_locs.lock();
		locs.push_back(newLoc);
		if(locs.size() > max_locs)
		{
			int idx = rand() % locs.size();
			LocStamped* oldLoc = locs[idx];
			locs.erase(locs.begin() + idx);
			delete oldLoc;
		}
		mutex_locs.unlock();
	}
}

void Worker::addRegionCross(double time, int clazz, int targetClazz,
		CMVision::Region* region, int width, int height, int exclWidth,
		int exclHeight, int offset) {
	for (double i = -width / 2 - offset; i <= width / 2 + offset; i++) {
		if (abs(i) < exclWidth)
			continue;
		int x = region->cen_x + i;
		int y = region->cen_y;
		addLoc(time, targetClazz, x, y);
	}
	for (int i = -height / 2 - offset; i <= height / 2 + offset; i++) {
		if (abs(i) < exclHeight)
			continue;
		int x = region->cen_x;
		int y = region->cen_y + i;
		addLoc(time, targetClazz, x, y);
	}
}

void Worker::addRegionEllipse(double time, int clazz, int targetClazz,
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

				if (x * x * width * width / 4 + y * y * height * height / 4
						<= width * width * height * height / 16)
					addLoc(time, targetClazz, rx, ry);
				else
					addLoc(time, -1, rx, ry);
			}
		}
	}
}

void Worker::addRegionKMeans(RawImage * img, double time, int targetClazz,
		CMVision::Region* region, int width, int height, int offset) {

	Blob blob;
	blob.center.x = region->cen_x;
	blob.center.y = region->cen_y;
	blob.height = height;
	blob.width = width;
	bool ok = blobDetector.detectBlob(img, &blob, 0);
	if(ok)
	{
		for(int i=0;i<blob.detectedPixels.size();i++)
		{
			int x = blob.center.x + blob.detectedPixels[i].x;
			int y = blob.center.y + blob.detectedPixels[i].y;
			addLoc(time, targetClazz, x, y);
		}
	}

}

static double distanceSq(const vector3d& p1, const vector3d& p2) {
	double dx = p1.x - p2.x;
	double dy = p1.y - p2.y;
	return dx * dx + dy * dy;
}

BotPosStamped* Worker::findNearestBotPos(const vector3d& loc,
		double* dist, BotPosStamped* exceptThisBot) {
	double minDistSq = 1e20;
	BotPosStamped* nearestBot = 0;
	mutex_botPoss.lock();
	for (int i = 0; i < botPoss.size(); i++) {
		double distSq = distanceSq(botPoss[i]->pos, loc);
		if (distSq < minDistSq && exceptThisBot != botPoss[i]) {
			minDistSq = distSq;
			nearestBot = botPoss[i];
		}
	}
	mutex_botPoss.unlock();
	*dist = sqrt(minDistSq);
	return nearestBot;
}

void Worker::updateBotPositions(const SSL_DetectionFrame * detection_frame) {
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
		loc.z = robot->height();
		double dist = 0;
		BotPosStamped* nearestBot = findNearestBotPos(loc, &dist);
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
		if ((detection_frame->t_capture() - botPos->time) > robot_tracking_time) {
			delete botPos;
			botPos = 0;
			mutex_botPoss.lock();
			it = botPoss.erase(it);
			mutex_botPoss.unlock();
		} else {
			it++;
		}
	}
}

void Worker::getRegionDesiredPixelDim(CMVision::Region* region,
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

	width = std::abs(pImg_left.x - pImg_right.x);
	height = std::abs(pImg_top.y - pImg_bottom.y);
}

void Worker::getRegionFieldDim(CMVision::Region* region, int clazz,
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

	width = std::abs(pField_right.x - pField_left.x);
	height = std::abs(pField_top.y - pField_bottom.y);
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

bool regionByAreaSorter (const CMVision::Region& r1, const CMVision::Region& r2) { return (r1.area<r2.area); }

void Worker::processRegions(RawImage& img, double time, std::vector<CMVision::Region>& regions)
{
//	std::sort(regions.begin(), regions.end(), regionByAreaSorter);
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
			BotPosStamped* botPos = findNearestBotPos(pField,
					&dist);
			if (dist < cProp[clazz].maxDist) {
				double fWidth, fHeight;
				getRegionFieldDim(region, clazz, fWidth, fHeight);

				if (dist < cProp[clazz].minDist
						|| !isInAngleRange(pField, clazz, botPos)) {
					// region too narrow / not in angle range -> unwanted
					addRegionCross(time, clazz, -1,
							region, region->width(), region->height(),
							-1, -1, 0);
				} else {
					int pWidth, pHeight;
					getRegionDesiredPixelDim(region, clazz, pWidth,
							pHeight);

					addRegionKMeans(&img, time, clazz, region, pWidth, pHeight, 2);
//					if (fWidth > cProp[clazz].radius * 2 + 10
//							|| fHeight > cProp[clazz].radius * 2 + 10) {
//						// region too large -> force decreasing size
//						addRegionEllipse(time, clazz,
//								-1, region, region->width(),
//								region->height(), pWidth, pHeight, 0);
//					} else {
//						addRegionEllipse(time, clazz,
//							clazz, region, pWidth, pHeight, -1, -1, 2);
//					}
				}
			} else if (_v_removeOutlierBlobs->getBool()) {
				addRegionCross(time, clazz, -1, region,
						region->width(), region->height(), -1, -1, 0);
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

	ColorFormat source_format = frame->video.getColorFormat();

	if (_v_enable->getBool()) {

		if (source_format != COLOR_YUV422_UYVY) {
			std::cerr << "Unsupported source format: " << source_format
					<< std::endl;
			_v_enable->setBool(false);
			return ProcessingFailed;
		}

		worker->update(frame);

		if(_v_debug->getBool())
		{
			Image<raw8> * img_debug;
			if ((img_debug = (Image<raw8> *) frame->map.get(
					"cmv_online_color_calib")) == 0) {
				img_debug = (Image<raw8> *) frame->map.insert(
						"cmv_online_color_calib", new Image<raw8>());
			}
			img_debug->allocate(frame->video.getWidth(), frame->video.getHeight());
			img_debug->fillColor(0);

			worker->mutex_locs.lock();
			for (int i = 0; i < worker->locs_out.size(); i++) {
				raw8 color;
				if (worker->locs_out[i].clazz >= 0)
					color = worker->cProp[worker->locs_out[i].clazz].color;
				else
					color = 1;
				img_debug->setPixel(worker->locs_out[i].x, worker->locs_out[i].y, color);
			}
			worker->mutex_locs.unlock();

			worker->mutex_botPoss.lock();
			for(int i=0;i<worker->botPoss.size();i++)
			{
				BotPosStamped* bp = worker->botPoss[i];
				vector2d pField;
				worker->camera_parameters.field2image(bp->pos, pField);
				int w = 40;
				img_debug->drawBox(pField.x-w/2, pField.y-w/2, w, w, 6);
			}
			worker->mutex_botPoss.unlock();

		}
	}

	if (source_format == COLOR_YUV422_UYVY) {
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
	}

	return ProcessingOk;
}

VarList * PluginOnlineColorCalib::getSettings() {
	return _settings;
}

string PluginOnlineColorCalib::getName() {
	return "OnlineColorCalib";
}
