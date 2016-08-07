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

PluginOnlineColorCalib::PluginOnlineColorCalib(FrameBuffer * _buffer, LUT3D * lut, const CameraParameters& camera_params, const RoboCupField& field)
	: VisionPlugin(_buffer), camera_parameters(camera_params), field(field), cProp(5)
{

	_lut=lut;
	_settings=new VarList("Online Color Calib");

	_settings->addChild(_updGlob=new VarTrigger("UpdateGlob","Update Global LUT"));
	_settings->addChild(_updVis=new VarTrigger("UpdateVis","Update Visualization"));
	connect(_updGlob,SIGNAL(signalTriggered()),this,SLOT(slotUpdateTriggered()));
	connect(_updVis,SIGNAL(signalTriggered()),this,SLOT(slotUpdateVisTriggered()));
	_settings->addChild(_v_enable=new VarBool("enabled", false));
	_settings->addChild(_v_lifeUpdate=new VarBool("life update", false));
	_settings->addChild(_v_removeOutlierBlobs=new VarBool("remove outlier blobs", false));

	cProp[0].color = CH_ORANGE;
	cProp[0].height = 21;
	cProp[0].minDist = 150;
	cProp[0].maxDist = 20000;
	cProp[0].radius = 21;
	cProp[0].nAngleRanges = 0;

	cProp[1].color = CH_YELLOW;
	cProp[2].color = CH_BLUE;
	for(int i=1;i<3;i++)
	{
		cProp[i].height = 130;
		cProp[i].minDist = 0;
		cProp[i].maxDist = 20;
		cProp[i].radius = 25;
		cProp[i].nAngleRanges = 0;
	}

	cProp[3].color = CH_PINK;
	cProp[4].color = CH_GREEN;
	for(int i=3;i<5;i++)
	{
		cProp[i].height = 130;
		cProp[i].minDist = 40;
		cProp[i].maxDist = 90;
		cProp[i].radius = 20;
		cProp[i].nAngleRanges = 2;
		cProp[i].angleRanges[0].min = 40.0/180.0 * M_PI;
		cProp[i].angleRanges[0].max = 75.0/180.0 * M_PI;
		cProp[i].angleRanges[1].min = 125.0/180.0 * M_PI;
		cProp[i].angleRanges[1].max = 165.0/180.0 * M_PI;
	}

	model = new LWPR_Object(3,cProp.size());
	doubleVec norm(3, 255);
	model->normIn(norm);
	model->setInitD(500);
	model->wGen(0.1);
}

PluginOnlineColorCalib::~PluginOnlineColorCalib() {
}


void PluginOnlineColorCalib::slotUpdateTriggered() {
  lock();
  CopytoLUT(_lut);
  unlock();
}

void PluginOnlineColorCalib::slotUpdateVisTriggered() {
  lock();
  CopytoLUT(&lut);
  unlock();
}

bool PluginOnlineColorCalib::isInAnyRegion(CMVision::ColorRegionList * colorlist, pixelloc& loc, double maxArea, double margin)
{
	for(int r= 0; r<colorlist->getNumColorRegions(); r++)
	{
		CMVision::Region* region = colorlist->getRegionList(r).getInitialElement();
		while(region != 0)
		{
			if(region->area < maxArea &&
				loc.x >= region->x1 - margin && loc.x <= region->x2 + margin
				&& loc.y >= region->y1 - margin && loc.y <= region->y2 + margin)
			{
				return true;
			}
			region = region->next;
		}
	}
	return false;
}

void PluginOnlineColorCalib::updateModel(FrameData * frame, pixelloc& loc, int clazz)
{
	yuv color;
	uyvy color2 = *((uyvy*)(frame->video.getData() + (sizeof(uyvy) * (((loc.y * (frame->video.getWidth())) + loc.x) / 2))));
	color.u=color2.u;
	color.v=color2.v;
	if ((loc.x % 2)==0) {
		color.y=color2.y1;
	} else {
		color.y=color2.y2;
	}

	doubleVec v_in(3);
	doubleVec v_out(cProp.size());
	v_in[0] = color.y;
	v_in[1] = color.u;
	v_in[2] = color.v;
	for(int i=0;i<v_out.size();i++)
		v_out[i] = (clazz==i);

	doubleVec out = model->update(v_in, v_out);
	int col = getColorFromModelOutput(out);
	lut.set(color.y,color.u,color.v,col);
	if(_v_lifeUpdate->getBool())
	{
		_lut->set(color.y,color.u,color.v,col);
	}
}

static double distanceSq(const pixelloc& p1, const pixelloc& p2)
{
	double dx = p1.x-p2.x;
	double dy = p1.y-p2.y;
	return dx*dx+dy*dy;
}

static LocStamped* findNearestLoc(const std::vector<LocStamped*>& locs, const LocStamped& loc, double* dist, LocStamped* exceptThis = 0)
{
	double minDistSq = 1e20;
	LocStamped* nearest = 0;
	for(int i=0;i<locs.size();i++)
	{
		if(locs[i]->clazz != loc.clazz) continue;
		double distSq = distanceSq(locs[i]->loc, loc.loc);
		if(distSq < minDistSq && exceptThis != locs[i])
		{
			minDistSq = distSq;
			nearest = locs[i];
		}
	}
	*dist = sqrt(minDistSq);
	return nearest;
}

static void addLoc(std::vector<LocStamped*>& locs, LocStamped& loc)
{
	double dist = 0;
	LocStamped* nearest = findNearestLoc(locs, loc, &dist);
	if(nearest != 0 && dist < 1)
	{
		*nearest = loc;
	} else {
		LocStamped* newLoc = new LocStamped;
		*newLoc = loc;
		locs.push_back(newLoc);
	}
}

static void addRegion(
		const SSL_DetectionFrame * detection_frame,
		std::vector<LocStamped*>& locs,
		int clazz,
		float x, float y,
		double prop = 1)
{
	LocStamped locStamped;
	locStamped.clazz = clazz;
	locStamped.time = detection_frame->t_capture();
	locStamped.loc.x = x;
	locStamped.loc.y = y;
	locStamped.prop = prop;
	addLoc(locs, locStamped);
}

void PluginOnlineColorCalib::addRegionCross(
		const SSL_DetectionFrame * detection_frame,
		std::vector<LocStamped*>& locs,
		int clazz,
		int targetClazz,
		CMVision::Region* region,
		int width,
		int height,
		int exclWidth,
		int exclHeight,
		int offset)
{
	for(double i=-width/2-offset;i<=width/2+offset;i++)
	{
		if(abs(i) < exclWidth) continue;
		int x = region->cen_x + i;
		int y = region->cen_y;
		double prop = 1; //exp(-(i*i) / (2.0*width*width));
		addRegion(detection_frame, locs, targetClazz, x, y, prop);
	}
	for(int i=-height/2-offset;i<=height/2+offset;i++)
	{
		if(abs(i) < exclHeight) continue;
		int x = region->cen_x;
		int y = region->cen_y + i;
		double prop = 1; // = exp(-(i*i) / (2.0*height*height));
		addRegion(detection_frame, locs, targetClazz, x, y, prop);
	}
}

void PluginOnlineColorCalib::addRegionEllipse(
		const SSL_DetectionFrame * detection_frame,
		std::vector<LocStamped*>& locs,
		int clazz,
		int targetClazz,
		CMVision::Region* region,
		int width,
		int height,
		int exclWidth,
		int exclHeight,
		int offset)
{
	int maxY = height/2+offset;
	int maxX = width/2+offset;
	for(int y=-maxY; y<=maxY; y+=2) {
		if(abs(y) < exclHeight) continue;
	    for(int x=-maxX; x<=maxX; x+=2) {
			if(abs(x) < exclWidth) continue;
	        if(x*x*maxY*maxY+y*y*maxX*maxX <= maxY*maxY*maxX*maxX)
	        {
	    		int rx = region->cen_x + x;
	    		int ry = region->cen_y + y;

//	    		double prop = exp(-((x*x) / (width*width) + (y*y) / (height*height))/2.0);
	    		double prop = 1;
//	    		if(abs(y) > height/2 || abs(x) > width/2)
				if(x*x*width*width/4+y*y*height*height/4 <= width*width*height*height/16)
	    			addRegion(detection_frame, locs, targetClazz, rx, ry, prop);
	    		else
	    			addRegion(detection_frame, locs, -1, rx, ry, prop);
	        }
	    }
	}
}

static double distanceSq(const vector3d& p1, const vector3d& p2)
{
	double dx = p1.x-p2.x;
	double dy = p1.y-p2.y;
	return dx*dx+dy*dy;
}

static BotPosStamped* findNearestBotPos(const std::vector<BotPosStamped*> botPoss, const vector3d& loc, double* dist, BotPosStamped* exceptThisBot = 0)
{
	double minDistSq = 1e20;
	BotPosStamped* nearestBot = 0;
	for(int i=0;i<botPoss.size();i++)
	{
		double distSq = distanceSq(botPoss[i]->pos, loc);
		if(distSq < minDistSq && exceptThisBot != botPoss[i])
		{
			minDistSq = distSq;
			nearestBot = botPoss[i];
		}
	}
	*dist = sqrt(minDistSq);
	return nearestBot;
}

static void updateBotPositions(const SSL_DetectionFrame * detection_frame, std::vector<BotPosStamped*>& botPoss)
{
	std::vector<SSL_DetectionRobot> robots;
	robots.insert(robots.end(),
			detection_frame->robots_blue().begin(),
			detection_frame->robots_blue().end());
	robots.insert(robots.end(),
			detection_frame->robots_yellow().begin(),
			detection_frame->robots_yellow().end());

	for(std::vector<SSL_DetectionRobot>::iterator robot = robots.begin(); robot != robots.end(); robot++)
	{
		vector3d loc;
		loc.x = robot->x();
		loc.y = robot->y();
		double dist = 0;
		BotPosStamped* nearestBot = findNearestBotPos(botPoss, loc, &dist);
		if(nearestBot != 0 && dist < 50)
		{
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
	for(std::vector<BotPosStamped*>::iterator it = botPoss.begin(); it != botPoss.end();)
	{
		BotPosStamped* botPos = *it;
		if((detection_frame->t_capture() - botPos->time) > 2)
		{
			delete botPos;
			botPos = 0;
			it = botPoss.erase(it);
		} else {
			it++;
		}
	}
}

static void updateLocs(const SSL_DetectionFrame * detection_frame, std::vector<LocStamped*>& locs)
{
	// remove old ones
	for(std::vector<LocStamped*>::iterator it = locs.begin(); it != locs.end();)
	{
		LocStamped* loc = *it;
		if((detection_frame->t_capture() - loc->time) > 0.2)
		{
			delete loc;
			loc = 0;
			it = locs.erase(it);
		} else {
			it++;
		}
	}
}

void PluginOnlineColorCalib::regionDesiredPixelDim(CMVision::Region* region, int clazz, int& width, int& height)
{
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

void PluginOnlineColorCalib::regionFieldDim(CMVision::Region* region, int clazz, double& width, double& height)
{
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
	camera_parameters.image2field(pField_right, pImg_right, cProp[clazz].height);
	camera_parameters.image2field(pField_bottom, pImg_bottom, cProp[clazz].height);
	camera_parameters.image2field(pField_top, pImg_top, cProp[clazz].height);

	width = pField_right.x - pField_left.x;
	height = pField_top.y - pField_bottom.y;
}

double normalizeAngle(double angle) {
	// Don't call this a hack! It's numeric!
	return (angle - (round((angle / (M_PI*2)) - 1e-6) * M_PI*2));
}

double angleDiff(double angle1, double angle2) {
	return normalizeAngle(normalizeAngle(angle1) - normalizeAngle(angle2));
}

double norm(vector2d vec)
{
	return sqrt(vec.x*vec.x+vec.y*vec.y);
}

double getAngle(vector2d vec)
{
	if(vec.x == 0 && vec.y == 0)
		return 0;
	double tmp = acos( vec.x / norm(vec) );
	if(vec.y > 0)
	{
		return tmp;
	}
	return -tmp;
}

bool PluginOnlineColorCalib::isInAngleRange(vector3d& pField, int clazz, BotPosStamped* botPos)
{
	bool inAngleRange = true;
	if(cProp[clazz].nAngleRanges > 0)
	{
		inAngleRange = false;
		vector2d regionDir;
		regionDir.x = pField.x - botPos->pos.x;
		regionDir.y = pField.y - botPos->pos.y;
		double regionAngle = getAngle(regionDir);
		double diff = std::abs(angleDiff(botPos->orientation, regionAngle));

		for(int i=0;i<cProp[clazz].nAngleRanges;i++)
		{
			if(diff > cProp[clazz].angleRanges[i].min
				&& diff < cProp[clazz].angleRanges[i].max)
			{
				inAngleRange = true;
				break;
			}
		}
	}
	return inAngleRange;
}

ProcessResult PluginOnlineColorCalib::process(FrameData * frame, RenderOptions * options)
{
	(void)options;
	if (frame==0) return ProcessingFailed;

	ColorFormat source_format=frame->video.getColorFormat();
	if (source_format!=COLOR_YUV422_UYVY) {
//		std::cerr << "Unsupported source format: " << source_format << std::endl;
		return ProcessingFailed;
	}

	Image<raw8> * img_thresholded;
	if ((img_thresholded=(Image<raw8> *)frame->map.get("cmv_learned_threshold")) == 0) {
		img_thresholded=(Image<raw8> *)frame->map.insert("cmv_learned_threshold",new Image<raw8>());
	}
	img_thresholded->allocate(frame->video.getWidth(),frame->video.getHeight());

	Image<raw8> * img_debug;
	if ((img_debug=(Image<raw8> *)frame->map.get("cmv_online_color_calib")) == 0) {
		img_debug=(Image<raw8> *)frame->map.insert("cmv_online_color_calib",new Image<raw8>());
	}
	img_debug->allocate(frame->video.getWidth(),frame->video.getHeight());
	img_debug->fillColor(0);

	if(_v_enable->getBool())
	{
		SSL_DetectionFrame * detection_frame = (SSL_DetectionFrame *) frame->map.get("ssl_detection_frame");
		if(detection_frame == 0) {
			printf("no detection frame\n");
			return ProcessingFailed;
		}

		CMVision::ColorRegionList * colorlist;
		colorlist=(CMVision::ColorRegionList *)frame->map.get("cmv_colorlist");
		if (colorlist==0) {
			printf("error in robot detection plugin: no region-lists were found!\n");
			return ProcessingFailed;
		}

		updateBotPositions(detection_frame, botPoss);

		for(int clazz=0;clazz<cProp.size();clazz++)
		{
			CMVision::Region* region = colorlist->getRegionList(cProp[clazz].color).getInitialElement();
			while(region != 0)
			{
				vector2d pImg;
				pImg.x = region->cen_x;
				pImg.y = region->cen_y;
				vector3d pField;
				camera_parameters.image2field(pField, pImg, cProp[clazz].height);

				if(
					pField.x > -(field.field_length->getDouble())/2 - field.boundary_width->getDouble()
					&& pField.x < field.field_length->getDouble()/2 + field.boundary_width->getDouble()
					&& pField.y > -field.field_width->getDouble()/2 - field.boundary_width->getDouble()
					&& pField.y < field.field_width->getDouble()/2 + field.boundary_width->getDouble()
				)
				{
					double dist;
					BotPosStamped* botPos = findNearestBotPos(botPoss, pField, &dist);
					if(dist < cProp[clazz].maxDist)
					{
						double fWidth, fHeight;
						regionFieldDim(region, clazz, fWidth, fHeight);

						if(dist < cProp[clazz].minDist
							|| !isInAngleRange(pField, clazz, botPos))
						{
							// region too narrow / not in angle range -> unwanted
							addRegionCross(detection_frame, locs, clazz, -1, region,
									region->width(), region->height(),
									-1, -1,
									0);
						} else
						{
							int pWidth, pHeight;
							regionDesiredPixelDim(region, clazz, pWidth, pHeight);

							if(fWidth > cProp[clazz].radius*2 + 10
							|| fHeight > cProp[clazz].radius*2 + 10)
							{
								// region too large -> force decreasing size
								addRegionEllipse(detection_frame, locs, clazz, -1, region,
										region->width(), region->height(),
										pWidth, pHeight,
										0);
							}
							addRegionEllipse(detection_frame, locs, clazz, clazz, region,
									pWidth, pHeight,
									-1, -1,
									2);
						}
					} else if(_v_removeOutlierBlobs->getBool()){
						addRegionCross(detection_frame, locs, clazz, -1, region,
										region->width(), region->height(),
										-1, -1,
										0);
					}
				}
				region = region->next;
			}
		}

		updateLocs(detection_frame, locs);

		for(int i=0;i<locs.size();i++)
		{
			updateModel(frame, locs[i]->loc, locs[i]->clazz);
			raw8 color;
			if(locs[i]->clazz >= 0)
				color = cProp[locs[i]->clazz].color;
			else
				color = 1;
			img_debug->setPixel(locs[i]->loc.x, locs[i]->loc.y, color);
		}

		// add some random samples
//		pixelloc loc;
//		vector2d pImg;
//		vector3d pField;
//		int rndSamples = std::min(100, (int)locs.size());
//		int rs = rndSamples;
//		for(int i=0; i<1000; i++)
//		{
//			int length = (int) (field.field_length->getInt()); // + 2*field.boundary_width->getInt());
//			int width = (int) (field.field_width->getInt()); // + 2*field.boundary_width->getInt());
//			pField.x = (rand() % length) - length/2;
//			pField.y = (rand() % width) - width/2;
//			camera_parameters.field2image(pField, pImg);
//			if(pImg.x > 0 && pImg.x < frame->video.getWidth() &&
//					pImg.y > 0 && pImg.y < frame->video.getHeight())
//			{
//				loc.x = pImg.x;
//				loc.y = pImg.y;
//				if(!isInAnyRegion(colorlist, loc, 200, 10))
//				{
//					updateModel(frame, loc, -1);
//					img_debug->setPixel(pImg.x, pImg.y, 1);
//					rndSamples--;
//					if(rndSamples <= 0)
//						break;
//				}
//			}
//		}

//		std::chrono::duration<double> diff = std::chrono::system_clock::now()-lastUpdate;
//		if(diff.count() > 5)
//		{
//			CopytoLUT(&lut);
//			lastUpdate = std::chrono::system_clock::now();
//		}
	}

	CMVisionThreshold::thresholdImageYUV422_UYVY(img_thresholded,&(frame->video),&lut);

	return ProcessingOk;
}

int PluginOnlineColorCalib::getColorFromModelOutput(doubleVec& output)
{
	int maxIdx = 0;
	double maxValue = 0;
	for(int i=0;i<cProp.size();i++)
	{
		if(output[i] > maxValue)
		{
			maxValue = output[i];
			maxIdx = i;
		}
	}
	if(maxValue > 0.5)
		return cProp[maxIdx].color;
	return 0;
}

void PluginOnlineColorCalib::CopytoLUT(LUT3D *lut) {

    doubleVec input(3);
    doubleVec output(cProp.size());

    lut->lock();
	for (int y = 0; y <= 255; y+= (0x1 << _lut->X_SHIFT)) {
		for (int u = 0; u <= 255; u+= (0x1 << _lut->Y_SHIFT)) {
			for (int v = 0; v <= 255; v+= (0x1 << _lut->Z_SHIFT)) {
				input[0]= (double)y;
				input[1]= (double)u;
				input[2]= (double)v;
				output = model->predict(input, 0.01);
				int color = getColorFromModelOutput(output);
				lut->set(y, u, v, color);

			}
		}
	}
	lut->unlock();
}


VarList * PluginOnlineColorCalib::getSettings() {
  return _settings;
}

string PluginOnlineColorCalib::getName() {
  return "OnlineColorCalib";
}
