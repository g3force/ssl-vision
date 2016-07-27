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
#include "messages_robocup_ssl_detection.pb.h"
#include "geometry.h"

#define CH_ORANGE 2
#define CH_YELLOW 3
#define CH_BLUE 4
#define CH_PINK 5
#define CH_GREEN 7

PluginOnlineColorCalib::PluginOnlineColorCalib(FrameBuffer * _buffer, LUT3D * lut, const CameraParameters& camera_params, const RoboCupField& field)
	: VisionPlugin(_buffer), camera_parameters(camera_params), field(field)
{

	_lut=lut;
	_settings=new VarList("Online Color Calib");

	_settings->addChild(_pub=new VarTrigger("Update","Update"));
	connect(_pub,SIGNAL(signalTriggered()),this,SLOT(slotUpdateTriggered()));
	_settings->addChild(_v_enable=new VarBool("enabled", false));

	colorMap.push_back(CH_ORANGE); // orange
	colorMap.push_back(CH_YELLOW); // yellow
	colorMap.push_back(CH_BLUE); // blue
	colorMap.push_back(CH_PINK); // pink
	colorMap.push_back(CH_GREEN); // green
	outDim = colorMap.size();

	maxDist.push_back(-1);
	maxDist.push_back(50);
	maxDist.push_back(50);
	maxDist.push_back(90);
	maxDist.push_back(90);

	minDist.push_back(-1);
	minDist.push_back(0);
	minDist.push_back(0);
	minDist.push_back(20);
	minDist.push_back(20);

	maxArea.push_back(-1);
	maxArea.push_back(160);
	maxArea.push_back(160);
	maxArea.push_back(144);
	maxArea.push_back(144);

	radius.push_back(-1);
	radius.push_back(5);
	radius.push_back(5);
	radius.push_back(4);
	radius.push_back(4);

	model = new LWPR_Object(3,outDim);
	doubleVec norm(3, 255);
	model->normIn(norm);
	model->setInitD(100);

	lastUpdate = std::chrono::system_clock::now();
}

PluginOnlineColorCalib::~PluginOnlineColorCalib() {
	// TODO Auto-generated destructor stub
}


void PluginOnlineColorCalib::slotUpdateTriggered() {
  lock();
  CopytoLUT(_lut);
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
	doubleVec v_out(outDim);
	v_in[0] = color.y;
	v_in[1] = color.u;
	v_in[2] = color.v;
	for(int i=0;i<v_out.size();i++)
		v_out[i] = (clazz==i);

	doubleVec out = model->update(v_in, v_out);
//	double diff = 0;
//	for(int i=0;i<out.size();i++)
//	{
//		diff += fabsf(v_out[i] - out[i]);
//	}
//	if(clazz == 0 && diff > 0.1)
//		std::cout << model->nData() << " " << diff << " " << v_out[0] << "/" << v_out[1] << " " << out[0] << "/" << out[1] <<
//			" " << v_in[0] << "/" << v_in[1] << "/" << v_in[2] << std::endl;
}

void PluginOnlineColorCalib::processRegion(FrameData* frame, CMVision::Region* region, int clazz, float radius)
{
	pixelloc loc;
	float cx = region->cen_x;
	float cy = region->cen_y;
	float xRadius = radius;
	float yRadius = radius;
	float xStep = 1;
	float yStep = 1;
	LocStamped locStamped;
	locStamped.clazz = clazz;

	for(float x = cx-xRadius; x<=cx+xRadius; x+=xStep)
	{
		for(float y = cy-yRadius; y<=cy+yRadius; y+=yStep)
		{
			locStamped.time = frame->time;
			locStamped.loc.x = x;
			locStamped.loc.y = y;
			locs.push_back(locStamped);
		}
	}
}

static double distanceSq(const vector3d& p1, const vector3d& p2)
{
	double dx = p1.x-p2.x;
	double dy = p1.y-p2.y;
	return dx*dx+dy*dy;
}

double distanceToNearestBot(const std::vector<vector3d>& botPositions, const vector3d& loc)
{
	double minDistSq = 1e20;
	for(int i=0;i<botPositions.size();i++)
	{
		double distSq = distanceSq(botPositions[i], loc);
		if(distSq < minDistSq)
		{
			minDistSq = distSq;
		}
	}
	return sqrt(minDistSq);
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

		std::vector<vector3d> botPositions;
		for(int i=0;i<detection_frame->robots_blue_size();i++)
		{
			SSL_DetectionRobot robot = detection_frame->robots_blue(i);
			vector3d loc;
			loc.x = robot.x();
			loc.y = robot.y();
			botPositions.push_back(loc);
		}
		for(int i=0;i<detection_frame->robots_yellow_size();i++)
		{
			SSL_DetectionRobot robot = detection_frame->robots_yellow(i);
			vector3d loc;
			loc.x = robot.x();
			loc.y = robot.y();
			botPositions.push_back(loc);
		}

		std::vector<int> detections(colorMap.size(), 0);
		std::vector<int> updates(colorMap.size(), 0);

		int clazz = 0;
		CMVision::Region* region = colorlist->getRegionList(colorMap[clazz]).getInitialElement();
		while(region != 0)
		{
			vector2d pImg;
			pImg.x = region->cen_x;
			pImg.y = region->cen_y;
			vector3d pField;
			camera_parameters.image2field(pField, pImg, 21);
			if(
					region->height() > 3 &&
					region->width() > 3 &&
					region->area < 144 &&
					pField.x > -field.field_length->getDouble()/2 &&
					pField.x < field.field_length->getDouble()/2 &&
					pField.y > -field.field_width->getDouble()/2 &&
					pField.y < field.field_width->getDouble()/2)
			{
				double minDist = distanceToNearestBot(botPositions, pField);
				if(minDist > 90)
				{
					float rad = 4;
					img_debug->drawBox(pImg.x-rad, pImg.y-rad, rad*2, rad*2, region->color);
					int locBefore = locs.size();
					processRegion(frame, region, clazz, rad);
					detections[clazz] = detections[clazz] + 1;
					updates[clazz] = updates[clazz] + locs.size() - locBefore;
				}
			}
			region = region->next;
		}

		for(clazz=1;clazz<colorMap.size();clazz++)
		{
			CMVision::Region* region = colorlist->getRegionList(colorMap[clazz]).getInitialElement();
			while(region != 0)
			{
				vector2d pImg;
				pImg.x = region->cen_x;
				pImg.y = region->cen_y;
				vector3d pField;
				camera_parameters.image2field(pField, pImg, 130);
				double dist = distanceToNearestBot(botPositions, pField);
				if(
					dist < maxDist[clazz] &&
					dist > minDist[clazz] &&
					region->area < maxArea[clazz] &&
					region->height() > 3 &&
					region->width() > 3)
				{
					img_debug->drawBox(pImg.x-radius[clazz], pImg.y-radius[clazz], radius[clazz]*2, radius[clazz]*2, region->color);
					int locBefore = locs.size();
					processRegion(frame, region, clazz, radius[clazz]);
					detections[clazz] = detections[clazz] + 1;
					updates[clazz] = updates[clazz] + locs.size() - locBefore;
				} else {
					processRegion(frame, region, -1, radius[clazz]);
				}
				region = region->next;
			}
		}

		// remove old locs
		while(!locs.empty() && frame->time - locs[0].time > 0.1)
		{
			locs.pop_front();
		}

		// update
		for(int i=0;i<locs.size();i++)
		{
			updateModel(frame, locs[i].loc, locs[i].clazz);
		}

		// add some random samples
		pixelloc loc;
		vector2d pImg;
		vector3d pField;
		raw8 color;
		int rndSamples = std::min(1000, (int)locs.size());
		int rs = rndSamples;
		for(int i=0; i<rs; i++)
		{
			int length = (int) (field.field_length->getInt()); // + 2*field.boundary_width->getInt());
			int width = (int) (field.field_width->getInt()); // + 2*field.boundary_width->getInt());
			pField.x = (rand() % length) - length/2;
			pField.y = (rand() % width) - width/2;
			camera_parameters.field2image(pField, pImg);
			if(pImg.x > 0 && pImg.x < frame->video.getWidth() &&
					pImg.y > 0 && pImg.y < frame->video.getHeight())
			{
				loc.x = pImg.x;
				loc.y = pImg.y;
				if(!isInAnyRegion(colorlist, loc, 200, 10))
				{
					img_debug->drawBox(pImg.x, pImg.y, 1, 1, 1);
					updateModel(frame, loc, -1);
					rndSamples--;
				}
			}
		}

		std::cout << "det: ";
		for(int i=0;i<detections.size();i++)
			std::cout << detections[i] << " ";

		std::cout << "upd: ";
		for(int i=0;i<updates.size();i++)
			std::cout << updates[i] << " ";

		std::cout << "rnd: " << (rs-rndSamples) << "/" << rs;
		std::cout << std::endl;
		CopytoLUT(&lut);
	}

	CMVisionThreshold::thresholdImageYUV422_UYVY(img_thresholded,&(frame->video),&lut);

	return ProcessingOk;
}

void PluginOnlineColorCalib::CopytoLUT(LUT3D *lut) {

    doubleVec input(3);
    doubleVec output(outDim);

    lut->lock();
	for (int y = 0; y <= 255; y+= (0x1 << _lut->X_SHIFT)) {
		for (int u = 0; u <= 255; u+= (0x1 << _lut->Y_SHIFT)) {
			for (int v = 0; v <= 255; v+= (0x1 << _lut->Z_SHIFT)) {
				input[0]= (double)y;
				input[1]= (double)u;
				input[2]= (double)v;
				output = model->predict(input, 0.001);
				int maxIdx = 0;
				double maxValue = 0;
				for(int i=0;i<outDim;i++)
				{
					if(output[i] > maxValue)
					{
						maxValue = output[i];
						maxIdx = i;
					}
				}
				if(maxValue > 0.5)
					lut->set(y,u,v,colorMap[maxIdx]);
				else
					lut->set(y,u,v,0);
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
