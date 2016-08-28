/*
 * plugin_init_color_calib.cpp
 *
 *  Created on: Aug 12, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "plugin_init_color_calib.h"

#include <opencv2/opencv.hpp>


#define CH_ORANGE 2
#define CH_YELLOW 3
#define CH_BLUE 4
#define CH_PINK 5
#define CH_GREEN 7

PluginInitColorCalib::PluginInitColorCalib(FrameBuffer * _buffer,
		LUT3D * lut, const CameraParameters& camera_params,
		const RoboCupField& field) :
		VisionPlugin(_buffer),
		cam_params(camera_params), field(field){

	_settings = new VarList("Init Color Calib");

	_settings->addChild(
			_update = new VarTrigger("Init", "Classify initial LUT"));
	connect(_update, SIGNAL(signalTriggered()), this,
			SLOT(slotUpdateTriggered()));

	global_lut = lut;
	local_lut = new YUVLUT(8,8,8);
	running = false;
	nFrames = 0;
	nSamples = 0;

	maxColorDist = 7000;

//	colors.push_back(ColorClazz(0,0,255, CH_BLUE));
//	colors.push_back(ColorClazz(255,255,0, CH_YELLOW));
//	colors.push_back(ColorClazz(0,255,0, CH_GREEN));
//	colors.push_back(ColorClazz(255,0,255, CH_PINK));
//	colors.push_back(ColorClazz(255,0,0, CH_ORANGE));
//	colors.push_back(ColorClazz(0,0,0, 9));
//	colors.push_back(ColorClazz(255,255,255, 8));

	colors.push_back(ColorClazz(0,100,255, CH_BLUE));
	colors.push_back(ColorClazz(255,255,0, CH_YELLOW));
	colors.push_back(ColorClazz(100,255,0, CH_GREEN));
	colors.push_back(ColorClazz(255,0,180, CH_PINK));
	colors.push_back(ColorClazz(255,0,0, CH_ORANGE));
//	colors.push_back(ColorClazz(0,0,0, 9));
//	colors.push_back(ColorClazz(255,255,255, 8));
//	colors.push_back(ColorClazz(45,45,0, 1));
//	colors.push_back(ColorClazz(0,255,255, 6));

//	colors.push_back(ColorClazz(25,63,253, CH_BLUE));
//	colors.push_back(ColorClazz(8,44,123, CH_BLUE));
//	colors.push_back(ColorClazz(74,255,78, CH_GREEN));
//	colors.push_back(ColorClazz(33,106,19, CH_GREEN));
//	colors.push_back(ColorClazz(15,114,6, CH_GREEN));
//	colors.push_back(ColorClazz(156,60,38, CH_ORANGE));
//	colors.push_back(ColorClazz(252,19,5, CH_ORANGE));
//	colors.push_back(ColorClazz(228,8,26, CH_ORANGE));
//	colors.push_back(ColorClazz(255,0,221, CH_PINK));
//	colors.push_back(ColorClazz(121,0,70, CH_PINK));
//	colors.push_back(ColorClazz(158,208,103, CH_YELLOW));
//	colors.push_back(ColorClazz(205,193,50, CH_YELLOW));
//	colors.push_back(ColorClazz(74,91,5, CH_YELLOW));
//	colors.push_back(ColorClazz(71,85,18, CH_YELLOW));
//	colors.push_back(ColorClazz(154,165,64, CH_YELLOW));
//	colors.push_back(ColorClazz(0,0,0, 0));
//	colors.push_back(ColorClazz(255,255,255, 0));
//	colors.push_back(ColorClazz(51,53,70, 0));
}

PluginInitColorCalib::~PluginInitColorCalib() {
}

ColorClazz::ColorClazz(unsigned char r, unsigned char g, unsigned char b, int clazz)
	: color_rgb(r,g,b)
{
	this->clazz = clazz;
	color_yuv = Conversions::rgb2yuv(color_rgb);
}

void PluginInitColorCalib::slotUpdateTriggered() {
	local_lut->reset();
	global_lut->reset();
	nFrames = 0;
	nSamples = 0;
	running = true;
}

static float rgbColorDist(rgb& c1, rgb& c2)
{
	float r = c1.r-c2.r;
	float g = c1.g-c2.g;
	float b = c1.b-c2.b;
//	return abs(r)+abs(g)+abs(b);
	return r*r+g*g+b*b;
}

static float yuvColorDist(yuv& c1, yuv& c2)
{
	float u = c1.u-c2.u;
	float v = c1.v-c2.v;
	float y = c1.y-c2.y;
	return u*u+v*v; //+y*y;
}

static yuv getColorFromImage(RawImage* img, int x, int y)
{
	yuv color;
	uyvy color2 = *((uyvy*) (img->getData()
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

void PluginInitColorCalib::addBlobs(SSL_DetectionFrame* detection_frame, RawImage* img)
{
	std::vector<SSL_DetectionRobot> robots;
	robots.insert(robots.end(), detection_frame->robots_blue().begin(),
			detection_frame->robots_blue().end());
	robots.insert(robots.end(), detection_frame->robots_yellow().begin(),
			detection_frame->robots_yellow().end());

	int nBlueRobots = detection_frame->robots_blue_size();

	for (int r=0;r<robots.size();r++) {
		SSL_DetectionRobot* robot = &robots[r];
		vector3d f_marker[5];
		f_marker[0].x = robot->x();
		f_marker[0].y = robot->y();

		float angle[] = {60,-60,145,-145};
		for(int i=0;i<4;i++)
		{
			float a = robot->orientation() + angle[i] / 180.0 * M_PI;
			f_marker[i+1].x = robot->x() + 70*cos(a);
			f_marker[i+1].y = robot->y() + 70*sin(a);
		}

		for(int i=0;i<5;i++)
			f_marker[i].z = robot->height();

		for(int i=0;i<5;i++)
		{
			vector2d img_center;
			cam_params.field2image(f_marker[i], img_center);
			Blob blob;
			blob.center.x = img_center.x;
			blob.center.y = img_center.y;
			blob.height = 16;
			blob.width = 16;
			if(i==0)
			{
				if(r < nBlueRobots)
					blob.channel = 4;
				else
					blob.channel = 3;
			} else {
				yuv img_color = getColorFromImage(img, blob.center.x, blob.center.y);
				yuv pink = {92, 154, 244};
				double pinkDist = yuvColorDist(img_color, pink);
				yuv green = {149, 43, 21};
				double greenDist = yuvColorDist(img_color, green);
				if(pinkDist < greenDist)
					blob.channel = 7; // pink
				else
					blob.channel = 5; // green
			}
			blobDetector.addBlob(blob);
		}
	}

//	for(int i=0;i<detection_frame->balls_size();i++)
//	{
//		vector3d f_pos;
//		f_pos.x = detection_frame->balls(i).x();
//		f_pos.y = detection_frame->balls(i).y();
//		f_pos.z = 42;
//		vector2d img_center;
//		cam_params.field2image(f_pos, img_center);
//		Blob blob;
//		blob.center.x = img_center.x;
//		blob.center.y = img_center.y;
//		blob.height = 16;
//		blob.width = 16;
//		blob.channel = 2;
//		blobDetector.addBlob(blob);
//	}
}

ProcessResult PluginInitColorCalib::process(FrameData * frame,
		RenderOptions * options) {
	(void) options;
	if (frame == 0)
		return ProcessingFailed;

	Image<raw8> * img_debug;
	if ((img_debug = (Image<raw8> *) frame->map.get(
			"cmv_online_color_calib")) == 0) {
		img_debug = (Image<raw8> *) frame->map.insert(
				"cmv_online_color_calib", new Image<raw8>());
	}
	img_debug->allocate(frame->video.getWidth(), frame->video.getHeight());
	img_debug->fillColor(0);

	SSL_DetectionFrame * detection_frame =
				(SSL_DetectionFrame *) frame->map.get("ssl_detection_frame");
	if (detection_frame == 0) {
		printf("no detection frame\n");
		return ProcessingFailed;
	}

//	addBlobs(detection_frame, &frame->video);
//	blobDetector.update(&frame->video, img_debug);
//
//	for(int i=0;i<blobDetector.blobs.size();i++)
//	{
//		for(int j=0;j<blobDetector.blobs[i]->detectedPixels.size();j++)
//		{
//			pixelloc loc = blobDetector.blobs[i]->detectedPixels[j];
//			loc.x += blobDetector.blobs[i]->center.x;
//			loc.y += blobDetector.blobs[i]->center.y;
//			yuv color = getColorFromImage(&frame->video, loc.x, loc.y);
//			global_lut->set(color.y, color.u, color.v, blobDetector.blobs[i]->channel);
//		}
//	}

	if(running)
	{
		int nConflicts = 0;
		for(int x=0;x<frame->video.getWidth();x++)
		{
			for(int y=0;y<frame->video.getHeight();y++)
			{
				vector2d pImg;
				pImg.x = x;
				pImg.y = y;
				vector3d pField;
				cam_params.image2field(pField, pImg, 0);

				double boundary = 100; // field.boundary_width->getDouble();
				if (pField.x
						> -(field.field_length->getDouble()) / 2
								- boundary
						&& pField.x
								< field.field_length->getDouble() / 2
										+ boundary
						&& pField.y
								> -field.field_width->getDouble() / 2
										- boundary
						&& pField.y
								< field.field_width->getDouble() / 2
										+ boundary) {
					// inside
				} else {
					continue;
				}

				yuv color = getColorFromImage(&frame->video, x, y);

				float minDiff = 1e10;
				int clazz = 0;
				for(int j=0;j<colors.size();j++)
				{
//					rgb color_rgb = Conversions::yuv2rgb(color);
//					float diff = rgbColorDist(color_rgb, colors[j].color_rgb);
					float diff = yuvColorDist(color, colors[j].color_yuv);
					if(diff < minDiff)
					{
						minDiff = diff;
						clazz = colors[j].clazz;
					}
				}

				if(minDiff < maxColorDist)
				{
					int curClazz = global_lut->get(color.y, color.u, color.v);
					if(curClazz != 0 && curClazz != clazz)
					{
						nConflicts++;
//						std::cout << "Conflict: " << curClazz << ", should be " << clazz << std::endl;
					}
					global_lut->set(color.y, color.u, color.v, clazz);
					local_lut->set(color.y, color.u, color.v, clazz);
//				if(clazz != 0)
//				{
//					int val = local_lut->get(color.y, color.u, color.v) + 1;
//					local_lut->set(color.y, color.u, color.v, val);
//				}
				}
			}
		}
		std::cout << "Num conflicts: " << nConflicts << std::endl;
		int n = frame->video.getHeight() * frame->video.getWidth();
		nSamples+=n;
		nFrames++;
		if(nFrames == 30)
		{
//			classify();
			running = false;
		}
	}

	return ProcessingOk;
}

void PluginInitColorCalib::classify()
{
	cv::Mat data(0, 3, CV_32F);
	int n = 0;
//	for (int y = 0; y <= 255; y += 1) {
//		for (int u = 0; u <= 255; u += 1) {
//			for (int v = 0; v <= 255; v += 1) {
	for (int y = 0; y <= 255; y += (0x1 << local_lut->X_SHIFT)) {
			for (int u = 0; u <= 255; u += (0x1 << local_lut->Y_SHIFT)) {
				for (int v = 0; v <= 255; v += (0x1 << local_lut->Z_SHIFT)) {
				int numSamples = local_lut->get(y, u, v);
				if(numSamples > 0)
				{
						cv::Mat row(1,3,CV_32F);
						row.at<float>(0) = y;
						row.at<float>(1) = u;
						row.at<float>(2) = v;
						data.push_back(row);
				}
				n++;
			}
		}
	}

	int K = colors.size();
	cv::Mat out;
	cv::TermCriteria tc;
	int attempts = 5;
	int flags = cv::KMEANS_PP_CENTERS;
	cv::kmeans(data, K, out, tc, attempts, flags);

	int voting[K][K];
	memset(voting, 0, sizeof(voting));
	for(int i=0;i<data.rows;i++)
	{
		yuv col = {(unsigned char)data.at<float>(i,0),
				(unsigned char)data.at<float>(i,1),
				(unsigned char)data.at<float>(i,2)};
		float minDiff = 1e10;
		int colClazz = 0;
		for(int j=0;j<colors.size();j++)
		{
			float diff = yuvColorDist(col, colors[j].color_yuv);
			if(diff < minDiff)
			{
				minDiff = diff;
				colClazz = j;
			}
		}
		int kClazz = out.at<int>(i);
		voting[colClazz][kClazz]++;
	}

	std::vector<int> k2Col(K, -1);

	for(int colClazz=0;colClazz<K;colClazz++)
	{
		int maxVote = -1;
		int bestK = -1;
		for(int kClazz=0;kClazz<K;kClazz++)
		{
			if(k2Col[kClazz] != -1) continue;
			if(voting[colClazz][kClazz] > maxVote)
			{
				bestK = kClazz;
				maxVote = voting[colClazz][kClazz];
			}
		}
		k2Col[bestK] = colClazz;
	}

	for(int i=0;i<data.rows;i++)
	{
		int o = out.at<int>(i);
		int clazz = k2Col[o];
		int channel = colors[clazz].clazz;
		global_lut->set((int)data.at<float>(i,0),
				(int)data.at<float>(i,1),(int)data.at<float>(i,2),channel);
	}
	std::cout << "Found " << data.rows << "/" << n << " samples." << std::endl;
}

void PluginInitColorCalib::mousePressEvent ( QMouseEvent * event, pixelloc loc )
{
	if((event->buttons() & Qt::LeftButton)!=0)
	{
		Blob blob;
		blob.center = loc;
		blob.width = 16;
		blob.height = 16;
		blob.channel = 2;
		blobDetector.addBlob(blob);
	}
}

VarList * PluginInitColorCalib::getSettings() {
	return _settings;
}

string PluginInitColorCalib::getName() {
	return "InitColorCalib";
}
