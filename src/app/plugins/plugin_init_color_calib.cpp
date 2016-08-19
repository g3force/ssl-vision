/*
 * plugin_init_color_calib.cpp
 *
 *  Created on: Aug 12, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "plugin_init_color_calib.h"

#include <opencv2/opencv.hpp>

PluginInitColorCalib::PluginInitColorCalib(FrameBuffer * _buffer,
		LUT3D * lut, const CameraParameters& camera_params,
		const RoboCupField& field) :
		VisionPlugin(_buffer),
		clazz2Channel(6),
		rgbColors(6),
		yuvColors(6),
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


	//#define CH_ORANGE 2
	//#define CH_YELLOW 3
	//#define CH_BLUE 4
	//#define CH_PINK 5
	//#define CH_GREEN 7

	clazz2Channel[0] = 4;
	clazz2Channel[1] = 7;
	clazz2Channel[2] = 2;
	clazz2Channel[3] = 5;
	clazz2Channel[4] = 3;
	clazz2Channel[5] = 0;

//	rgb black = {0,0,0};
//	rgb orange = {255,128,0};
//	rgb yellow = {255,255,0};
//	rgb blue = {0,0,255};
//	rgb magenta = {255,0,255};
//	rgb green = {0,255,0};
	rgb black = {0,0,0};
	rgb orange = {156,60,38};
	rgb yellow = {158,208,103};
//	rgb yellow = {205,193,50};
//	rgb yellow = {74,91,5};
	rgb blue = {25,63,253};
	rgb magenta = {255,0,221};
	rgb green = {74,255,78};

	rgbColors[0] = (blue);
	rgbColors[1] = (green);
	rgbColors[2] = (orange);
	rgbColors[3] = (magenta);
	rgbColors[4] = (yellow);
	rgbColors[5] = (black);
	for(int i=0;i<6;i++)
		yuvColors[i] = Conversions::rgb2yuv(rgbColors[i]);
}

PluginInitColorCalib::~PluginInitColorCalib() {
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
	int r = c1.r-c2.r;
	int g = c1.g-c2.g;
	int b = c1.b-c2.b;
	return abs(r)+abs(g)+abs(b);
}

static float yuvColorDist(yuv& c1, yuv& c2)
{
	int r = c1.u-c2.u;
	int g = c1.v-c2.v;
	int b = c1.y-c2.y;
	return abs(r)+abs(g); //+abs(b);
}


ProcessResult PluginInitColorCalib::process(FrameData * frame,
		RenderOptions * options) {
	(void) options;
	if (frame == 0)
		return ProcessingFailed;

	if(running)
	{
		for(int x=0;x<frame->video.getWidth();x++)
		{
			for(int y=0;y<frame->video.getHeight();y++)
			{
				vector2d pImg;
				pImg.x = x;
				pImg.y = y;
				vector3d pField;
				cam_params.image2field(pField, pImg, 0);

				if (pField.x
						> -(field.field_length->getDouble()) / 2
								- field.boundary_width->getDouble()
						&& pField.x
								< field.field_length->getDouble() / 2
										+ field.boundary_width->getDouble()
						&& pField.y
								> -field.field_width->getDouble() / 2
										- field.boundary_width->getDouble()
						&& pField.y
								< field.field_width->getDouble() / 2
										+ field.boundary_width->getDouble()) {
					// inside
				} else {
					continue;
				}

				yuv color;
				uyvy color2 = *((uyvy*) (frame->video.getData()
						+ (sizeof(uyvy)
								* (((y * (frame->video.getWidth())) + x) / 2))));
				color.u = color2.u;
				color.v = color2.v;
				if ((x % 2) == 0) {
					color.y = color2.y1;
				} else {
					color.y = color2.y2;
				}

				float minDiff = 1e10;
				int clazz = 0;
				for(int j=0;j<yuvColors.size();j++)
				{
					float diff = yuvColorDist(color, yuvColors[j]);
					if(diff < minDiff)
					{
						minDiff = diff;
						clazz = j;
					}
				}

				if(clazz != 5)
				{
////					int channel = clazz2Channel[clazz];
////					global_lut->set(color.y, color.u, color.v, channel);
					int val = local_lut->get(color.y, color.u, color.v) + 1;
					local_lut->set(color.y, color.u, color.v, val);
				}
			}
		}
		int n = frame->video.getHeight() * frame->video.getWidth();
		nSamples+=n;
		nFrames++;
		if(nFrames == 1)
		{
			classify();
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

	int K = 6;
	cv::Mat out;
	cv::TermCriteria tc;
	int attempts = 5;
	int flags = cv::KMEANS_PP_CENTERS;
	cv::kmeans(data, K, out, tc, attempts, flags);

	int voting[6][6];
	memset(voting, 0, sizeof(voting));
	for(int i=0;i<data.rows;i++)
	{
		yuv col = {(unsigned char)data.at<float>(i,0),
				(unsigned char)data.at<float>(i,1),
				(unsigned char)data.at<float>(i,2)};
		float minDiff = 1e10;
		int colClazz = 0;
		for(int j=0;j<yuvColors.size();j++)
		{
			float diff = yuvColorDist(col, yuvColors[j]);
			if(diff < minDiff)
			{
				minDiff = diff;
				colClazz = j;
			}
		}
		int kClazz = out.at<int>(i);
		voting[colClazz][kClazz]++;
	}

	std::vector<int> k2Col(6, -1);

	for(int colClazz=0;colClazz<6;colClazz++)
	{
		int maxVote = -1;
		int bestK = -1;
		for(int kClazz=0;kClazz<6;kClazz++)
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
		int channel = clazz2Channel[clazz];
		global_lut->set((int)data.at<float>(i,0),
				(int)data.at<float>(i,1),(int)data.at<float>(i,2),channel);
	}
	std::cout << "Found " << data.rows << "/" << n << " samples." << std::endl;
}

VarList * PluginInitColorCalib::getSettings() {
	return _settings;
}

string PluginInitColorCalib::getName() {
	return "InitColorCalib";
}
