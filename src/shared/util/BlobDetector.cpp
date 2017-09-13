/*
 * BlobDetector.cpp
 *
 *  Created on: Aug 22, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 *      Mark Geiger <markgeiger@posteo.de>
 */

#include "BlobDetector.h"

#include <opencv2/opencv.hpp>

BlobDetector::BlobDetector() {
	//
}

BlobDetector::~BlobDetector() {
	//
}

void BlobDetector::addBlob(
		const Blob& blob)
{
	mutex.lock();
	blobs.push_back(blob);
	mutex.unlock();
}

void BlobDetector::findRegion(
		const std::vector<std::vector<int>>& classes,
		const int x,
		const int y,
		const int posClass,
			  std::vector<pixelloc>& result)
{
	int height = classes.size();
	int width = classes[0].size();
//	if (x < 0 || y < 0 || x >= width || y >= height)
	if(abs(x) > width/2 || abs(y) > height/2)
		return;

	if(classes[y+height/2][x+width/2] != posClass) return;

	for(int i=0;i<result.size();i++)
		if(result[i].x == x && result[i].y == y) return;

	pixelloc loc = {x,y};
	result.push_back(loc);

	findRegion(classes, x+1, y, posClass, result);
	findRegion(classes, x-1, y, posClass, result);
	findRegion(classes, x, y+1, posClass, result);
	findRegion(classes, x, y-1, posClass, result);
}

bool BlobDetector::detectBlob(
		const RawImage * img,
			  Blob& blob,
			  Image<raw8> * img_debug)
{
	int height = blob.height;
	int width = blob.width;
	if(height % 2 == 1) height++;
	if(width % 2 == 1) width++;
	int maxY = height / 2;
	int maxX = width / 2;
	cv::Mat data(0, 3, CV_32F);
	for (int y = -maxY; y <= maxY; y += 1) {
		for (int x = -maxX; x <= maxX; x += 1) {
			int rx = blob.center.x + x;
			int ry = blob.center.y + y;
			yuv color;
			if(rx >= 0 && ry >= 0 && rx < img->getWidth() && ry < img->getHeight() &&
					x * x * maxY * maxY
					+ y * y * maxX * maxX
					<= maxY * maxY * maxX * maxX)
			{
				uyvy color2 = *((uyvy*) (img->getData()
						+ (sizeof(uyvy)
								* (((ry * (img->getWidth())) + rx) / 2))));
				color.u = color2.u;
				color.v = color2.v;
				if ((x % 2) == 0) {
					color.y = color2.y1;
				} else {
					color.y = color2.y2;
				}
				cv::Mat row(1,3,CV_32F);
				row.at<float>(0) = color.y;
				row.at<float>(1) = color.u;
				row.at<float>(2) = color.v;
				data.push_back(row);
			}
		}
	}


	int K = 3;
	if(data.rows < K)
		return false;
	cv::Mat out;
	bool repeat = true;
	while(repeat)
	{
		cv::TermCriteria tc;
		int attempts = 10;
		int flags = cv::KMEANS_PP_CENTERS;
		cv::Mat centers;
		cv::setNumThreads(0);
		cv::kmeans(data, K, out, tc, attempts, flags, centers);

		repeat = false;
		if(K > 2)
		{
			for(int i=0;i<centers.rows && !repeat;i++)
			{
				yuv col1 = {	(unsigned char) centers.at<float>(i,0),
								(unsigned char) centers.at<float>(i,1),
								(unsigned char) centers.at<float>(i,2) };
				for(int j=i+1;j<centers.rows;j++)
				{
					yuv col2 = {(unsigned char) centers.at<float>(j,0),
								(unsigned char) centers.at<float>(j,1),
								(unsigned char) centers.at<float>(j,2) };
					int du = col1.u - col2.u;
					int dv = col1.v - col2.v;
					int dist = abs(du) + abs(dv);
					if(dist < 30)
					{
						K--;
						repeat = true;
						break;
					}
				}
			}
		}
	}

	std::vector<std::vector<int>> classes(height+1);
	int i=0;
	for (int y = -maxY; y <= maxY; y += 1) {
		classes[y+maxY].resize(width+1);
		for (int x = -maxX; x <= maxX; x += 1) {
			int rx = blob.center.x + x;
			int ry = blob.center.y + y;
			yuv color;
			if(rx >= 0 && ry >= 0 && rx < img->getWidth() && ry < img->getHeight() &&
				x * x * maxY * maxY
				+ y * y * maxX * maxX
				<= maxY * maxY * maxX * maxX)
			{
				classes[y+maxY][x+maxX] = out.at<int>(i++);
			} else {
				classes[y+maxY][x+maxX] = -1;
			}
		}
	}

	int sum_mean[K];
	for(int i=0;i<K;i++) sum_mean[i] = 0;
	int s=1;
	for(int x=-s;x<=s;x++)
	{
		for(int y=-s;y<=s;y++)
		{
			sum_mean[classes[maxY+y][maxX+x]]++;
		}
	}

	int posClass = 0;
	int max_sum = 0;
	for(int k=0;k<K;k++)
	{
		if(sum_mean[k] > max_sum)
		{
			posClass = k;
			max_sum = sum_mean[k];
		}
	}

	int numPosClass = 0;
	for (int y = -maxY; y <= maxY; y += 1)
	{
		for (int x = -maxX; x <= maxX; x += 1)
		{
			if (x * x * maxY * maxY
					+ y * y * maxX * maxX
					<= maxY * maxY * maxX * maxX) {
				if(classes[y+maxY][x+maxX] == posClass)
				{
					numPosClass++;
					if(img_debug != 0) img_debug->setPixel(blob.center.x+x, blob.center.y+y, 2);
				} else {
					if(img_debug != 0) img_debug->setPixel(blob.center.x+x, blob.center.y+y, 1);
				}
			}
		}
	}

	int sumx=0;
	int sumy=0;
	blob.detectedPixels.clear();
	findRegion(classes, 0, 0, posClass, blob.detectedPixels);

	int lx=1e7, ly=1e7,hx=0,hy=0;
	for(int i=0;i<blob.detectedPixels.size();i++)
	{
		int x = blob.center.x + blob.detectedPixels[i].x;
		int y = blob.center.y + blob.detectedPixels[i].y;
		if(img_debug != 0) img_debug->setPixel(x,y, 4);
		sumx+=x;
		sumy+=y;
		lx = min(lx, x);
		ly = min(ly, y);
		hx = max(hx, x);
		hy = max(hy, y);
	}

	double circleArea = M_PI * width/2.0 * height/2.0;
	if(blob.detectedPixels.size() <= 0 || blob.detectedPixels.size() > circleArea * 0.9)
	{
		return false;
	}
	double relRegion = (double) blob.detectedPixels.size() / numPosClass;
	if(relRegion < 0.7)
		return false;

	int regionWidth = hx-lx;
	int regionHeight = hy-ly;
	if(regionWidth >= width-2 || regionHeight >= height-2)
		return false;

	double ratio = (double) min(regionWidth,regionHeight) / max(regionWidth,regionHeight);
	if(ratio < 0.5)
		return false;

	int mu_x = round((double) sumx/blob.detectedPixels.size());
	int mu_y = round((double) sumy/blob.detectedPixels.size());
	blob.center.x = mu_x;
	blob.center.y = mu_y;
	if(img_debug != 0) img_debug->setPixel(blob.center.x, blob.center.y, 3);

	return true;
}

void BlobDetector::update(
		const RawImage* img,
			  Image<raw8> * img_debug)
{
	mutex.lock();
	for(std::vector<Blob>::iterator it = blobs.begin(); it != blobs.end();)
	{
		bool ok = detectBlob(img, *it, img_debug);
		if(!ok)
		{
			it = blobs.erase(it);
		} else {
			bool del = false;
			for(std::vector<Blob>::iterator it2 = blobs.begin(); it2 != it; it2++)
			{
				if(it->center.x == it2->center.x &&
					it->center.y == it2->center.y)
				{
					it = blobs.erase(it);
					del = true;
					break;
				}
			}
			if(!del) it++;
		}
	}
	mutex.unlock();
}
