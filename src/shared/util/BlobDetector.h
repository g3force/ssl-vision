/*
 * BlobDetector.h
 *
 *  Created on: Aug 22, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef SRC_SHARED_UTIL_BLOBDETECTOR_H_
#define SRC_SHARED_UTIL_BLOBDETECTOR_H_

#include <vector>
#include <mutex>
#include "image.h"
#include "pixelloc.h"


class Blob {
public:
	pixelloc center;
	int width;
	int height;
	int channel;
	std::vector<pixelloc> detectedPixels;
};

class BlobDetector {
public:
	BlobDetector();
	virtual ~BlobDetector();
	virtual bool detectBlob(RawImage * img, Blob* blob, Image<raw8> * img_debug);
	virtual void findRegion(std::vector<std::vector<int>>& classes, int x, int y, int posClass, std::vector<pixelloc>& result);
	virtual void update(RawImage* img, Image<raw8> * img_debug = 0);
	virtual void addBlob(Blob& blob);

	std::vector<Blob*> blobs;
private:
	std::mutex mutex;
};

#endif /* SRC_SHARED_UTIL_BLOBDETECTOR_H_ */
