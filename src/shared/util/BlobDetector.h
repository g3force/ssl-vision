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

	virtual void addBlob(
			const Blob& blob);

	virtual void findRegion(
			const std::vector<std::vector<int>>& classes,
			const int x,
			const int y,
			const int posClass,
				  std::vector<pixelloc>& result);

	virtual bool detectBlob(
			const RawImage * img,
				  Blob& blob,
				  Image<raw8> * img_debug);

	virtual void update(
			const RawImage* img,
				  Image<raw8> * img_debug = 0);

	std::vector<Blob> blobs;
private:
	std::mutex mutex;
};

#endif /* SRC_SHARED_UTIL_BLOBDETECTOR_H_ */
