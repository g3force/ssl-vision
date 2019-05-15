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
    std::vector<pixelloc> detectedPixels;
};

class BlobDetector {
public:
    BlobDetector();

    virtual ~BlobDetector();

    void findRegion(
            const std::vector<std::vector<int>> &classes,
            int x,
            int y,
            int posClass,
            std::vector<pixelloc> &result);

    bool detectBlob(
            const RawImage *img,
            Blob &blob,
            Image<raw8> *img_debug);

    void update(const RawImage *img, Image<raw8> *img_debug = nullptr);

    std::vector<Blob> blobs;
private:
    std::mutex mutex;
};

#endif /* SRC_SHARED_UTIL_BLOBDETECTOR_H_ */
