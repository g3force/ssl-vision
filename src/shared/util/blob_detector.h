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
#include <opencv2/core/mat.hpp>
#include "image.h"
#include "pixelloc.h"
#include "color_calibrator.h"


class Blob {
public:
    pixelloc center;
    int width;
    int height;
    std::vector<pixelloc> detectedPixels;
    std::vector<LocLabeled> classifiedLocations;

    inline int getHeightEven() const {
      if (height % 2 == 1) {
        return height + 1;
      }
      return height;
    }

    inline int getWidthEven() const {
      if (width % 2 == 1) {
        return width + 1;
      }
      return width;
    }

    inline void circularLocations(int imageWidth, int imageHeight, std::vector<pixelloc> &locations) const {
      const int maxY = getHeightEven() / 2;
      const int maxX = getWidthEven() / 2;

      for (int y = -maxY; y <= maxY; y += 1) {
        for (int x = -maxX; x <= maxX; x += 1) {
          int rx = center.x + x;
          int ry = center.y + y;
          if (rx >= 0 && ry >= 0 && rx < imageWidth && ry < imageHeight &&
              x * x * maxY * maxY
              + y * y * maxX * maxX
              <= maxY * maxY * maxX * maxX) {
            locations.push_back(pixelloc{rx, ry});
          }
        }
      }
    }
};

class BlobDetector {
public:
    BlobDetector() = default;

    ~BlobDetector() = default;

    bool detectBlob(const RawImage *img, Blob &blob, int targetClazz);

private:

    // number of clusters for kmeans
    int numClusters = 3;

    void colorAtLocations(const RawImage *img, const std::vector<pixelloc> &locations, cv::Mat &data) const;

    int clusterColors(const cv::Mat &allPixelsOfBlob, cv::Mat &out);

    bool similarClusterCentersPresent(cv::Mat &centers) const;

    int findBlobClass(int K, const int *locationsPerClusterCenter) const;
};

#endif /* SRC_SHARED_UTIL_BLOBDETECTOR_H_ */
