//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
  \file    blob_detector.cpp
  \brief   C++ Implementation: BlobDetector
  \author  Nicolai Ommer <nicolai.ommer@gmail.com>, (C) 2016
*/
//========================================================================

#include "blob_detector.h"
#include "color_calibrator.h"
#include <opencv2/opencv.hpp>

bool BlobDetector::detectBlob(const RawImage *img, Blob &blob, int targetClazz) {

  std::vector<pixelloc> locations;
  blob.circularLocations(img->getWidth(), img->getHeight(), locations);

  cv::Mat allPixelsOfBlob(0, 3, CV_32F);
  colorAtLocations(img, locations, allPixelsOfBlob);
  if (allPixelsOfBlob.rows < numClusters) {
    return false;
  }

  cv::Mat clusteredClasses;
  int K = clusterColors(allPixelsOfBlob, clusteredClasses);

  int locationsPerClusterCenter[K];
  memset(locationsPerClusterCenter, 0, sizeof(int) * K);
  for (int i = 0; i < locations.size(); i++) {
    auto loc = locations[i];
    if (abs(loc.x - blob.center.x) < 2 && abs(loc.y - blob.center.y) < 2) {
      locationsPerClusterCenter[clusteredClasses.at<int>(i)]++;
    }
  }

  int blobKMeansClass = findBlobClass(K, locationsPerClusterCenter);

  blob.detectedPixels.clear();
  for (int i = 0; i < locations.size(); i++) {
    auto &loc = locations[i];
    if (clusteredClasses.at<int>(i) == blobKMeansClass) {
      blob.detectedPixels.push_back(loc);
      blob.classifiedLocations.push_back(LocLabeled{loc, targetClazz});
    } else {
      blob.classifiedLocations.push_back(LocLabeled{loc, -1});
    }
  }

  int sumx = 0;
  int sumy = 0;
  int lx = static_cast<int>(1e7), ly = static_cast<int>(1e7), hx = 0, hy = 0;
  for (int j = 0; j < blob.detectedPixels.size(); j++) {
    int x = blob.center.x + blob.detectedPixels[j].x;
    int y = blob.center.y + blob.detectedPixels[j].y;
    sumx += x;
    sumy += y;
    lx = min(lx, x);
    ly = min(ly, y);
    hx = max(hx, x);
    hy = max(hy, y);
  }

  const int height = blob.getHeightEven();
  const int width = blob.getWidthEven();
  int regionWidth = hx - lx;
  int regionHeight = hy - ly;
  if (regionWidth >= width - 2 || regionHeight >= height - 2) {
    // blob covers full patch
    return false;
  }

  double ratio = (double) min(regionWidth, regionHeight) / max(regionWidth, regionHeight);
  if (ratio < 0.5) {
    // ration between width and height too unequal
    return false;
  }

  // update blob center
  auto mu_x = static_cast<int>(round((double) sumx / blob.detectedPixels.size()));
  auto mu_y = static_cast<int>(round((double) sumy / blob.detectedPixels.size()));
  blob.center.x = mu_x;
  blob.center.y = mu_y;

  return true;
}

void BlobDetector::colorAtLocations(const RawImage *img, const std::vector<pixelloc> &locations, cv::Mat &data) const {
  for (auto loc : locations) {
    yuv color = img->getYuv(loc.x, loc.y);
    cv::Mat row(1, 3, CV_32F);
    row.at<float>(0) = color.y;
    row.at<float>(1) = color.u;
    row.at<float>(2) = color.v;
    data.push_back(row);
  }
}

int BlobDetector::clusterColors(const cv::Mat &allPixelsOfBlob, cv::Mat &out) {
  int K = numClusters;
  for (;; K--) {
    cv::TermCriteria tc;
    int attempts = 10;
    int flags = cv::KMEANS_PP_CENTERS;
    cv::Mat centers;
    cv::setNumThreads(0);
    cv::kmeans(allPixelsOfBlob, K, out, tc, attempts, flags, centers);

    if (K <= 2 || !similarClusterCentersPresent(centers)) {
      return K;
    }
  }
}

bool BlobDetector::similarClusterCentersPresent(cv::Mat &centers) const {
  for (int i = 0; i < centers.rows; i++) {
    yuv col1 = {(unsigned char) centers.at<float>(i, 0),
                (unsigned char) centers.at<float>(i, 1),
                (unsigned char) centers.at<float>(i, 2)};
    for (int j = i + 1; j < centers.rows; j++) {
      yuv col2 = {(unsigned char) centers.at<float>(j, 0),
                  (unsigned char) centers.at<float>(j, 1),
                  (unsigned char) centers.at<float>(j, 2)};
      int du = col1.u - col2.u;
      int dv = col1.v - col2.v;
      int dist = abs(du) + abs(dv);
      if (dist < 30) {
        return true;
      }
    }
  }
  return false;
}

int BlobDetector::findBlobClass(int K, const int *locationsPerClusterCenter) const {
  int blobClazz = 0;
  int max_sum = 0;
  for (int k = 0; k < K; k++) {
    if (locationsPerClusterCenter[k] > max_sum) {
      blobClazz = k;
      max_sum = locationsPerClusterCenter[k];
    }
  }
  return blobClazz;
}
