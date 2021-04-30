#pragma once

#include <opencv2/opencv.hpp>

namespace Disparity {
  cv::Mat compute_disparity(const cv::Mat& cvImageLeft, const cv::Mat& cvImageRight);

} // namespace Disparity

