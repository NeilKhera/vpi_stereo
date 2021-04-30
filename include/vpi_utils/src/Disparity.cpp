#include <opencv2/imgproc/imgproc.hpp>
#include <vpi/OpenCVInterop.hpp>

#include <vpi/Image.h>
#include <vpi/Status.h>
#include <vpi/Stream.h>
#include <vpi/algo/StereoDisparity.h>

#include <cstring> // for memset
#include <iostream>
#include <sstream>

#include "Disparity.h"

#define CHECK_STATUS(STMT)                                  \
  do {                                                      \
    VPIStatus status = (STMT);                              \
    if (status != VPI_SUCCESS) {                            \
      char buffer[VPI_MAX_STATUS_MESSAGE_LENGTH];           \
      vpiGetLastStatusMessage(buffer, sizeof(buffer));      \
      std::ostringstream ss;                                \
      ss << vpiStatusGetName(status) << ": " << buffer;     \
      throw std::runtime_error(ss.str());                   \
    }                                                       \
  } while (0);

namespace Disparity {

cv::Mat compute_disparity(const cv::Mat& image_left, const cv::Mat& image_right) {
  cv::Mat cvImageLeft = image_left;
  cv::Mat cvImageRight = image_right;
  cv::Mat cvOut;

  VPIImage left      = NULL;
  VPIImage right     = NULL;
  VPIImage disparity = NULL;
  VPIStream stream   = NULL;
  VPIPayload stereo  = NULL;

  // Currently we only accept unsigned 16bpp inputs.
  cvImageLeft.convertTo(cvImageLeft, CV_16UC1);
  cvImageRight.convertTo(cvImageRight, CV_16UC1);

  // Now parse the backend
  VPIBackend backendType = VPI_BACKEND_CUDA;

  // Create the stream for the given backend.
  CHECK_STATUS(vpiStreamCreate(backendType, &stream));

  // We now wrap the loaded images into a VPIImage object to be used by VPI.
  // VPI won't make a copy of it, so the original image must be in scope at all times.
  CHECK_STATUS(vpiImageCreateOpenCVMatWrapper(cvImageLeft, 0, &left));
  CHECK_STATUS(vpiImageCreateOpenCVMatWrapper(cvImageRight, 0, &right));

  // Create the image where the disparity map will be stored.
  CHECK_STATUS(vpiImageCreate(cvImageLeft.cols, cvImageLeft.rows, VPI_IMAGE_FORMAT_U16, 0, &disparity));

  // Create the payload for Harris Corners Detector algorithm
  CHECK_STATUS(vpiCreateStereoDisparityEstimator(backendType, cvImageLeft.cols, cvImageLeft.rows,
                                                 VPI_IMAGE_FORMAT_U16, NULL, &stereo));

  VPIStereoDisparityEstimatorParams params;
  params.windowSize   = 5;
  params.maxDisparity = 64;

  // Submit it with the input and output images
  CHECK_STATUS(vpiSubmitStereoDisparityEstimator(stream, backendType, stereo, left, right, disparity, NULL, &params));

  // Wait until the algorithm finishes processing
  CHECK_STATUS(vpiStreamSync(stream));

  // Now let's retrieve the output
  {
    // Lock output to retrieve its data on cpu memory
    VPIImageData data;
    CHECK_STATUS(vpiImageLock(disparity, VPI_LOCK_READ, &data));

    // Make an OpenCV matrix out of this image
    cvOut = cv::Mat(data.planes[0].height, data.planes[0].width, CV_16UC1, data.planes[0].data,
                    data.planes[0].pitchBytes);

    // Scale result and write it to disk
    double min, max;
    minMaxLoc(cvOut, &min, &max);
    cvOut.convertTo(cvOut, CV_8UC1, 255.0 / (max - min), -min);

    // Done handling output, don't forget to unlock it.
    CHECK_STATUS(vpiImageUnlock(disparity));
  }

  // Clean up

  // Make sure stream is synchronized before destroying the objects
  // that might still be in use.
  if (stream != NULL) {
    vpiStreamSync(stream);
  }

  vpiImageDestroy(left);
  vpiImageDestroy(right);
  vpiImageDestroy(disparity);
  vpiPayloadDestroy(stereo);
  vpiStreamDestroy(stream);

  return cvOut;
}

} // namespace Disparity

