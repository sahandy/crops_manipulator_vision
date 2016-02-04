#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace crops_vision {
  enum Color {
    RED = 0,
    GREEN,
    BLUE,
    YELLOW,
    YELLOW_RED,
    MAGENTA_RED,
  };
}

int const MAX_HUE = 179;
int const MAX_SAT = 255;
int const MAX_VAL = 255;
int const MAX_KERNEL_SIZE = 21;

/**
 * Class that performs the HSV filter on a given RGB image.
 * The image is given as an input, along with user-defined HSV values for the
 * manual mode, or the COLOR NAME for the auto mode.
 */
class HSVFilter {
public:
    HSVFilter();
    HSVFilter(cv::Mat &rgb);

    virtual ~HSVFilter() { }

    void setImage(cv::Mat &rgb);
    /**
     * Sets the HSV values based on the given color name.
     */
    void setHsvValues(crops_vision::Color color);
    /**
     * Sets the HSV values explicitly
     */
    void setHsvValues(
      int hmin, int hmax, int smin, int smax, int vmin, int vmax);
    /**
     * Sets the attributes for the process of Dilation and Erosion
     */
    // Kernel Size
    void setMorphKernels(int er_sz, int dl_sz);
    // Number of iterations
    void setMorphIters(int er_it, int dl_it);
    /**
     * Performs dilation and erosion to grow/shrink black and white regions.
     * This helps removing noise from the thresholded image and restoring the
     * object structure.
     */
    void morphOps(cv::Mat &threshold);
    // Accessors
    /*
     * Return a filtered image based on predefined HSV values as threshold.
     * The threshold values are provided by ImageFilter::setHsvValues(...)
     */
    cv::Mat getFiltered();
    cv::Mat getFilteredImage(crops_vision::Color color);

private:
    cv::Mat rgb_;
    int H_MIN_, H_MAX_;
    int S_MIN_, S_MAX_;
    int V_MIN_, V_MAX_;
    int erosion_size_, dilation_size_;
    int num_iter_erode_, num_iter_dilate_;

};

HSVFilter::HSVFilter()
        : H_MIN_(0),
          H_MAX_(MAX_HUE),
          S_MIN_(0),
          S_MAX_(MAX_SAT),
          V_MIN_(0),
          V_MAX_(MAX_VAL),
          erosion_size_(1),
          dilation_size_(3),
          num_iter_erode_(1),
          num_iter_dilate_(1) {}

HSVFilter::HSVFilter(cv::Mat &rgb)
        : rgb_(rgb),
          H_MIN_(0),
          H_MAX_(MAX_HUE),
          S_MIN_(0),
          S_MAX_(MAX_SAT),
          V_MIN_(0),
          V_MAX_(MAX_VAL),
          erosion_size_(1),
          dilation_size_(3),
          num_iter_erode_(1),
          num_iter_dilate_(1) { }

void HSVFilter::setImage(cv::Mat &rgb) {
  rgb_ = rgb;
}


void HSVFilter::setHsvValues(crops_vision::Color color) {
  switch (color) {
    case crops_vision::YELLOW_RED :
      H_MIN_ = 0;
      H_MAX_ = 10;

      S_MIN_ = 140;
      S_MAX_ = 255;

      V_MIN_ = 70;
      V_MAX_ = 255;
      break;
    case crops_vision::MAGENTA_RED :
      H_MIN_ = 165;
      H_MAX_ = 179;

      S_MIN_ = 140;
      S_MAX_ = 255;

      V_MIN_ = 70;
      V_MAX_ = 255;
      break;
    case crops_vision::YELLOW :
      H_MIN_ = 12;
      H_MAX_ = 22;

      S_MIN_ = 130;
      S_MAX_ = 255;

      V_MIN_ = 140;
      V_MAX_ = 255;
      break;
    case crops_vision::GREEN :
      H_MIN_ = 40;
      H_MAX_ = 70;

      S_MIN_ = 40;
      S_MAX_ = 255;

      V_MIN_ = 10;
      V_MAX_ = 255;
      break;
  }
}

void HSVFilter::setHsvValues(
  int hmin, int hmax, int smin, int smax, int vmin, int vmax) {
    H_MIN_ = hmin; H_MAX_ = hmax;
    S_MIN_ = smin; S_MAX_ = smax;
    V_MIN_ = vmin; V_MAX_ = vmax;
}

void HSVFilter::setMorphKernels(int er_sz, int dl_sz) {
  erosion_size_ = er_sz;
  dilation_size_ = dl_sz;
}

void HSVFilter::setMorphIters(int er_it, int dl_it) {
  num_iter_erode_ = er_it;
  num_iter_dilate_ = dl_it;
}

void HSVFilter::morphOps(cv::Mat &thresh) {
  // create structuring elements that will be used to "dilate" and "erode" image
  cv::Mat erodeElement = cv::getStructuringElement(
    cv::MORPH_RECT,
    cv::Size(2*erosion_size_+1, 2*erosion_size_+1));
  // dilate with larger element so make sure object is nicely visible
  cv::Mat dilateElement = cv::getStructuringElement(
    cv::MORPH_RECT,
    cv::Size(2*dilation_size_+1, 2*dilation_size_+1));

  // first: erode and grow the black region
  cv::erode(thresh,
            thresh,
            erodeElement,
            cv::Point(-1, -1),
            num_iter_erode_);
  // second: dilate and grow white regions
  cv::dilate(thresh,
             thresh,
             dilateElement,
             cv::Point(-1,-1),
             num_iter_dilate_);
}

cv::Mat HSVFilter::getFiltered() {
  cv::Mat HSV, threshold;
  // Change the color format from BGR to HSV
  cv::cvtColor(rgb_, HSV, CV_BGR2HSV);
  // Filter HSV image between values and store filtered image to threshold matrix
  cv::inRange(HSV, cv::Scalar(H_MIN_, S_MIN_, V_MIN_), cv::Scalar(H_MAX_, S_MAX_, V_MAX_), threshold);

  return threshold;
}

cv::Mat HSVFilter::getFilteredImage(crops_vision::Color color) {
  // TODO (interactive mode:for intervals higher than 180 degrees) check if
  // H_MIN > H_MAX, then create two intervals

  // HSV is a cylindrical coordinate system. That's why red has different cut-offs.
  // the object 'extra' is for when we want to threshold the RED color.
  cv::Mat threshold, extra;
  switch (color) {
    case crops_vision::RED :
      setHsvValues(crops_vision::YELLOW_RED);
      threshold = getFiltered();
      setHsvValues(crops_vision::MAGENTA_RED);
      extra = getFiltered();
      return threshold + extra;
      break;
    case crops_vision::YELLOW :
      setHsvValues(color);
      return getFiltered();
      break;
    case crops_vision::GREEN :
      setHsvValues(color);
      return getFiltered();
      break;
  }
}
