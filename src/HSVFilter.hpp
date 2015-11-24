#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class HSVFilter {
public:
    HSVFilter(cv::Mat &rgb);

    virtual ~HSVFilter() { }

    void setColorValues(std::string);

    void morphOps(cv::Mat &threshold);

    cv::Mat getFilteredImage();

private:
    cv::Mat rgb_;
    int H_MIN_;
    int H_MAX_;
    int S_MIN_;
    int S_MAX_;
    int V_MIN_;
    int V_MAX_;

};

HSVFilter::HSVFilter(cv::Mat &rgb)
        : rgb_(rgb),
          H_MIN_(0),
          H_MAX_(255),
          S_MIN_(0),
          S_MAX_(255),
          V_MIN_(0),
          V_MAX_(255) { }

void HSVFilter::setColorValues(std::string color) {
  if (color == "GREEN") {
    H_MIN_ = 57;
    H_MAX_ = 85;

    S_MIN_ = 50;
    S_MAX_ = 255;

    V_MIN_ = 0;
    V_MAX_ = 255;
  }
  if (color == "RED") {
    H_MIN = 0;
    H_MAX = 20;

    S_MIN = 210;
    S_MAX = 255;

    V_MIN = 0;
    V_MAX = 255;
  }
  if (color == "YELLOW") {
    H_MIN = 20;
    H_MAX = 35;

    S_MIN = 135;
    S_MAX = 255;

    V_MIN = 0;
    V_MAX = 255;
  }
}

void HSVFilter::morphOps(cv::Mat &thresh) {
  // create structuring element that will be used to "dilate" and "erode" image.
  // the element chosen here is a 3px by 3px rectangle
  cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  // dilate with larger element so make sure object is nicely visible
  cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8));

  cv::erode(thresh, thresh, erodeElement);
  cv::erode(thresh, thresh, erodeElement);

  cv::dilate(thresh, thresh, dilateElement);
  cv::dilate(thresh, thresh, dilateElement);
}

/*
 * Return a filtered image based on predefined HSV values as threshold.
 * The threshold values are provided by ImageFilter::setHsvValues(...)
 */
cv::Mat HSVFilter::getFilteredImage() {
  cv::Mat HSV, threshold;
  // Change the color format from BGR to HSV
  cv::cvtColor(rgb_, HSV, CV_BGR2HSV);

  // Filter HSV image between values and store filtered image to threshold matrix
  cv::inRange(HSV, cv::Scalar(H_MIN_, S_MIN_, V_MIN_), cv::Scalar(H_MAX_, S_MAX_, V_MAX_), threshold);

  return threshold;
}
