#ifndef AVG_COLOR_H_
#define AVG_COLOR_H_

#include <opencv2/core/core.hpp>

#include "threshold.hpp" // for component struct

using namespace cv;

double avg_color(Mat hsv, Component c);

#endif // AVG_COLOR_H_
