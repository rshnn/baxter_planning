#include <iostream>

#include <inttypes.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "threshold.hpp" // for component struct

using namespace cv;

double avg_color(Mat hsv, Component c) {
	int height = hsv.rows;
	int width = hsv.cols;

	double h_val = 0.0;
	double s_val = 0.0;
	double v_val = 0.0;

	int count = 0;
	Mat masked = c.masked;
	for (int y = 0; y< height; y++) {
		for (int x = 0; x < width; x++) {
			if (masked.at<uchar>(y,x) == UCHAR_MAX) {
				h_val+=hsv.at<Vec3b>(y,x)[0];
				s_val+=hsv.at<Vec3b>(y,x)[1];
				v_val+=hsv.at<Vec3b>(y,x)[2];
				count+=1;
			}
		}
	}
	return h_val/count;
}
