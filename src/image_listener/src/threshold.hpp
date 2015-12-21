#ifndef THRESHOLD_HPP_
#define THRESHOLD_HPP_

#include <inttypes.h>

#include <vector>
#include <tuple>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

#define COMPONENT_SEPARATION_CONST 7001

#define SQ(x) (x)*(x)

typedef struct component {
	Mat masked;
	Moments m;
	double c_x;
	double c_y;
	double major_axis;
	double minor_axis;
	double axis_angle;
	uint16_t component_value;
	bool upright;
} Component;

bool compare_component_size(const Component& lhs, const Component& rhs);
bool compare_component_position(const Component& lhs, const Component& rhs);

// get image

Mat get_image(char *image_file, int &width, int &height);

Mat convert_to_greyscale(Mat image);

// thresholding functions

std::vector<int> create_histogram(Mat greyscale);

std::vector<double> memoize_P(std::vector<int> histogram, int width, int height);

double variance(std::vector<double> P);

int automatic_threshold(Mat greyscale);

Mat threshold_image(Mat greyscale);


// connectivity functions

Mat connected_components(Mat binarized, std::vector<Component> &components);

Mat mask_by_component(Mat componentized, uint16_t value);
#endif // THRESHOLD_HPP_
