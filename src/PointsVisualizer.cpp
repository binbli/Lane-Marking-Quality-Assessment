#include"../include/PointsVisualizer.h"
#include"../include/Utilities.h"

namespace LD {

	void PointsVisualizer::operator()(Mat &_inputImg, Eigen::ArrayXXf &_coordinates) {
		Eigen::MatrixXf imgPoints;
		Mat reflectivity;

		Project(_coordinates, imgPoints);

		for (ulli i = 0; i < imgPoints.rows(); i++) {
			int x = imgPoints(i, 0), y = imgPoints(i, 1);
			if (isValid(y, x, _inputImg.rows, _inputImg.cols))
				circle(_inputImg, Point(x, y), 5, Scalar(0, 255, 0), -1);
		}

	}

}
