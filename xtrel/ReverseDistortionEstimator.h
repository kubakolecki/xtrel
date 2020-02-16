#pragma once
#include "Camera.h"

class ReverseDistortionEstimator
{
public:
	ReverseDistortionEstimator(const Camera& cam);
	~ReverseDistortionEstimator();
	std::array<double, 5> DistortionYUp{ 0.0, 0.0, 0.0, 0.0, 0.0 };
	std::array<double, 5> DistortionYDown{ 0.0, 0.0, 0.0, 0.0, 0.0 };

	double RMSE{ 0.0 };
};

