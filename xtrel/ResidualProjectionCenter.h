#pragma once
#include <ceres/ceres.h>

class ResidualProjectionCenter : public ceres::SizedCostFunction<
	3, //number of residuals 
	3> //number of coordinates parameters[0]
{
public:
	ResidualProjectionCenter(double * projection_center_mes_, double mx_, double my_, double mz_)
		: mx(mx_), my(my_), mz(mz_),
		ProjectionCenterPointMes(projection_center_mes_) {}

	virtual ~ResidualProjectionCenter() {}
	virtual bool Evaluate(double const * const* parameters,
		double * residuals,
		double ** jacobians) const
	{
		double scale_x { 1.0 / mx};
		double scale_y { 1.0 / my};
		double scale_z { 1.0 / mz};


		residuals[0] = scale_x * (parameters[0][0] - ProjectionCenterPointMes[0]);
		residuals[1] = scale_y * (parameters[0][1] - ProjectionCenterPointMes[1]);
		residuals[2] = scale_z * (parameters[0][2] - ProjectionCenterPointMes[2]);

		if (jacobians != NULL && jacobians[0] != NULL)
		{
			jacobians[0][0] = scale_x * 1.0;
			jacobians[0][1] = 0.0;
			jacobians[0][2] = 0.0;

			jacobians[0][3 + 0] = 0.0;
			jacobians[0][3 + 1] = scale_y * 1.0;
			jacobians[0][3 + 2] = 0.0;

			jacobians[0][6 + 0] = 0.0;
			jacobians[0][6 + 1] = 0.0;
			jacobians[0][6 + 2] = scale_z * 1.0;
		}

		return true;
	}

	double *ProjectionCenterPointMes{ nullptr };
	double mx{1.0};
	double my{1.0};
	double mz{1.0};
};
