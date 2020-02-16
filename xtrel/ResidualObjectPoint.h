#pragma once
#include <ceres/ceres.h>

class ResidualObjectPoint : public ceres::SizedCostFunction<
	3, //number of residuals 
	3> //number of coordinates parameters[0]
{
	public:
		ResidualObjectPoint(double * object_point_mes_, double mx_, double my_, double mz_) 
			:	mx(mx_), my(my_), mz(mz_),
				ObjectPointMes(object_point_mes_){}

		virtual ~ResidualObjectPoint() {}
		virtual bool Evaluate(double const * const* parameters,
			double * residuals,
			double ** jacobians) const
		{
			double scale_x = 1.0 / mx;
			double scale_y = 1.0 / my;
			double scale_z = 1.0 / mz;
			
			
			residuals[0] = scale_x * (parameters[0][0] - ObjectPointMes[0]);
			residuals[1] = scale_y * (parameters[0][1] - ObjectPointMes[1]);
			residuals[2] = scale_z * (parameters[0][2] - ObjectPointMes[2]);

			if (jacobians != NULL  && jacobians[0] != NULL)
			{
				jacobians[0][0]		= scale_x * 1.0;
				jacobians[0][1]		= 0.0;
				jacobians[0][2]		= 0.0;

				jacobians[0][3 + 0] = 0.0;
				jacobians[0][3 + 1] = scale_y * 1.0;
				jacobians[0][3 + 2] = 0.0;

				jacobians[0][6 + 0] = 0.0;
				jacobians[0][6 + 1] = 0.0;
				jacobians[0][6 + 2] = scale_z * 1.0;
			}

			return true;
		}

		double *ObjectPointMes = nullptr;
		double mx;
		double my;
		double mz;
};