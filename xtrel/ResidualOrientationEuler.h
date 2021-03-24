#pragma once
#include <ceres/ceres.h>

class ResidualOrientationEuler : public ceres::SizedCostFunction<
	3, //number of residuals 
	3> //number of coordinates parameters[0]
{
public:
	ResidualOrientationEuler(double * angles_mes_, double m_angle1_, double m_angle2_, double m_angle3_)
		: m_angle_1(m_angle1_), m_angle_2(m_angle2_), m_angle_3(m_angle3_),
		AnglesMes(angles_mes_) {}

	virtual ~ResidualOrientationEuler() {}
	virtual bool Evaluate(double const * const* parameters,
		double * residuals,
		double ** jacobians) const
	{
		double scale_x{ 1.0 / m_angle_1 };
		double scale_y{ 1.0 / m_angle_2 };
		double scale_z{ 1.0 / m_angle_3 };


		residuals[0] = scale_x * (parameters[0][0] - AnglesMes[0]);
		residuals[1] = scale_y * (parameters[0][1] - AnglesMes[1]);
		residuals[2] = scale_z * (parameters[0][2] - AnglesMes[2]);

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

	double *AnglesMes{ nullptr };
	double m_angle_1{ 1.0 };
	double m_angle_2{ 1.0 };
	double m_angle_3{ 1.0 };
};
