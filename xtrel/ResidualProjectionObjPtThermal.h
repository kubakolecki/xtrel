#pragma once
#pragma once
#include <ceres/ceres.h>
#include "photogrammetry.h"

class ResidualProjectionObjPtThermal : public ceres::SizedCostFunction<
	2, //residuals					
	3, //image projection center coordinates	parameters[0]
	3, //angles									parameters[1]
	3, //object point coordinates				parameters[2]
	3, //camera internal orientation parameters	parameters[3]
	2, //radial distortion coefficients (k1+k2)	parameters[4]
	1, //radial distortion coefficient k3		parameters[5]
	2, //tangential distortion coefficients		parameters[6]
	2> //thermal coeffs							parameters[7]
{
public:
	ResidualProjectionObjPtThermal(double x_, double y_, double mp_, string parametrization_) :
		x(x_), y(y_), mp(mp_), Parametrization(parametrization_) {}
	virtual ~ResidualProjectionObjPtThermal() {}
	virtual bool Evaluate(double const * const* parameters,
		double * residuals,
		double ** jacobians) const
	{
		double scale = 1.0 / mp; //multiplying by scale is equivalent and faster to multiplying by diagonal weight matrix
		double R[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
		fT_angles2rot(parameters[1], R, Parametrization);

		double P[2]; //projected point

		//rewriting radial distortion coeffs to one array
		double radial_distorion[3];
		radial_distorion[0] = parameters[4][0];
		radial_distorion[1] = parameters[4][1];
		radial_distorion[2] = parameters[5][0];

		//calculation of residuals
		fT_projection_thermal(parameters[2], parameters[0], R, parameters[3], radial_distorion, parameters[6], parameters[7], P);
		residuals[0] = scale * (P[0] - x);
		residuals[1] = scale * (P[1] - y);

		if (jacobians != NULL)
		{

			//calculation of jacobian:	
			if (jacobians[0] != NULL) //diffs wrt image projection center
			{
				double dxo[3]; //partial derivatives with respect to projection centre
				double dyo[3]; //partial derivatives with respect to projection centre
				fT_projection_center_differentials_thermal(parameters[2], parameters[0], parameters[1], parameters[3], parameters[7], "al-ni-ka", dxo, dyo);
				jacobians[0][0] = scale * dxo[0];		//std::cout << jacobians[0][0]<<std::endl;		//dx/dX0 std::cout<<jacobians[0][0];
				jacobians[0][1] = scale * dxo[1];		//std::cout << jacobians[0][1]<<std::endl;		//dx/dY0 std::cout<<jacobians[0][1];
				jacobians[0][2] = scale * dxo[2];		//std::cout << jacobians[0][2]<<std::endl;		//dx/dZ0 std::cout<<jacobians[0][2];
				jacobians[0][3 + 0] = scale * dyo[0];	//std::cout << jacobians[0][3 + 0]<<std::endl;//dy/dX0 std::cout<<jacobians[0][3+0];
				jacobians[0][3 + 1] = scale * dyo[1];	//std::cout << jacobians[0][3 + 1]<<std::endl;//dy/dY0 std::cout<<jacobians[0][3+1];
				jacobians[0][3 + 2] = scale * dyo[2];	//std::cout << jacobians[0][3 + 2]<<std::endl;//dy/dZ0 std::cout<<jacobians[0][3+2];
			}

			if (jacobians[1] != NULL) //diffs wrt rotation
			{
				double dxr[3]; //partial derivatives with respect to rotation
				double dyr[3]; //partial derivatives with respect to rotation
				fT_rotation_differentials_thermal(parameters[2], parameters[0], parameters[1], parameters[3], parameters[7], "al-ni-ka", dxr, dyr);
				jacobians[1][0] = scale * dxr[0];		//std::cout << jacobians[1][0]<<std::endl;	//dx/da1
				jacobians[1][1] = scale * dxr[1];		//std::cout << jacobians[1][1]<<std::endl;	//dx/da2
				jacobians[1][2] = scale * dxr[2];		//std::cout << jacobians[1][2]<<std::endl;	//dx/da3
				jacobians[1][3 + 0] = scale * dyr[0];	//std::cout << jacobians[1][3 + 0]<<std::endl;//dy/da1
				jacobians[1][3 + 1] = scale * dyr[1];	//std::cout << jacobians[1][3 + 1]<<std::endl;//dy/da2
				jacobians[1][3 + 2] = scale * dyr[2];	//std::cout << jacobians[1][3 + 2]<<std::endl;//dy/da3
			}

			if (jacobians[2] != NULL) //diffs wrt object point coordinates
			{
				double dx[3]; //partial derivatives with respect to object point coordinates
				double dy[3]; //partial derivatives with respect to object point coordinates
				fT_object_point_differentials_thermal(parameters[2], parameters[0], parameters[1], parameters[3], parameters[7], "al-ni-ka", dx, dy);
				jacobians[2][0] = scale * dx[0];		//std::cout << jacobians[1][0]<<std::endl;	//dx/da1
				jacobians[2][1] = scale * dx[1];		//std::cout << jacobians[1][1]<<std::endl;	//dx/da2
				jacobians[2][2] = scale * dx[2];		//std::cout << jacobians[1][2]<<std::endl;	//dx/da3
				jacobians[2][3 + 0] = scale * dy[0];	//std::cout << jacobians[1][3 + 0]<<std::endl;//dy/da1
				jacobians[2][3 + 1] = scale * dy[1];	//std::cout << jacobians[1][3 + 1]<<std::endl;//dy/da2
				jacobians[2][3 + 2] = scale * dy[2];	//std::cout << jacobians[1][3 + 2]<<std::endl;//dy/da3
			}


			if (jacobians[3] != NULL) //diffs wrt internal orientation: [ck x0 y0]
			{
				double dxi[3]; //partial derivatives with respect to internal parameters
				double dyi[3]; //partial derivatives with respect to internal parameters
				fT_internal_differentials_thermal(parameters[2], parameters[0], parameters[1], parameters[7], "al-ni-ka", dxi, dyi);
				jacobians[3][0] = scale * dxi[0];		//std::cout << jacobians[2][0]<<std::endl;	///dx/dck
				jacobians[3][1] = scale * dxi[1];		//std::cout << jacobians[2][1]<<std::endl;	///dx/dx0
				jacobians[3][2] = scale * dxi[2];		//std::cout << jacobians[2][2]<<std::endl;	///dx/dy0
				jacobians[3][3 + 0] = scale * dyi[0];	//std::cout << jacobians[2][3 + 0]<<std::endl;//dy/dck
				jacobians[3][3 + 1] = scale * dyi[1];	//std::cout << jacobians[2][3 + 1]<<std::endl;//dy/dx0
				jacobians[3][3 + 2] = scale * dyi[2]; 	//std::cout << jacobians[2][3 + 2]<<std::endl;//dy/dy0
			}

			if (jacobians[4] != NULL) //diffs wrt radial distortion coeffs k1 and k2
			{
				double dxdr[3]; //partial derivatives with respect to distortion (3 radial coeffs.)
				double dydr[3]; //partial derivatives with respect to distortion (3 radial coeffs.)
				fT_distortion_differentials_r(parameters[3], x, y, dxdr, dydr);
				jacobians[4][0] = scale * dxdr[0];		//std::cout << jacobians[3][0] << std::endl;		//dx/dk1
				jacobians[4][1] = scale * dxdr[1];		//std::cout << jacobians[3][1] << std::endl;		//dx/dk2
				jacobians[4][2 + 0] = scale * dydr[0];	//std::cout << jacobians[3][3 + 0] << std::endl;//dy/dk1
				jacobians[4][2 + 1] = scale * dydr[1];	//std::cout << jacobians[3][3 + 1] << std::endl;//dy/dk2
			}

			if (jacobians[5] != NULL) //diffs wrt radial distoriton k3 coeff
			{
				double dxdr[3]; //partial derivatives with respect to distortion (3 radial coeffs.)
				double dydr[3]; //partial derivatives with respect to distortion (3 radial coeffs.)
				fT_distortion_differentials_r(parameters[3], x, y, dxdr, dydr);
				jacobians[5][0] = scale * dxdr[2];	//std::cout << jacobians[3][0] << std::endl;	//dx/dk3
				jacobians[5][1 + 0] = scale * dydr[2];	//std::cout << jacobians[3][3 + 0] << std::endl;//dy/dk3
			}

			if (jacobians[6] != NULL) //diffs wrt tangential distortion
			{
				double dxdt[2]; //partial derivatives with respect to distortion (2 tangential coeffs.)
				double dydt[2]; //partial derivatives with respect to distortion (2 tangential coeffs.)
				fT_distortion_differentials_t(parameters[3], x, y, dxdt, dydt);
				jacobians[6][0] = scale * dxdt[0];		//std::cout << jacobians[4][0]	<< std::endl;	//dx/dp1
				jacobians[6][1] = scale * dxdt[1];		//std::cout << jacobians[4][1]	<< std::endl;	//dx/dp2
				jacobians[6][2 + 0] = scale * dydt[0];	//std::cout << jacobians[4][2 + 0] << std::endl;	//dy/dp1
				jacobians[6][2 + 1] = scale * dydt[1];	//std::cout << jacobians[4][2 + 1] << std::endl;//dy/dp2
			}

			if (jacobians[7] != NULL) //diffs wrt themral coeffs
			{
				double dxdT[2]; //partial derivatives with respect to thermal coeffs (2 coeffs.)
				double dydT[2]; //partial derivatives with respect to thermal coeffs (2 coeffs.)
				fT_thermal_differentials(parameters[2], parameters[0], parameters[1], parameters[3], parameters[7], "al-ni-ka", dxdT, dydT);
				jacobians[7][0] = scale * dxdT[0];		//dx/dTx
				jacobians[7][1] = scale * dxdT[1];		//dx/dTz
				jacobians[7][2 + 0] = scale * dydT[0];	//dy/dTx
				jacobians[7][2 + 1] = scale * dydT[1];	//dy/dTz

			}
		}
		return true;
	}

	double x{ 0.0 };
	double y{ 0.0 };
	double mp{ 0.0005 }; //accuracy of image point measurement
	std::string Parametrization{ "al-ni-ka" };
};
