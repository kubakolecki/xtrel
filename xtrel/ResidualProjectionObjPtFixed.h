#pragma once
#include <ceres/ceres.h>
#include "photogrammetry.h"

class ResidualProjectionObjPtFixed : public ceres::SizedCostFunction<
	2, //nubmer of residuals					
	3, //image projection center coordinates	parameters[0]
	3, //angles									parameters[1]
	3, //camera internal orientation parameters	parameters[2]
	2, //radial distortion coefficients (k1+k2)	parameters[3]
	1, //radial distortion coefficient k3		parameters[4]
	2> //tangential distortion coefficients		parameters[5]
{	
	public:
		ResidualProjectionObjPtFixed(double x_, double y_, double* object_point_, double mp_, std::string parametrization_):
			x(x_), y(y_), ObjectPoint(object_point_), mp(mp_), Parametrization(parametrization_) {}
		virtual ~ResidualProjectionObjPtFixed() {}
		virtual bool Evaluate(	double const * const* parameters,
								double * residuals,
								double ** jacobians) const
		{
			double scale = 1.0 / mp; //multiplying by scale is equivalent and faster to applying by diagonal weight matrix
			double R[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
			fT_angles2rot(parameters[1], R, Parametrization);
					
			double P[2]; //projected point
			
			//rewriting radial distortion coeffs to one array
			double radial_distorion[3];
			radial_distorion[0] = parameters[3][0];
			radial_distorion[1] = parameters[3][1];
			radial_distorion[2] = parameters[4][0];
			
			//calculation of residuals
			fT_projection(ObjectPoint, parameters[0], R, parameters[2], radial_distorion, parameters[5], P);
			residuals[0] = scale*(P[0] - x);
			residuals[1] = scale*(P[1] - y);
			
			if (jacobians != NULL)
			{
				
				//calculation of jacobian:	
				if (jacobians[0] != NULL)
				{
					double dxo[3]; //partial derivatives with respect to projection centre
					double dyo[3]; //partial derivatives with respect to projection centre
					fT_projection_center_differentials(ObjectPoint, parameters[0], parameters[1], parameters[2], Parametrization, dxo, dyo);
					jacobians[0][0]		= scale*dxo[0];		//std::cout << jacobians[0][0]<<std::endl;		//dx/dX0 std::cout<<jacobians[0][0];
					jacobians[0][1]		= scale*dxo[1];		//std::cout << jacobians[0][1]<<std::endl;		//dx/dY0 std::cout<<jacobians[0][1];
					jacobians[0][2]		= scale*dxo[2];		//std::cout << jacobians[0][2]<<std::endl;		//dx/dZ0 std::cout<<jacobians[0][2];
					jacobians[0][3 + 0] = scale*dyo[0];	//std::cout << jacobians[0][3 + 0]<<std::endl;//dy/dX0 std::cout<<jacobians[0][3+0];
					jacobians[0][3 + 1] = scale*dyo[1];	//std::cout << jacobians[0][3 + 1]<<std::endl;//dy/dY0 std::cout<<jacobians[0][3+1];
					jacobians[0][3 + 2] = scale*dyo[2];	//std::cout << jacobians[0][3 + 2]<<std::endl;//dy/dZ0 std::cout<<jacobians[0][3+2];
				}

				if (jacobians[1] != NULL)
				{
					double dxr[3]; //partial derivatives with respect to rotation
					double dyr[3]; //partial derivatives with respect to rotation
					fT_rotation_differentials(ObjectPoint, parameters[0], parameters[1], parameters[2], Parametrization, dxr, dyr);
					jacobians[1][0]		= scale*dxr[0];		//std::cout << jacobians[1][0]<<std::endl;	//dx/da1
					jacobians[1][1]		= scale*dxr[1];		//std::cout << jacobians[1][1]<<std::endl;	//dx/da2
					jacobians[1][2]		= scale*dxr[2];		//std::cout << jacobians[1][2]<<std::endl;	//dx/da3
					jacobians[1][3 + 0] = scale*dyr[0];	//std::cout << jacobians[1][3 + 0]<<std::endl;//dy/da1
					jacobians[1][3 + 1] = scale*dyr[1];	//std::cout << jacobians[1][3 + 1]<<std::endl;//dy/da2
					jacobians[1][3 + 2] = scale*dyr[2];	//std::cout << jacobians[1][3 + 2]<<std::endl;//dy/da3
				}

				if (jacobians[2] != NULL)
				{
					double dxi[3]; //partial derivatives with respect to internal parameters
					double dyi[3]; //partial derivatives with respect to internal parameters
					fT_internal_differentials(ObjectPoint, parameters[0], parameters[1], Parametrization, dxi, dyi);
					jacobians[2][0]		= scale*dxi[0];		//std::cout << jacobians[2][0]<<std::endl;	///dx/dck
					jacobians[2][1]		= scale*dxi[1];		//std::cout << jacobians[2][1]<<std::endl;	///dx/dx0
					jacobians[2][2]		= scale*dxi[2];		//std::cout << jacobians[2][2]<<std::endl;	///dx/dy0
					jacobians[2][3 + 0] = scale*dyi[0];	//std::cout << jacobians[2][3 + 0]<<std::endl;//dy/dck
					jacobians[2][3 + 1] = scale*dyi[1];	//std::cout << jacobians[2][3 + 1]<<std::endl;//dy/dx0
					jacobians[2][3 + 2] = scale*dyi[2]; 	//std::cout << jacobians[2][3 + 2]<<std::endl;//dy/dy0
				}

				if (jacobians[3] != NULL)
				{
					double dxdr[3]; //partial derivatives with respect to distortion (3 radial coeffs.)
					double dydr[3]; //partial derivatives with respect to distortion (3 radial coeffs.)
					fT_distortion_differentials_r(parameters[2], x, y, dxdr, dydr);
					jacobians[3][0]		= scale*dxdr[0];		//std::cout << jacobians[3][0] << std::endl;		//dx/dk1
					jacobians[3][1]		= scale*dxdr[1];		//std::cout << jacobians[3][1] << std::endl;		//dx/dk2
					jacobians[3][2 + 0] = scale*dydr[0];	//std::cout << jacobians[3][3 + 0] << std::endl;//dy/dk1
					jacobians[3][2 + 1] = scale*dydr[1];	//std::cout << jacobians[3][3 + 1] << std::endl;//dy/dk2
				}

				if (jacobians[4] != NULL)
				{
					double dxdr[3]; //partial derivatives with respect to distortion (3 radial coeffs.)
					double dydr[3]; //partial derivatives with respect to distortion (3 radial coeffs.)
					fT_distortion_differentials_r(parameters[2], x, y, dxdr, dydr);
					jacobians[4][0]		= scale * dxdr[2];	//std::cout << jacobians[3][0] << std::endl;	//dx/dk3
					jacobians[4][1 + 0] = scale * dydr[2];	//std::cout << jacobians[3][3 + 0] << std::endl;//dy/dk3
				}

				if (jacobians[5] != NULL)
				{
					double dxdt[2]; //partial derivatives with respect to distortion (2 tangential coeffs.)
					double dydt[2]; //partial derivatives with respect to distortion (2 tangential coeffs.)
					fT_distortion_differentials_t(parameters[2], x, y, dxdt, dydt);
					jacobians[5][0]		= scale*dxdt[0];		//std::cout << jacobians[4][0]	<< std::endl;	//dx/dp1
					jacobians[5][1]		= scale*dxdt[1];		//std::cout << jacobians[4][1]	<< std::endl;	//dx/dp2
					jacobians[5][2 + 0] = scale*dydt[0];	//std::cout << jacobians[4][2 + 0] << std::endl;	//dy/dp1
					jacobians[5][2 + 1] = scale*dydt[1];	//std::cout << jacobians[4][2 + 1] << std::endl;//dy/dp2
				}
			}
			return true;
		}
	
		double x;
		double y;
		double mp = 0.0005; //accuracy of image point measurement 
		double *ObjectPoint = nullptr; //pointer to 3-element array
		std::string Parametrization{ "al-ni-ka" };
};
