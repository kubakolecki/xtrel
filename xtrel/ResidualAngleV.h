#pragma once
#include <cmath>
#include "TerrainPoint.h"
struct ResidualAngleV
{
public:
	ResidualAngleV(TerrainPoint CentralPoint_, double angle_rad_, double scale_) :
		CentralPoint(CentralPoint_), angle_rad(angle_rad_), scale(scale_) {}

	template<typename T>
	bool operator() (const T* const coords, T* residual) const
	{
		residual[0] =
			scale*T(angle_rad) -
			scale*std::acos((coords[2] - T(CentralPoint.Coords[2])) /
			std::sqrt(
			(coords[0] - T(CentralPoint.Coords[0])) * (coords[0] - T(CentralPoint.Coords[0])) +
			(coords[1] - T(CentralPoint.Coords[1])) * (coords[1] - T(CentralPoint.Coords[1])) + 
			(coords[2] - T(CentralPoint.Coords[2])) * (coords[2] - T(CentralPoint.Coords[2]))
			)
		    );

		//std::cout << "angle v residual = " << (1.0/scale)*(residual[0] * 200) / 3.1415 << std::endl;

		return true;
	}

private:
	TerrainPoint CentralPoint;
	double angle_rad{ 0.0 };
	double scale{ 1.0/0.000031415 }; //=20cc 
};