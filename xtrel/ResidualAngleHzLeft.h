#pragma once
#include <cmath>
#include "TerrainPoint.h"
struct ResidualAngleHzLeft
{
public:
	ResidualAngleHzLeft(TerrainPoint centerpoint_, TerrainPoint rightpoint_, double angle_rad_, double scale_):
		CenralPoint(centerpoint_), RightPoint(rightpoint_), angle_rad(angle_rad_), scale(scale_) {}

	template<typename T>
	bool operator() (const T* const coords, T* residual) const
	{
		residual[0] =	scale * T(angle_rad) -
						scale * (std::atan2(coords[1] - T(CenralPoint.Coords[1]), coords[0] - T(CenralPoint.Coords[0]))
							- std::atan2(T(RightPoint.Coords[1]) - T(CenralPoint.Coords[1]), T(RightPoint.Coords[0]) - T(CenralPoint.Coords[0])));
		
		//std::cout << "angle hz left residual = " << (1.0 / scale)*(residual[0] * 200) / 3.1415 << std::endl;
		
		return true;
	}

private:
	TerrainPoint CenralPoint;
	TerrainPoint RightPoint;
	double angle_rad{ 0.0 };
	double scale{1.0/0.000031415}; //=20cc 
};