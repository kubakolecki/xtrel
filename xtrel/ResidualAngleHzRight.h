#pragma once
#include <cmath>
#include "TerrainPoint.h"
struct ResidualAngleHzRight
{
public:
	ResidualAngleHzRight(TerrainPoint centerpoint_, TerrainPoint leftpoint_, double angle_rad_, double scale_) :
		CenralPoint(centerpoint_), LeftPoint(leftpoint_), angle_rad(angle_rad_), scale(scale_) {}

	template<typename T>
	bool operator() (const T* const coords, T* residual) const
	{
		residual[0] = scale * T(angle_rad) -
			scale * (std::atan2(T(LeftPoint.Coords[1]) - T(CenralPoint.Coords[1]),T(LeftPoint.Coords[0]) - T(CenralPoint.Coords[0]))
				- std::atan2(coords[1] - T(CenralPoint.Coords[1]), coords[0] - T(CenralPoint.Coords[0])));
		
		//std::cout << "angle hz right residual = " << (1.0 / scale)*(residual[0] * 200) / 3.1415 << std::endl;
		
		return true;
	}

private:
	TerrainPoint CenralPoint;
	TerrainPoint LeftPoint;
	double angle_rad{ 0.0 };
	double scale{ 1.0/0.000031415 }; //=20cc 
};