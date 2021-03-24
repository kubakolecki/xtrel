#pragma once
#include <vector>
#include <array>
#include <string>

#include "ImagePoint.h"
#include "TerrainPoint.h"
#include "ImageDataContainer.h"

class RayIntersectionCheck
{
	//this class calculate the vectors of the shortest distance
	//between object point and rays generated by imagePoints (distorted, original coordinates)
	//The external orientation and internal orientation has to be provided in ImageDataContainer
	//Output data are calculaed directly inside the constructor
public:
	RayIntersectionCheck(
		const TerrainPoint& objectPoint,
		const std::vector<ImagePoint>& imagePoints,
		const ImageDataContainer& imageData);
	~RayIntersectionCheck() = default;

	//output data, to be accessed right after object construction

	//cooridinates of all 3D point - ray vectors, first element in pair is the image name, second element is an array with vector coordinates
	std::vector< std::pair<std::string, std::array<double, 3> > > Errors;

	//distaces between rays and object points
	std::vector<std::pair<std::string, double> >Distances;

	//average of above distances
	double MeanDistance{ 0.0 };

};
