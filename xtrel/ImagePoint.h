#pragma once
#include <string>
class ImagePoint
{
public:
	ImagePoint(std::string Name_, std::string ImageName_, double X_, double Y_);
	ImagePoint(std::string Name_, std::string ImageName_, double X_, double Y_, int Status_);
	void scale(double s);
	void scale_back(double s);

	std::string Name;
	std::string ImageName;
	//coordinates:
	double X = 0.0; 
	double Y = 0.0;
	//measurement_errors
	double MX = 0.5;
	double MY = 0.5;
	//residuals
	double VX = 0.0;
	double VY = 0.0;

	int Status = 1; //0 - do not use, 1 - use for estimation

};

