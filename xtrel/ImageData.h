#pragma once
#include "EO.h"
class ImageData
{
public:
	ImageData();
	ImageData(EO &eo, std::string& name, std::string& camera_file, std::string& camera_name);
	ImageData(EO &eo, std::string& name, bool observedPosition, bool observedOrientation, std::string& camera_file, std::string& camera_name);
	~ImageData();
	EO EOApproximated;
	EO EOObserved;
	EO EOAdjusted;
	
	bool observedPosition{ false };
	bool observedOrientation{ false };
	
	std::string CameraFile{ "" };
	std::string CameraName{ "" };
	std::string Name{ "" };
	
	double AnglesResiduals[3] = { 0.0, 0.0, 0.0 };
	double CoordsResiduals[3] = { 0.0, 0.0, 0.0 };

	unsigned int NumOfPoints{ 0 };

	void setEOObserved(const EO& eo);
	void setParametrization(const std::string& parametrization);

private:
};

