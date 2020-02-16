#pragma once
#include "EO.h"
class ImageData
{
public:
	ImageData();
	ImageData(EO &eo, std::string& name, std::string& camera_file, std::string& camera_name);
	~ImageData();
	EO EOApproximated;
	EO EOObserved;
	EO EOAdjusted;
	std::string CameraFile{ "" };
	std::string CameraName{ "" };
	std::string Name{ "" };

	unsigned int NumOfPoints{ 0 };

private:
};

