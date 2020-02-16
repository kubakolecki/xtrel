#include "pch.h"
#include "ImageData.h"


ImageData::ImageData()
{
}

ImageData::ImageData(EO & eo, std::string & name, std::string & camera_file, std::string & camera_name):
	EOApproximated(eo), Name(name), CameraFile(camera_file), CameraName(camera_name)
{
}


ImageData::~ImageData()
{
}
