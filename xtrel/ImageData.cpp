#include "pch.h"
#include "ImageData.h"


ImageData::ImageData()
{
}

ImageData::ImageData(EO & eo, std::string & name, std::string & camera_file, std::string & camera_name):
	EOApproximated(eo), EOObserved(eo), Name(name), CameraFile(camera_file), CameraName(camera_name)
{
}

ImageData::ImageData(EO &eo, std::string& name, bool observedPosition, bool observedOrientation, std::string& camera_file, std::string& camera_name) :
	EOApproximated(eo),
	EOObserved(eo),
	observedPosition(observedPosition),
	observedOrientation(observedOrientation),
	Name(name),
	CameraFile(camera_file),
	CameraName(camera_name)
{
}

ImageData::~ImageData()
{
}

void ImageData::setEOObserved(const EO & eo)
{
	EOObserved = eo;
}

void ImageData::setParametrization(const std::string & parametrization)
{
	EOApproximated.RotParametrization = parametrization;
	EOObserved.RotParametrization = parametrization;
	EOAdjusted.RotParametrization = parametrization;
}
