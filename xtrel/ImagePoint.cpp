#include "pch.h"
#include "ImagePoint.h"


ImagePoint::ImagePoint(std::string name_, std::string image_name_, double X_, double Y_): 
	Name(name_),
	ImageName(image_name_),
	X(X_),
	Y(Y_)
{ }

ImagePoint::ImagePoint(std::string name_, std::string image_name_, double X_, double Y_, int Status_) :
	Name(name_),
	ImageName(image_name_),
	X(X_),
	Y(Y_),
	Status(Status_)
{ }

void ImagePoint::scale(double s)
{
	X *= s;
	Y *= s;
	MX *= s;
	MY *= s;
	VX *= s;
	VY *= s;
}

void ImagePoint::scale_back(double s)
{
	X /= s;
	Y /= s;
	MX /= s;
	MY /= s;
	VX /= s;
	VY /= s;
}

