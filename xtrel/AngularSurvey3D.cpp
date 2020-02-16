#include "pch.h"
#include "AngularSurvey3D.h"


AngularSurvey3D::AngularSurvey3D()
{
}

AngularSurvey3D::AngularSurvey3D(
	std::string IdLeft_,
	std::string IdCenter_,
	std::string IdRight_,
	double Hz_,
	double V_) :
	Hz(Hz_), V(V_)
{
	Ids[0] = IdLeft_;
	Ids[1] = IdCenter_;
	Ids[2] = IdRight_;
}


AngularSurvey3D::~AngularSurvey3D()
{
}
