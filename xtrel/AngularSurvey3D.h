#pragma once
class AngularSurvey3D
{
public:
	AngularSurvey3D();
	AngularSurvey3D(
		std::string IdLeft_,
		std::string IdCenter_,
		std::string IdRight_,
		double Hz_,
		double V_);
	~AngularSurvey3D();

	//identifiers of controll points
	//exectly one of the following: Idx[0] or Ids[2] has to be id of not geodetic controll point
	std::array<std::string, 3> Ids;

	int AdjustedPointId{ -1 }; //id of point that subjects to adjustment,
	//it shoudl be either left or right point,i.e. AdjustedPointId == 0 or AdjustedPointId == 2

	double Hz { 0.0 }; //value of horizontal angle (in radians)
	double V{ 0.0 }; //value of angle from zenith (in radians)

	double Residuals[2] = { 0.0, 0.0 };

};

