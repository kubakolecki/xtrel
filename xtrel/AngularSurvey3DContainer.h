#pragma once
#include "AngularSurvey3D.h"
class AngularSurvey3DContainer
{
public:
	AngularSurvey3DContainer();
	~AngularSurvey3DContainer();
	bool read_from_file(std::string filename);
	void print_in_console();
	std::vector<AngularSurvey3D> Data;

	
};

